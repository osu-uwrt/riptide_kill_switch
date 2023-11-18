import os
import time
import rclpy
import threading
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

from std_msgs.msg import Bool
from riptide_msgs2.msg import DshotPartialTelemetry, FirmwareStatus, KillSwitchReport

from riptide_hardware2.common import ExpiringMessage, Mk2Board

from enum import Enum
import gpiod

# Physical Kill
RED_PIN = 21
GREEN_PIN = 26
YELLOW_PIN = 20
KILL_SWITCH_PIN = 19
REPORT_REQ_PIN = 16
# SENSE_PIN = 10  # This pin didn't end up getting routed on the current kill switch

# Stack Light
STACK_RED = 18
STACK_GREEN = 24
STACK_YELLOW = 23
STACK_BUZZER = 25

# Timeouts
FIRMWARE_MESSAGE_TIMEOUT_SEC = 2.0
NACK_TIMEOUT_SEC = 2.0

class DebouncedInput:
    debounceTimeSeconds = 0.5

    def __init__(self, input: 'gpiod.line', preferred_value: bool):
        assert type(preferred_value) == bool, "Preferred value must be bool"

        self.input = input
        self.preferredValue = preferred_value
        self.lastChangeTime = time.time()
        self.lastValue = bool(input.get_value())

    def getState(self) -> bool:
        """Gets the current debounced state of the GPIO line.
        Note this must be called frequently enough to ensure debounce logic functions as expected.
        """
        value = bool(self.input.get_value())
        if value != self.lastValue:
            self.lastChangeTime = time.time()
        self.lastValue = value

        if time.time() - self.lastChangeTime < self.debounceTimeSeconds:
            return self.preferredValue
        else:
            return value


class ACKState(Enum):
    STATE_OKAY = 0
    STATE_WAITING = 1
    STATE_NACK = 2


class LEDMode(Enum):
    def __new__(cls, red_en: bool, yellow_en: bool, green_en: bool, mode_blink: bool, mode_chase: bool) -> 'LEDMode':
        assert not (mode_blink and mode_chase), "Mode cannot be both blink and chase"
        obj = object.__new__(cls)
        obj._value_ = ((int(red_en) << 0) | (int(yellow_en) << 1) | (int(green_en) << 2) |
                        (int(mode_blink) << 3) | (int(mode_chase) << 4))
        obj._red_en_ = red_en
        obj._yellow_en_ = yellow_en
        obj._green_en_ = green_en
        obj._mode_blink_ = mode_blink
        obj._mode_chase_ = mode_chase
        obj._chase_sequence_ = []
        if red_en:
            obj._chase_sequence_.append((True, False, False))
        if yellow_en:
            obj._chase_sequence_.append((False, True, False))
        if green_en:
            obj._chase_sequence_.append((False, False, True))
        assert not (mode_chase and len(obj._chase_sequence_) == 0), "Chase requires at least one mode"

        return obj

    @property
    def red_en(self) -> bool:
        return self._red_en_

    @property
    def yellow_en(self) -> bool:
        return self._yellow_en_

    @property
    def green_en(self) -> bool:
        return self._green_en_

    @property
    def mode_blink(self) -> bool:
        return self._mode_blink_

    @property
    def mode_chase(self) -> bool:
        return self._mode_chase_

    @property
    def chase_sequence(self) -> list[tuple[bool]]:
        return self._chase_sequence_

    def computeColors(self) -> tuple[bool]:
        if self.mode_chase:
            seq_len = len(self.chase_sequence)
            seq_id = int(time.time()) % seq_len
            return self.chase_sequence[seq_id]
        else:
            if int(time.time()) % 2 or not self.mode_blink:
                return (self.red_en, self.yellow_en, self.green_en)
            else:
                return (False, False, False)

    # Format: ed_en, yellow_en, green_en, mode_blink, mode_chase
    ALL_OFF = False, False, False, False, False
    ALL_SOLID = True, True, True, False, False
    CHASE_MODE = True, True, True, False, True
    SOLID_RED = True, False, False, False, False
    BLINK_RED = True, False, False, True, False
    SOLID_YELLOW = False, True, False, False, False
    BLINK_YELLOW = False, True, False, True, False
    SOLID_GREEN = False, False, True, False, False
    BLINK_GREEN = False, False, True, True, False
    BLINK_RED_GREEN = True, False, True, True, False
    BLINK_ALL = True, True, True, True, False
    CHASE_RED_GREEN = True, False, True, False, True
    CHASE_YELLOW_GREEN = False, True, True, False, True


class GPIOController:
    gpioConsumer = 'physical_kill_switch'

    def __init__(self):
        # Grab the GPIO controller
        self.gpioChip = gpiod.chip('gpiochip0')

        self.stackRed = self.gpioChip.get_line(STACK_RED)
        self.stackYellow = self.gpioChip.get_line(STACK_YELLOW)
        self.stackGreen = self.gpioChip.get_line(STACK_GREEN)
        self.stackBuzzer = self.gpioChip.get_line(STACK_BUZZER)

        self.redLed = self.gpioChip.get_line(RED_PIN)
        self.yellowLed = self.gpioChip.get_line(YELLOW_PIN)
        self.greenLed = self.gpioChip.get_line(GREEN_PIN)

        # Configure led pins
        ledCFG = gpiod.line_request()
        ledCFG.consumer = self.gpioConsumer
        ledCFG.request_type=gpiod.line_request.DIRECTION_OUTPUT
        self.redLed.request(ledCFG)
        self.yellowLed.request(ledCFG)
        self.greenLed.request(ledCFG)
        self.stackRed.request(ledCFG)
        self.stackYellow.request(ledCFG)
        self.stackGreen.request(ledCFG)
        self.stackBuzzer.request(ledCFG)

        self.killSwitch = self.gpioChip.get_line(KILL_SWITCH_PIN)
        self.requireSwitch = self.gpioChip.get_line(REPORT_REQ_PIN)
        # self.sensePin = self.gpioChip.get_line(SENSE_PIN)

        # Configure switch pins
        inputCFG = gpiod.line_request()
        inputCFG.consumer = self.gpioConsumer
        inputCFG.request_type=gpiod.line_request.DIRECTION_INPUT
        inputCFG.flags = gpiod.line_request.FLAG_BIAS_PULL_UP
        self.killSwitch.request(inputCFG)
        self.requireSwitch.request(inputCFG)
        # self.sensePin.request(inputCFG)

        # Create the debounced inputs
        self.killSwitchDebounced = DebouncedInput(self.killSwitch, True)
        self.requireSwitchDebounced = DebouncedInput(self.requireSwitch, True)
        # self.sensePinDebounced = DebouncedInput(self.sensePin, True)
        self.buzzerThread = None

    def applyKillswitchLeds(self, state: 'LEDMode'):
        red, yellow, green = state.computeColors()
        self.redLed.set_value(red)
        self.yellowLed.set_value(yellow)
        self.greenLed.set_value(green)

    def applyStackLeds(self, state: 'LEDMode'):
        red, yellow, green = state.computeColors()
        self.stackRed.set_value(red)
        self.stackYellow.set_value(yellow)
        self.stackGreen.set_value(green)

    def getKillswitchAssertingKill(self) -> bool:
        return self.killSwitchDebounced.getState()

    def getKillswitchRequiringUpdate(self) -> bool:
        return self.requireSwitchDebounced.getState()

    # def getKillswitchConnected(self) -> bool:
    #     return not self.sensePinDebounced.getState()

    def chirpBuzzer(self):
        self._startBuzzerTask(self._chirpBuzzerTask)

    def alertBuzzer(self):
        self._startBuzzerTask(self._alertBuzzerTask)

    def _startBuzzerTask(self, task):
        if self.buzzerThread is not None:
            if self.buzzerThread.is_alive():
                return
            self.buzzerThread.join()
        self.buzzerThread = threading.Thread(target=task)
        self.buzzerThread.start()

    def _chirpBuzzerTask(self):
        self.stackBuzzer.set_value(1)
        time.sleep(0.01)
        self.stackBuzzer.set_value(0)

    def _alertBuzzerTask(self):
        for _ in range(10):
            self.stackBuzzer.set_value(1)
            time.sleep(0.01)
            self.stackBuzzer.set_value(0)
            time.sleep(0.01)
        self.stackBuzzer.set_value(0)
        time.sleep(0.1)
        self.stackBuzzer.set_value(1)
        time.sleep(0.1)
        self.stackBuzzer.set_value(0)

    """
    Here's a really cool buzzer pattern that sounds like the stack is going to explode. However it would probably be
    unwise to add this into the code for obvious reasons. But if you're messing around with the stack you can try it:

        for _ in range(5):
            gpioController.stackBuzzer.set_value(1)
            time.sleep(0.5)
            for _ in range(30):
                gpioController.stackBuzzer.set_value(1)
                time.sleep(0.01)
                gpioController.stackBuzzer.set_value(0)
                time.sleep(0.01)
    """

    def releaseGpio(self):
        self.buzzerThread.join()
        # Release LEDS
        self.redLed.set_value(0)
        self.redLed.release()
        self.yellowLed.set_value(0)
        self.yellowLed.release()
        self.greenLed.set_value(0)
        self.greenLed.release()

        # Release stack light
        self.stackRed.set_value(0)
        self.stackRed.release()
        self.stackYellow.set_value(0)
        self.stackYellow.release()
        self.stackGreen.set_value(0)
        self.stackGreen.release()
        self.stackBuzzer.set_value(0)
        self.stackBuzzer.release()

        # Release inputs
        self.requireSwitch.release()
        self.killSwitch.release()
        # self.sensePin.release()


class KillSwitchSelfcheck:
    def __init__(self, logger):
        self.logger = logger
        self.reset()

    def reset(self):
        self.logger.info("Topside Switch Self Check - Toggle both required and assert into killed/require update position")
        self.assertSeenLow = False
        self.assertSeenHigh = False
        self.requiredSeenLow = False
        self.requiredSeenHigh = False

    def isReady(self) -> bool:
        return self.assertSeenLow and self.assertSeenHigh and self.requiredSeenLow and self.requiredSeenHigh

    def checkPoll(self, physkillRequired: bool, physkillAssertingKill: bool, change_cb) -> 'LEDMode':
        if not self.assertSeenLow and not physkillAssertingKill:
            self.assertSeenLow = True
            if change_cb:
                change_cb()
        elif self.assertSeenLow and not self.assertSeenHigh and physkillAssertingKill:
            self.assertSeenHigh = True
            if change_cb:
                change_cb()
        elif not self.requiredSeenLow and not physkillRequired:
            self.requiredSeenLow = True
            if change_cb:
                change_cb()
        elif self.requiredSeenLow and not self.requiredSeenHigh and physkillRequired:
            self.requiredSeenHigh = True
            if change_cb:
                change_cb()

        # TODO: This should ideally check if its plugged in so it doesn't accidentally selfcheck on insertion and removal
        if self.requiredSeenHigh and self.assertSeenHigh:
            return LEDMode.ALL_SOLID
        elif self.requiredSeenHigh and not self.assertSeenHigh:
            return LEDMode.CHASE_RED_GREEN
        elif self.assertSeenHigh and not self.requiredSeenHigh:
            return LEDMode.CHASE_YELLOW_GREEN
        else:
            return LEDMode.CHASE_MODE


class TopsideKillSwitch(Node):
    authorityBoard = Mk2Board.POWER_BOARD
    firmwareStatusTimeout = 2.0

    def __init__(self, gpioController: 'GPIOController'):
        super().__init__('topside_kill_switch')
        self.get_logger().info('Topside kill switch started')

        self.gpioController = gpioController
        self.switchSelfcheck = KillSwitchSelfcheck(self.get_logger())

        self.create_subscription(FirmwareStatus, 'state/firmware', self.firmware_state_callback, qos_profile_sensor_data)
        self.create_subscription(Bool, 'state/kill', self.kill_state_callback, qos_profile_sensor_data)
        self.create_subscription(DshotPartialTelemetry, 'state/thrusters/telemetry', self.thruster_telemetry_callback, qos_profile_sensor_data)
        self.reportPublisher = self.create_publisher(KillSwitchReport, 'command/software_kill', qos_profile_system_default)

        self.refreshTimer = self.create_timer(0.15, self.refreshCallback)

        # Set last time killswitch matched to current time
        # In case the robot is already up when we come up, make sure we give time for state to stabalize
        self.lastMatchTime = time.time()
        self.ackState = ACKState.STATE_OKAY

        # last recieved or sent messages
        self.killReport = KillSwitchReport(sender_id='topside_kill_' + os.urandom(3).hex(), kill_switch_id=KillSwitchReport.KILL_SWITCH_TOPSIDE_BUTTON)
        self.firmwareStatus = ExpiringMessage(self.get_clock(), self.firmwareStatusTimeout)
        self.movingBrd0Message = ExpiringMessage(self.get_clock(), self.firmwareStatusTimeout)
        self.movingBrd1Message = ExpiringMessage(self.get_clock(), self.firmwareStatusTimeout)
        self.globalKillMessage = ExpiringMessage(self.get_clock(), self.firmwareStatusTimeout)

    def firmware_state_callback(self, msg: FirmwareStatus):
        if msg.client_id != self.authorityBoard.client_id or msg.bus_id != self.authorityBoard.bus_id:
            return

        self.firmwareStatus.update_value(msg)

    def thruster_telemetry_callback(self, msg: DshotPartialTelemetry):
        if msg.start_thruster_num == 0:
            self.movingBrd0Message.update_value(msg.thrusters_moving)
        else:
            self.movingBrd1Message.update_value(msg.thrusters_moving)

    def kill_state_callback(self, msg: Bool):
        self.globalKillMessage.update_value(msg.data)

    def refreshCallback(self):
        self.refreshStackLight()
        self.refreshKillSwitch()

    def refreshStackLight(self):
        if not self.checkAlive():
            stackLedState = LEDMode.ALL_OFF
        else:
            if self.checkMoving():
                stackLedState = LEDMode.SOLID_GREEN
            elif self.checkUnkilled():
                stackLedState = LEDMode.SOLID_YELLOW
            else:
                stackLedState = LEDMode.SOLID_RED

        self.gpioController.applyStackLeds(stackLedState)

    def refreshKillSwitch(self):
        physkillAssertingKill = self.gpioController.getKillswitchAssertingKill()
        physkillRequired = self.gpioController.getKillswitchRequiringUpdate()

        if not self.switchSelfcheck.isReady():
            self.gpioController.applyKillswitchLeds(self.switchSelfcheck.checkPoll(physkillRequired, physkillAssertingKill, self.gpioController.chirpBuzzer))

            # Don't do anything else if the switch isn't initialized
            return


        # Compute our kill report message and light state
        # Note these will be overridden below if the global state doesn't match
        self.killReport.switch_needs_update = physkillRequired
        self.killReport.switch_asserting_kill = physkillAssertingKill
        if physkillAssertingKill:
            killLedState = LEDMode.SOLID_RED
        elif not physkillRequired:
            killLedState = LEDMode.BLINK_RED_GREEN
        else:
            killLedState = LEDMode.SOLID_GREEN


        # Handle robot state now and adjust our status accordingly
        if not self.checkAlive():
            if self.ackState == ACKState.STATE_WAITING:
                self.gpioController.alertBuzzer()

            # Robot not online (or at least firmware isn't) if these aren't present
            killLedState = LEDMode.BLINK_RED

            # Mark us as okay since we lost connection, and won't be able to wait for an ACK to come in
            # Also prevents timeout as soon as it reconnects
            self.lastMatchTime = time.time()
            self.ackState = ACKState.STATE_OKAY
        else:
            if not self.isTopsideKillAssertedMatching(physkillAssertingKill) or \
                    not self.isTopsideRequiringMatching(physkillRequired):
                # Set LED to yellow to show the state isn't reflected yet
                killLedState = LEDMode.SOLID_YELLOW
                if time.time() - self.lastMatchTime > NACK_TIMEOUT_SEC:
                    # If we haven't chirped yet to alert of the NACK, do so now
                    if self.ackState == ACKState.STATE_WAITING:
                        self.gpioController.alertBuzzer()

                    # Set our state
                    killLedState = LEDMode.BLINK_YELLOW
                    self.ackState = ACKState.STATE_NACK
                else:
                    # Set waiting state so it'll chirp if we also loose connection
                    self.ackState = ACKState.STATE_WAITING
            else:
                self.lastMatchTime = time.time()
                self.ackState = ACKState.STATE_OKAY

        # Set the LEDs
        self.gpioController.applyKillswitchLeds(killLedState)

        # publish the kill switch report
        self.reportPublisher.publish(self.killReport)

    def checkAlive(self):
        firmwareMsg = self.firmwareStatus.get_value()
        killMsg = self.globalKillMessage.get_value()
        movingBrd0 = self.movingBrd0Message.get_value()
        movingBrd1 = self.movingBrd1Message.get_value()
        return firmwareMsg is not None and killMsg is not None and movingBrd0 is not None and movingBrd1 is not None

    def checkMoving(self):
        movingBrd0 = self.movingBrd0Message.get_value()
        movingBrd1 = self.movingBrd1Message.get_value()
        return bool(movingBrd0 or movingBrd1)

    def checkUnkilled(self):
        globalKillAsserting = self.globalKillMessage.get_value()
        if globalKillAsserting is None:
            return False

        return bool(not globalKillAsserting)

    def isTopsideRequiringMatching(self, requestedRequiringUpdate: bool) -> bool:
        msg: 'FirmwareStatus' = self.firmwareStatus.get_value()
        if msg is None:
            return False

        id_mask = 1 << self.killReport.kill_switch_id
        topside_enabled = bool(msg.kill_switches_enabled & id_mask)
        topside_requiring_update = bool(msg.kill_switches_needs_update & id_mask)

        return topside_enabled and (topside_requiring_update == requestedRequiringUpdate)

    def isTopsideKillAssertedMatching(self, requestedAssertingKill: bool) -> bool:
        msg: 'FirmwareStatus' = self.firmwareStatus.get_value()
        if msg is None:
            return False

        globalKillAsserting = self.globalKillMessage.get_value()
        if globalKillAsserting is None:
            return False

        id_mask = 1 << self.killReport.kill_switch_id
        topside_enabled = bool(msg.kill_switches_enabled & id_mask)
        topside_timed_out = bool(msg.kill_switches_timed_out & id_mask)
        topside_asserting_kill = bool(msg.kill_switches_asserting_kill & id_mask)

        globalKillMismatch = requestedAssertingKill and not globalKillAsserting

        return topside_enabled and (topside_asserting_kill == requestedAssertingKill) and (not topside_timed_out) and (not globalKillMismatch)


def main(args=None):
    gpioController = GPIOController()

    try:
        rclpy.init(args=args)

        physical_kill_switch_node = TopsideKillSwitch(gpioController)
        rclpy.spin(physical_kill_switch_node)
        rclpy.shutdown()
    finally:
        gpioController.releaseGpio()


if __name__ == '__main__':
    main()
