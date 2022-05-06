import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

from riptide_msgs2.msg import KillSwitchReport, FirmwareState
from std_msgs.msg import UInt8

import ctypes

from enum import Enum
import gpiod

RED_PIN = 21
GREEN_PIN = 26
YELLOW_PIN = 20
KILL_SWITCH_PIN = 19
REPORT_REQ_PIN = 16
sensePin = 10

GREEN_INIT_HOLD = 40
NACK_TIMEOUT = Duration(nanoseconds= 5 * 10e7)
RE_INIT_TIMEOUT = Duration(seconds=10)

REQUIRED_NODES = [
    'physical_kill_switch',
    'coprocessor_node'
]

class KillLedMode(Enum):
    ALL_OFF = 0
    ALL_SOLID = 1
    CHASE_MODE = 2
    SOLID_RED = 3
    BLINK_RED = 4
    SOLID_YELLOW = 5
    BLINK_YELLOW = 6
    SOLID_GREEN = 7
    BLINK_GREEN = 8
    BLINK_RED_GREEN = 9
    BLINK_ALL = 10

class PhysicalKillState(Enum):
    INIT = 0
    ROBOT_NOT_PRESENT = 1
    ROBOT_ALIVE = 2
    ROBOT_KILL_SENT = 3
    ROBOT_KILL_NACK = 4
    ROBOT_KILLED = 5
    KILL_NOT_REQ = 6



class PhysicalKillSwitch(Node):
    def __init__(self):
        super().__init__('physical_kill_switch')

        self.create_subscription(FirmwareState, 'state/firmware', self.firmware_state_callback, qos_profile_sensor_data)
        self.reportPublisher = self.create_publisher(KillSwitchReport, 'control/software_kill', qos_profile_system_default)

        self.killStatePub = self.create_publisher(UInt8, 'physical_kill_state', qos_profile_system_default)

        self.refreshTimer = self.create_timer(0.05, self.refreshCallback)
        self.blinkTimer = self.create_timer(0.5, self.blinkCallback)
        self.printHoldTimer = self.create_timer(1.0, self.printCallback)

        # Grab the GPIO controller
        self.gpioChip = gpiod.chip('pinctrl-bcm2711')

        self.redLed = self.gpioChip.get_line(RED_PIN)
        self.yellowLed = self.gpioChip.get_line(YELLOW_PIN)
        self.greenLed = self.gpioChip.get_line(GREEN_PIN)

        # Configure led pins
        ledCFG = gpiod.line_request()
        ledCFG.consumer = 'physical_kill_switch'
        ledCFG.request_type=gpiod.line_request.DIRECTION_OUTPUT
        self.redLed.request(ledCFG)
        self.yellowLed.request(ledCFG)
        self.greenLed.request(ledCFG)

        self.killSwitch = self.gpioChip.get_line(KILL_SWITCH_PIN)
        self.requireSwitch = self.gpioChip.get_line(REPORT_REQ_PIN)

        # Configure switch pins
        inputCFG = gpiod.line_request()
        inputCFG.consumer = 'physical_kill_switch'
        inputCFG.request_type=gpiod.line_request.DIRECTION_INPUT
        inputCFG.flags = gpiod.line_request.FLAG_BIAS_PULL_UP
        self.killSwitch.request(inputCFG)
        self.requireSwitch.request(inputCFG)

        self.get_logger().info('Physical kill switch started')

        self.state = PhysicalKillState.INIT
        self.killLedState = KillLedMode.ALL_OFF

        # internal vars for animating the LEDS and debouncing the buttons/ switches
        self.ledToggle = False
        self.switchDebounce = False
        self.ledCount = 0
        self.ledHold = 0
        self.print = True

        # state information on switches and the last kill time
        self.lastKillState = False
        self.lastRequireState = False
        self.killSentTime = self.get_clock().now()

        # last recieved or sent messages
        self.killReport = KillSwitchReport(sender_id='physical_kill_switch', kill_switch_id=KillSwitchReport.KILL_SWITCH_TOPSIDE_BUTTON)
        self.lastFirmState = FirmwareState()

    def firmware_state_callback(self, msg: FirmwareState):
        self.lastFirmState = msg
    
    def refreshCallback(self):
        self.lastKillState = bool(self.killSwitch.get_value())
        self.lastRequireState = bool(self.requireSwitch.get_value())

        if(self.state == PhysicalKillState.INIT):
            self.ledHold = 0 if self.ledHold < 0 else self.ledHold - 1
            # When kill pressed change to green
            if(self.killLedState == KillLedMode.BLINK_YELLOW and self.lastKillState and self.switchDebounce):
                self.switchDebounce = False
                self.killLedState = KillLedMode.SOLID_GREEN
                self.ledHold = GREEN_INIT_HOLD
                self.get_logger().info('Checking online nodes')

            # When require disabled change to yellow
            elif(self.killLedState == KillLedMode.BLINK_ALL and not self.lastRequireState and self.switchDebounce):
                self.switchDebounce = False
                self.killLedState = KillLedMode.BLINK_YELLOW
                self.get_logger().warning('Press and release kill button to proceed with initialization')
            
            # Show all blinking by default
            elif(self.killLedState == KillLedMode.ALL_OFF):
                self.killLedState = KillLedMode.BLINK_ALL
                self.get_logger().warning('Toggle requirement switch to proceed with initialization')
            
            # when green we have tested the remote and we can exit init to robot not present
            elif(self.killLedState == KillLedMode.SOLID_GREEN and self.ledHold == 0):
                self.switchDebounce = False
                self.state = PhysicalKillState.ROBOT_NOT_PRESENT

        elif(self.state == PhysicalKillState.ROBOT_NOT_PRESENT):
            self.killLedState = KillLedMode.BLINK_RED
            self.killReport.switch_needs_update = False
            self.killReport.switch_asserting_kill = True

            # check rosnode list to see if copro and controller are up
            if(self.checkAlive()):
                self.get_logger().warning('All required nodes online. Physical kill switch ready!')
                self.state = PhysicalKillState.ROBOT_ALIVE

        elif(self.state == PhysicalKillState.ROBOT_ALIVE):
            self.killLedState = KillLedMode.SOLID_GREEN
            self.killReport.switch_needs_update = True
            self.killReport.switch_asserting_kill = False

            # if the requirement is disabled goto requirement disabled state
            if(not self.lastRequireState):
                self.state = PhysicalKillState.KILL_NOT_REQ

            # handle kill pressed
            if(self.lastKillState):
                self.state = PhysicalKillState.ROBOT_KILL_SENT
                self.killSentTime = self.get_clock().now()
                self.switchDebounce = False

            # since we are not the only kill, make sure the robot is alive
            if(self.iskilled()):
                self.state = PhysicalKillState.ROBOT_KILLED

            # handle case where code dies
            if(not self.checkAlive()):
               self.state = PhysicalKillState.ROBOT_NOT_PRESENT

        elif(self.state == PhysicalKillState.KILL_NOT_REQ):
            self.killLedState = KillLedMode.BLINK_RED_GREEN
            self.killReport.switch_needs_update = False
            self.killReport.switch_asserting_kill = False

            if(self.print):
                self.get_logger().warning('Kill switch heartbeat not required!')
                self.print = False

            # if require has been re-enabled turn it back on
            if(self.lastRequireState):
                self.get_logger().warning('Requiring kill switch heartbeat enabled!')
                self.state = PhysicalKillState.ROBOT_ALIVE

            # handle kill pressed
            if(self.lastKillState):
                self.state = PhysicalKillState.ROBOT_KILL_SENT
                self.killSentTime = self.get_clock().now()
                self.switchDebounce = False

            # since we are not the only kill, make sure the robot is alive
            if(self.iskilled()):
                self.state = PhysicalKillState.ROBOT_KILLED

            # handle robot code death gracefully
            if(self.checkAlive()):
                self.state = PhysicalKillState.ROBOT_NOT_PRESENT

        elif(self.state == PhysicalKillState.ROBOT_KILL_SENT):
            self.killLedState = KillLedMode.SOLID_YELLOW
            self.killReport.switch_asserting_kill = True

            # when robot acks head to killed state
            if(self.iskillAsserted()):
                self.state = PhysicalKillState.ROBOT_KILLED

            elif(self.get_clock().now() - self.killSentTime > NACK_TIMEOUT):
                self.get_logger().error(f'Robot did not acknowldge kill within {NACK_TIMEOUT}')
                self.state = PhysicalKillState.ROBOT_KILL_NACK

        elif(self.state == PhysicalKillState.ROBOT_KILLED):
            if(self.print):
                self.get_logger().error('Robot killed')
                self.print = False

            self.killLedState = KillLedMode.SOLID_RED

            # we can now go back to un killed
            if(not self.iskilled() and not self.lastKillState and self.switchDebounce):
                self.get_logger().warning('Kill switch cleared! robot re-enabled!')
                self.state = PhysicalKillState.ROBOT_ALIVE
            # handle when robot code goes down while disabled
            if(self.checkAlive()):
                self.state = PhysicalKillState.ROBOT_NOT_PRESENT
        # this is very very bad
        elif(self.state == PhysicalKillState.ROBOT_KILL_NACK):
            self.killLedState = KillLedMode.BLINK_YELLOW

            # when robot acks head to killed state
            if(self.iskillAsserted()):
                self.state = PhysicalKillState.ROBOT_KILLED

            # the robot might have gone down, go back to no robot
            elif(self.get_clock().now() - self.killSentTime > RE_INIT_TIMEOUT):
                self.get_logger().error('Waiting for robot to acknowledge kill timed out. Kill switch re-setting to robot dead')
                self.state = PhysicalKillState.ROBOT_NOT_PRESENT

        # Handle led modes
        self.animateKillLeds()

        # publish state debug info
        stateMsg = UInt8(data=self.state.value)
        self.killStatePub.publish(stateMsg)

        # publish the kill switch report
        self.reportPublisher.publish(self.killReport)

    def blinkCallback(self):
        self.ledToggle = True
        self.switchDebounce = True

    def printCallback(self):
        self.print = True

    def animateKillLeds(self):
        if(self.killLedState == KillLedMode.ALL_OFF):
            self.applyLeds([0, 0, 0], False)

        elif(self.killLedState == KillLedMode.ALL_SOLID):
            self.applyLeds([1, 1, 1], False)

        elif(self.killLedState == KillLedMode.BLINK_ALL):
            self.applyLeds([self.ledCount % 2, self.ledCount % 2, self.ledCount % 2], True)

        elif(self.killLedState == KillLedMode.CHASE_MODE):
            self.applyLeds([int(self.ledCount == 1), int(self.ledCount == 2), int(self.ledCount == 3)], True)

        elif(self.killLedState == KillLedMode.BLINK_RED_GREEN):
            self.applyLeds([self.ledCount % 2, 0, self.ledCount % 2], True)

        elif(self.killLedState == KillLedMode.SOLID_GREEN):
            self.applyLeds([0, 0, 1], False)

        elif(self.killLedState == KillLedMode.BLINK_GREEN):
            self.applyLeds([0, 0, self.ledCount % 2], True)

        if(self.killLedState == KillLedMode.SOLID_YELLOW):
            self.applyLeds([0, 1, 0], False)

        if(self.killLedState == KillLedMode.BLINK_YELLOW):
            self.applyLeds([0, self.ledCount % 2, 0], True)

        if(self.killLedState == KillLedMode.SOLID_RED):
            self.applyLeds([1, 0, 0], False)

        if(self.killLedState == KillLedMode.BLINK_RED):
            self.applyLeds([self.ledCount % 2, 0, 0], True)                

    def applyLeds(self, values, isBlink: bool):
        if(len(values) != 3):
            self.get_logger().error(f'Applied led animation with incorrect length {len(values)}')
            return

        if(isBlink and self.ledToggle):
            self.ledCount = 0 if self.ledCount > 2 else self.ledCount + 1 # count zero through three
            self.ledToggle = False

            self.redLed.set_value(values[0])
            self.yellowLed.set_value(values[1])
            self.greenLed.set_value(values[2])

        if(not isBlink):
            self.redLed.set_value(values[0])
            self.yellowLed.set_value(values[1])
            self.greenLed.set_value(values[2])

    def releaseGpio(self):
        # Release LEDS
        self.redLed.release()
        self.yellowLed.release()
        self.greenLed.release()

        # Release switches
        self.requireSwitch.release()
        self.killSwitch.release()

    def checkAlive(self):
        nodes = self.get_node_names()
        for name in REQUIRED_NODES:
            if(not name in nodes):
                if(self.print):
                    self.get_logger().warning(f'Could not find node \'{name}\' in {nodes}')
                    self.print = False

                return False

        return True

    def iskillAsserted(self):
        # this is a conversion because bitwise not in python is stupid...
        index = int(ctypes.c_uint32(4).value)
        asserted = self.lastFirmState.kill_switches_asserting_kill
        timeout = self.lastFirmState.kill_switches_timed_out

        isAsserted = asserted & index
        isTimeout = timeout & index

        # self.get_logger().info(f'killassert index {index} assert: {asserted}, timeout: {timeout}, killed:{isAsserted}, timedout:{isTimeout}')

        return isAsserted > 0 or isTimeout > 0

    def iskilled(self):
        # this is a conversion because bitwise not in python is stupid...
        index = int(ctypes.c_uint32(~4).value)
        asserted = self.lastFirmState.kill_switches_asserting_kill
        timeout = self.lastFirmState.kill_switches_timed_out

        isAsserted = index & asserted
        isTimeout = index & timeout

        assertB = format(isAsserted, 'b')
        timeoutB = format(isTimeout, 'b')
        indexB = format(index, 'b')

        # self.get_logger().info(f'killed index {indexB} assert: {assertB}, timeout: {timeoutB}, killed:{isAsserted}, timedout:{isTimeout}')

        return isAsserted > 0 or isTimeout  > 0


def main(args=None):
    rclpy.init(args=args)

    physical_kill_switch_node = PhysicalKillSwitch()

    rclpy.spin(physical_kill_switch_node)

    # Shut down GPIO access
    physical_kill_switch_node.releaseGpio()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
