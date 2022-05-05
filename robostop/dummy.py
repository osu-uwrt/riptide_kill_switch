import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

from riptide_msgs2.msg import KillSwitchReport, FirmwareState

class Dummy(Node):
    def __init__(self):
        super().__init__('coprocessor_node')

        self.killSubscriber = self.create_subscription(KillSwitchReport, 'control/software_kill', self.killReportCallback, qos_profile_system_default)
        self.killReport = KillSwitchReport()

        self.loopTimer = self.create_timer(0.40, self.timerCallback)

        self.firmStatePub = self.create_publisher(FirmwareState, 'state/firmware', qos_profile_system_default)
        self.firmState = FirmwareState()

    def killReportCallback(self, msg):
        self.killReport = msg
        # print('got new kill info')

    def timerCallback(self):

        self.firmState.kill_switches_enabled = 1 if self.killReport.sender_id != '' else 0
        self.firmState.kill_switches_needs_update = 1 if self.killReport.switch_needs_update else 0
              
        self.firmState.kill_switches_asserting_kill = 1 if self.killReport.switch_asserting_kill else 0

        print(self.firmState)

        self.firmStatePub.publish(self.firmState)

def main(args=None):
    rclpy.init(args=args)

    node = Dummy()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()