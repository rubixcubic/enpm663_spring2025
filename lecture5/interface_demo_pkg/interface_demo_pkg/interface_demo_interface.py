from rclpy.node import Node
from enpm663_interfaces.msg import DayTimeInfo
import random
from rclpy.clock import Clock

class InterfaceDemoInterface(Node):
    """
    Class to test message interface
    """

    def __init__(self, node_name):
        super().__init__(node_name)
        self._pub = self.create_publisher(DayTimeInfo, "day_time_info", 10)
        self._pub_timer = self.create_timer(2, self._timer_callback)
        self._msg = DayTimeInfo()
        self.time_of_day_list = [DayTimeInfo.DAY, DayTimeInfo.NIGHT]
        self.day_of_week_list = [
            DayTimeInfo.MONDAY,
            DayTimeInfo.TUESDAY,
            DayTimeInfo.WEDNESDAY,
            DayTimeInfo.THURSDAY,
            DayTimeInfo.FRIDAY,
            DayTimeInfo.SATURDAY,
            DayTimeInfo.SUNDAY
            ]
        
        self.get_logger().info(f"{node_name} initialized")


    def _timer_callback(self):
        # generate a random float number for speed between 20 and 80
        random_time_of_day = random.choice(self.time_of_day_list)
        random_day_of_week = random.choice(self.day_of_week_list)
        self._msg.time_of_day = random_time_of_day
        self._msg.day_of_week = random_day_of_week
        self._msg.time = Clock().now().to_msg()

        self._pub.publish(self._msg)