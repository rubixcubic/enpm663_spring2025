#!/usr/bin/env python3

from rclpy.node import Node
from enpm663_interfaces.srv import HeatingSystem  # Custom service

class HeatingService(Node):
    def __init__(self):
        super().__init__("heating_service")
        self.srv = self.create_service(HeatingSystem, "adjust_heating", self.handle_heating_request)
        self.get_logger().info("Heating Service Ready")

    def handle_heating_request(self, request, response):
        day_mapping = ["", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"]
        # time_of_day = "DAY" if request.time_of_day == 0 else "NIGHT"
        day_of_week = day_mapping[request.day_of_week]

        # Heating logic
        if request.time_of_day == 1:  # NIGHT
            if request.day_of_week in [1, 2, 3, 4, 5]:  # Weekdays
                action = "Increasing heating (People home at night)"
            else:  # Weekends
                action = "Maintaining warm temperature for comfort"
        else:  # DAY
            if request.day_of_week in [1, 2, 3, 4, 5]:  # Weekdays
                action = "Lowering heating (People at work)"
            else:  # Weekends
                action = "Maintaining moderate heating"

        response.success = True
        response.message = f"{action} on {day_of_week}."
        self.get_logger().info(response.message)
        return response

# def main():
#     rclpy.init()
#     node = HeatingService()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
