from rclpy.node import Node
from enpm663_interfaces.msg import DayTimeInfo
from enpm663_interfaces.srv import HeatingSystem
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class HeatingControllerDirect(Node):
    """
    Class to demonstrate the use of a service client.
    This class consists of two service clients, one synchronous and one asynchronous.
    Both clients are called in the subscriber callback function.
    """

    # class attribute
    mutex_group = MutuallyExclusiveCallbackGroup()

    def __init__(self, node_name):
        super().__init__(node_name)

        # Create a subscriber to the vehicle_status topic
        self._subscriber = self.create_subscription(
            DayTimeInfo,  # Message type
            "day_time_info",  # Topic name
            self._day_time_info_cb,  # Callback function
            100,  # QoS profile,  # Callback group
        )

        # Create a client to call the get_speed_profile service
        # This client will be called synchronously
        # The callbacks associated with this service client will be executed in a mutually exclusive manner, ensuring thread safety.
        # Without this callback group, there will be a deadlock when the service client is called synchronously.
        self._sync_client = self.create_client(
            HeatingSystem,  # Service type
            "adjust_heating",  # Service name
            callback_group=HeatingControllerDirect.mutex_group,  # Callback group
        )

        # Create a client to call the get_speed_profile service
        # This client will be called asynchronously
        # A callback group is not needed
        self._async_client = self.create_client(HeatingSystem, "adjust_heating")

        # Wait for the service to be available
        while not self._sync_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        # Create a request object
        self._request = HeatingSystem.Request()
        # Sanity check
        self._logger.info("client created")

    def _day_time_info_cb(self, msg: DayTimeInfo):
        """
        Callback function for the subscriber.
        This function is called whenever a message is received on the day_time_info topic.
        """
        time_of_day = msg.time_of_day  # DAY or NIGHT
        day_of_week = msg.day_of_week  # Monday, Tuesday, etc.

        self.get_logger().info(f"Received day/time info: {time_of_day}, {day_of_week}. Calling service...")

        # Call the service clients
        self._send_async_request(time_of_day, day_of_week)
        self._send_sync_request(time_of_day, day_of_week)

    def _send_async_request(self, time_of_day: int, day_of_week: int) -> None:
        """
        Function to send an asynchronous request to the service.
        """

        # Wait for the service to be available
        while not self._async_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        # Fill in the request object
        self._request.time_of_day = time_of_day
        self._request.day_of_week = day_of_week

        # Call the service asynchronously
        future = self._async_client.call_async(self._request)

        # Add a callback function to be called when the future is complete
        future.add_done_callback(self.future_callback)

    def future_callback(self, future):
        """
        Callback function for the future object

        Args:
            future (Future): A future object
        """
        self.get_logger().info(f"🔴 TimerAsyncResult: {future.result().message}")

    def _send_sync_request(self, time_of_day: int, day_of_week: int) -> None:
        """
        Send a request synchronously to the server

        Args:
            speed (float): The speed value to be sent in the request
        """

        # Wait for the service to be available
        while not self._sync_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        # Fill in the request object
        self._request.time_of_day = time_of_day
        self._request.day_of_week = day_of_week

        response = self._sync_client.call(self._request)
        self.get_logger().info(f"🔵 TimerSyncResult: {response.message}")
