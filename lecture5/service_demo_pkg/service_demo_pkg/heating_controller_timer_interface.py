from rclpy.node import Node
from enpm663_interfaces.msg import DayTimeInfo
from enpm663_interfaces.srv import HeatingSystem
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class HeatingControllerTimer(Node):
    """
    Class to demonstrate the use of a service client.
    This class consists of two service clients, one synchronous and one asynchronous.
    Both clients are called in the timer callback function.
    """

    # Class attributes
    mutex_group1 = MutuallyExclusiveCallbackGroup()
    mutex_group2 = MutuallyExclusiveCallbackGroup()

    def __init__(self, node_name):
        super().__init__(node_name)

        # These attributes are set in the subscriber callback function
        # and are used in the timer callback function
        self._time_of_day = False
        self._day_of_week = False

        
        # Create a subscriber to the day_time_info topic
        self._subscriber = self.create_subscription(
            DayTimeInfo,  # Message type
            "day_time_info",  # Topic name
            self._day_time_info_cb,  # Callback function
            100,  # QoS profile,
            callback_group=HeatingControllerTimer.mutex_group1,  # Callback group
        )
        

        self._timer_async_call = self.create_timer(
            1,  # Timer period in seconds
            self._timer_async_call_cb,  # Callback function
            callback_group=HeatingControllerTimer.mutex_group1,  # Callback group
        )

        # Create a timer to call the service synchronously every second
        self._timer_sync_call = self.create_timer(
            1,  # Timer period in seconds
            self._timer_sync_call_cb,  # Callback function
            callback_group=HeatingControllerTimer.mutex_group1,  # Callback group
        )

        # Create a client to call the adjust_heating service
        # This client will be called synchronously
        # The callback group ensures that the callback function is called in a mutually exclusive manner
        self._sync_client = self.create_client(
            HeatingSystem,  # Service type
            "adjust_heating",  # Service name
            callback_group=HeatingControllerTimer.mutex_group2,  # Callback group
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

        self._logger.info("client created")

    def _day_time_info_cb(self, msg: DayTimeInfo):
        """
        Callback function for the subscriber.
        This function is called whenever a message is received on the day_time_info topic.
        """
      
        # store each field from DayTimeInfo
        self._time_of_day = msg.time_of_day
        self._day_of_week = msg.day_of_week

    def _timer_sync_call_cb(self):
        """
        Callback function for a timer which calls the service synchronously.
        This function is called every second.
        """

        # Call the synchronous service client
        self._send_sync_request()

    def _timer_async_call_cb(self):
        """
        Callback function for a timer which calls the service asynchronously.
        This function is called every second.
        """
        # Call the asynchronous service client
        self._send_async_request()

    def _send_async_request(self):
        """
        Function to send an asynchronous request to the service.
        """
        while not self._async_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        # Fill in the request object
        self._request.time_of_day = self._time_of_day
        self._request.day_of_week = self._day_of_week

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

    def _send_sync_request(self):
        """
        Send a request synchronously to the server
        """
        while not self._sync_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        # Fill in the request object
        self._request.time_of_day = self._time_of_day
        self._request.day_of_week = self._day_of_week

        response = self._sync_client.call(self._request)
        self.get_logger().info(f"🔵 TimerSyncResult: {response.message}")