import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from lifecycle_msgs.msg import State as StateMsg


class SimpleLifecycleDemo(LifecycleNode):
    def __init__(self):
        super().__init__('simple_lifecycle_node')

        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.iteration_count = 0  # Counts timer ticks
        self.current_state = StateMsg.PRIMARY_STATE_UNCONFIGURED  # Track lifecycle state
        
        # Timer that triggers every second
        self.timer = self.create_timer(1.0, self.timer_callback, callback_group=self.callback_group)

    def on_configure(self, state) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring node...")
        self.current_state = StateMsg.PRIMARY_STATE_INACTIVE
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info("Activating node...")
        self.current_state = StateMsg.PRIMARY_STATE_ACTIVE
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating node...")
        self.current_state = StateMsg.PRIMARY_STATE_INACTIVE
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up node...")
        self.current_state = StateMsg.PRIMARY_STATE_UNCONFIGURED
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down node...")
        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        """Changes the node state every 10 timer iterations."""
        self.iteration_count += 1

        if self.iteration_count % 10 == 0:  # Change state every 10 iterations
            self.get_logger().info(f"Iteration {self.iteration_count}: Changing state...")

            if self.current_state == StateMsg.PRIMARY_STATE_UNCONFIGURED:
                self.trigger_configure()
            elif self.current_state == StateMsg.PRIMARY_STATE_INACTIVE:
                self.trigger_activate()
            elif self.current_state == StateMsg.PRIMARY_STATE_ACTIVE:
                self.trigger_deactivate()
            elif self.current_state == StateMsg.PRIMARY_STATE_FINALIZED:
                self.get_logger().info("Node is finalized, stopping timer...")
                self.timer.cancel()  # Stop the timer when shutdown is reached
            else:
                self.trigger_cleanup()
