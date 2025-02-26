from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_msgs.msg import String
from rclpy.lifecycle import State
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State as StateMsg

import rclpy



class ControllerDemoInterface(LifecycleNode):
    def __init__(self, sensor_node, nav_node, actuator_node):
        super().__init__('controller_node')
        self._sensor_node = sensor_node
        self._nav_node = nav_node
        self._actuator_node = actuator_node
        self._command_sub = None
        self._current_state = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'configure'")
        self._command_sub = self.create_subscription(
            String, '/robot_command', self.command_callback, 10)
        # Configure other nodes
        self.trigger_transition(self._sensor_node, 'configure')
        self.trigger_transition(self._nav_node, 'configure')
        self.trigger_transition(self._actuator_node, 'configure')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'activate'")
   
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'deactivate'")
   
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'shutdown'")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("ControllerNode: Cleaning up controller...")
        if self._command_sub is not None:
            self.destroy_subscription(self._command_sub)
        return TransitionCallbackReturn.SUCCESS

    def trigger_transition(self, node: LifecycleNode, transition):
        """Helper method to trigger a transition on a lifecycle node."""
        self._current_state = get_lifecycle_state(node)

        if transition == 'configure' and self._current_state == StateMsg.PRIMARY_STATE_UNCONFIGURED:
            node.trigger_configure()
        elif transition == 'activate' and self._current_state == StateMsg.PRIMARY_STATE_INACTIVE:
            node.trigger_activate()
        elif transition == 'deactivate' and self._current_state == StateMsg.PRIMARY_STATE_ACTIVE:
            node.trigger_deactivate()
        elif transition == 'cleanup' and self._current_state == StateMsg.PRIMARY_STATE_INACTIVE:
            node.trigger_cleanup()
        elif transition == 'shutdown':
            node.trigger_shutdown()
        else:
            self.get_logger().warn(f"Invalid transition '{transition}' from state '{self._current_state}'")


    def command_callback(self, msg):
        nodes = [self, self._sensor_node, self._nav_node, self._actuator_node]
        self.get_logger().info(f"label: {self.get_state().label}")
        if msg.data == "start" and self.get_state().label == 'inactive':
            self.get_logger().info("ControllerNode: Starting robot...")
            for node in nodes:
                self.trigger_transition(node, 'activate')
        elif msg.data == "pause" and self.get_state().label == 'active':
            self.get_logger().info("ControllerNode: Pausing robot...")
            for node in nodes:
                self.trigger_transition(node, 'deactivate')
        elif msg.data == "shutdown" and self.get_state().label == 'inactive':
            self.get_logger().info("ControllerNode: Shutting down robot...")
            for node in nodes:
                self.trigger_transition(node, 'cleanup')
                self.trigger_transition(node, 'shutdown')
                
def get_lifecycle_state(node):
    """Queries the lifecycle state of a node using the lifecycle service."""
    client = node.create_client(GetState, f"{node.get_name()}/get_state")

    if not client.wait_for_service(timeout_sec=2.0):
        node.get_logger().error(f"Lifecycle state service for {node.get_name()} not available!")
        return None

    req = GetState.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        state_id = future.result().current_state.id
        node.get_logger().info(f"Lifecycle state of {node.get_name()}: {state_id}")
        return state_id
    else:
        node.get_logger().error(f"Failed to get lifecycle state for {node.get_name()}")
        return None