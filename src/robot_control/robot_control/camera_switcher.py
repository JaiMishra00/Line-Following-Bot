import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import time

class ServoHardwareController(Node):
    def __init__(self):
        super().__init__('servo_hardware_controller')

        # 1. Declare ROS 2 Parameter (Default to '90')
        self.declare_parameter('initial_angle', '90')
        startup_angle = self.get_parameter('initial_angle').get_parameter_value().string_value

        # 2. Initialize Hardware PWM
        try:
            self.factory = PiGPIOFactory()
            self.servo = Servo(18, pin_factory=self.factory, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
            self.get_logger().info("Hardware PWM Initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to pigpiod: {e}")

        # 3. Map Parameter to FSM Target
        # This converts the launch argument (0, 90, 180) to your FSM state
        angle_map = {
            '0': "TURN_0",
            '90': "DEFAULT",
            '180': "TURN_180"
        }
        
        self.target_pos = angle_map.get(startup_angle, "DEFAULT")
        self.current_pos = "UNKNOWN"
        self.state = "MOVING" # Set to MOVING so FSM loop triggers immediately

        # Subscription for later commands
        self.subscription = self.create_subscription(
            String,
            'servo_command',
            self.listener_callback,
            10)

        # Main FSM Loop (10Hz)
        self.timer = self.create_timer(0.1, self.fsm_loop)
        self.get_logger().info(f"Startup complete. Target position: {self.target_pos}")

    def listener_callback(self, msg):
        cmd = msg.data.upper()
        if cmd in ["TURN_0", "DEFAULT", "TURN_180"]:
            self.target_pos = cmd
            self.state = "MOVING"

    def fsm_loop(self):
        if self.state == "MOVING":
            self.get_logger().info(f"Transitioning to {self.target_pos}...")

            if self.target_pos == "TURN_0":
                self.servo.min()
            elif self.target_pos == "DEFAULT":
                self.servo.mid()
            elif self.target_pos == "TURN_180":
                self.servo.max()

            time.sleep(0.8)
            self.servo.value = None # Detach to prevent jitter
            self.current_pos = self.target_pos
            self.state = "STEADY"
            self.get_logger().info(f"Reached {self.current_pos}. Signal detached.")

def main(args=None):
    rclpy.init(args=args)
    node = ServoHardwareController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.servo.value = None
        node.destroy_node()
        rclpy.shutdown()
