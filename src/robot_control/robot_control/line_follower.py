import rclpy
from rclpy.node import Node
from gpiozero import PWMOutputDevice, DigitalOutputDevice, LineSensor, DistanceSensor
from time import sleep

class GridNavigatorNode(Node):
    def __init__(self):
        super().__init__('grid_navigator')
        
        # --- 1. HARDWARE SETUP ---
        self.left_pwm = PWMOutputDevice(12)
        self.left_forward = DigitalOutputDevice(23)
        self.left_backward = DigitalOutputDevice(24)

        self.right_pwm = PWMOutputDevice(13)
        self.right_forward = DigitalOutputDevice(17)
        self.right_backward = DigitalOutputDevice(27)

        sensor_pins = [4, 5, 6, 16, 19, 20, 21, 26]
        self.sensors = [LineSensor(pin) for pin in sensor_pins]
        self.ultrasonic = DistanceSensor(echo=25, trigger=22, max_distance=2.0)

        # --- 2. SETTINGS ---
        self.LEFT_BASE_SPEED = 0.25
        self.RIGHT_BASE_SPEED = 0.25
        self.SHARP_REVERSE = 0.30
        self.LEVELING_REVERSE = 0.10
        self.LEFT_REVERSED = False
        self.RIGHT_REVERSED = True

        # State management
        self.last_turn = "STRAIGHT"
        self.is_turning_180 = False
        
        # --- 3. ROS 2 TIMER ---
        # Runs the control loop at 20Hz (every 0.05 seconds)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Grid Navigator Node started and hardware initialized.")

    def set_motors(self, l_speed, r_speed):
        """Drives motors. Allows negative numbers for reverse."""
        l_speed = max(-1.0, min(1.0, l_speed))
        r_speed = max(-1.0, min(1.0, r_speed))

        actual_l_speed = -l_speed if self.LEFT_REVERSED else l_speed
        actual_r_speed = -r_speed if self.RIGHT_REVERSED else r_speed

        # Left Motor Logic
        if actual_l_speed >= 0:
            self.left_forward.on()
            self.left_backward.off()
            self.left_pwm.value = actual_l_speed
        else:
            self.left_forward.off()
            self.left_backward.on()
            self.left_pwm.value = -actual_l_speed 

        # Right Motor Logic
        if actual_r_speed >= 0:
            self.right_forward.on()
            self.right_backward.off()
            self.right_pwm.value = actual_r_speed
        else:
            self.right_forward.off()
            self.right_backward.on()
            self.right_pwm.value = -actual_r_speed

    def execute_180_turn(self):
        """Spins the bot until middle sensors find the line."""
        self.get_logger().warn("Obstacle detected! Executing 180 Turn.")
        
        # Stop and wait briefly
        self.set_motors(0.0, 0.0)
        sleep(0.5)

        # Start spinning
        self.set_motors(-self.SHARP_REVERSE, self.RIGHT_BASE_SPEED)
        sleep(0.4) # Blind period
        
        # Spin until line found
        while True:
            readings = [int(s.value) for s in self.sensors]
            if readings[3] == 1 or readings[4] == 1:
                break
            sleep(0.01)

        self.set_motors(0.0, 0.0)
        sleep(0.1)
        self.last_turn = "STRAIGHT"

    def control_loop(self):
        # 0. Obstacle Check
        if self.ultrasonic.distance <= 0.07:
            self.execute_180_turn()
            return # Exit loop to restart after turn

        # 1. Read Sensors
        readings = [int(s.value) for s in self.sensors]

        # 2. Logic Decisions
        # 90-Degree Left
        if readings[0] == 1 or readings[1] == 1:
            self.set_motors(-self.SHARP_REVERSE, self.RIGHT_BASE_SPEED)
            self.last_turn = "LEFT"

        # 90-Degree Right
        elif readings[6] == 1 or readings[7] == 1:
            self.set_motors(self.LEFT_BASE_SPEED, -self.SHARP_REVERSE)
            self.last_turn = "RIGHT"

        # Straight Ahead
        elif readings[3] == 1 or readings[4] == 1:
            self.set_motors(self.LEFT_BASE_SPEED, self.RIGHT_BASE_SPEED)
            self.last_turn = "STRAIGHT"

        # Gentle Left Leveling
        elif readings[2] == 1:
            self.set_motors(-self.LEVELING_REVERSE, self.RIGHT_BASE_SPEED)
            self.last_turn = "LEFT"

        # Gentle Right Leveling
        elif readings[5] == 1:
            self.set_motors(self.LEFT_BASE_SPEED, -self.LEVELING_REVERSE)
            self.last_turn = "RIGHT"

        # Lost Line (Memory Search)
        else:
            if self.last_turn == "LEFT":
                self.set_motors(-self.SHARP_REVERSE, self.RIGHT_BASE_SPEED)
            elif self.last_turn == "RIGHT":
                self.set_motors(self.LEFT_BASE_SPEED, -self.SHARP_REVERSE)
            else:
                self.set_motors(self.LEFT_BASE_SPEED, self.RIGHT_BASE_SPEED)

def main(args=None):
    rclpy.init(args=args)
    node = GridNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.set_motors(0.0, 0.0)
        node.get_logger().info("Bot safely stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
