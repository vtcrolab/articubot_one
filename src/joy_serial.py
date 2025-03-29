import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import Joy

class JoyToSerial(Node):
    def __init__(self):
        super().__init__('joy_to_serial')

        # Open serial port (Ensure correct port is used)
        self.serial_port = serial.Serial('/dev/ttyS0', baudrate=115200, timeout=1)

        # Subscribe to joystick topic
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Steering servo range (adjust if necessary)
        self.servo_min = 60   # Left max
        self.servo_max = 120  # Right max
        self.servo_center = 90  # Straight

        # Motor speed range
        self.speed_min = 0    # Min speed
        self.speed_max = 255  # Max speed
        self.current_speed = 0  # Initialize speed to 0

    def joy_callback(self, msg):
        serial_command = ""
        
        # Get steering input from the Left Stick X-Axis
        steering_input = msg.axes[0]  # Left (-1) to Right (+1)
        
        # Map joystick (-1 to 1) to servo angles (60° to 120°)
        servo_angle = int(self.servo_center + (steering_input * (self.servo_max - self.servo_center)))
        
        serial_command += f"S{servo_angle}"

        # Get motor speed input from RT & LT buttons (Triggers)
        rt_pressed = msg.buttons[5]  # RT increases speed
        lt_pressed = msg.buttons[4]  # LT decreases speed

        if rt_pressed:
            self.current_speed = min(self.current_speed + 20, self.speed_max)  # Increase speed
        elif lt_pressed:
            self.current_speed = max(self.current_speed - 20, self.speed_min)  # Decrease speed

        # Format the serial command as "SXXXMYYY\n" (Servo: 60-120, Motor: 0-255)
        serial_command += f"M{self.current_speed}"

        button_y = msg.buttons[3]
        button_a = msg.buttons[0]
        button_x = msg.buttons[2]

        if button_y:
            serial_command += "F"
        elif button_a:
            serial_command += "B"
        elif button_x
            serial_command += "H" #for halt

        serial_command += "\n"

        self.serial_port.write(serial_command.encode())

        # Debugging logs
        self.get_logger().info(f"Sent to Arduino: {serial_command}")

def main(args=None):
    rclpy.init(args=args)
    node = JoyToSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
