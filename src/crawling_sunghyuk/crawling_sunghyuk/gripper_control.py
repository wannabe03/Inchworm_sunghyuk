import rclpy 
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray
import serial

class GripperControl(Node):
    def __init__(self):
        super().__init__('gripper_control')
        self.get_logger().info("Gripper control node has started!")

        # 시리얼 포트 설정
        self.arduino_serial = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        # ROS2 Subscriber 생성
        self.subscription = self.create_subscription(
            Int8MultiArray,
            'gripper_state',
            self.gripper_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def gripper_callback(self, msg):
        if len(msg.data) < 1:
            return
        elif len(msg.data) == 1:
            base_state = msg.data[0]
            command = f"{base_state}\n"
        else:
            base_state = msg.data[0]
            ee_state = msg.data[1]
            command = f"{base_state},{ee_state}\n"

        self.arduino_serial.write(command.encode())  # 시리얼 포트로 데이터 전송
        self.get_logger().info(f"Sent to Arduino: {command}")

    def destroy_node(self):
        self.arduino_serial.close()  # 시리얼 포트 닫기
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GripperControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down gripper control node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()