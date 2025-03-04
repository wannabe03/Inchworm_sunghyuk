import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import sys,select,termios, tty

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_=self.create_publisher(Int8,'keyboard_topic',10)
        self.get_logger().info("Keyboard Node started! Use arrow keys for command")
        self.settings = termios.tcgetattr(sys.stdin)
        self.timer =self.create_timer(0.1, self.timer_callback)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ =select.select([sys.stdin], [], [], 0.1)
        key= ''
        if rlist:
            key =sys.stdin.read(1)
            if key == '\x1b':
                key +=sys.stdin.read(2)
        termios.tcsetattr(sys.stdin,termios.TCSADRAIN,self.settings)
        return key
    
    def timer_callback(self):
        try:
            key = self.get_key()
            if key:
                command = None
                if key == '\x1b[A':
                    command = 1
                elif key == '\x1b[B':
                    command = 2
                elif key == '\x1b[C':
                    command = 3
                elif key == '\x1b[D':
                    command = 4
                elif key == '\x03':
                    self.get_logger().info("Shutting down node..")
                    rclpy.shutdown()
                    return

                if command is not None:
                    msg = Int8()
                    msg.data = command
                    self.publisher_.publish(msg)
                    self.get_logger().info('Publishing: {}'.format(command))
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def destroy_node(self):
        self.destroy_publisher(self.publisher_)
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down keyboard control")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()  

