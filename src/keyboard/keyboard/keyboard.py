import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import sys,select,termios, tty

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_=self.create_publisher(Int8,'keyboard',10)
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
            command = 0
            if key:
                if key == '\x1b[A':
                    command = 1
                elif key == '\x1b[B':
                    command = 2
                elif key == '\x1b[C':
                    command = 3
                elif key == '\x1b[D':
                    command = 4
                elif key == '.':
                    command = 5
                elif key == ',':
                    command = 6
                elif key == 'f':
                    command = 7
                elif key == 'i':
                    command = 8
                elif key == 'o':
                    command = 9
                elif key == 'p':
                    command = 10
                elif key == '\x03':
                    rclpy.shutdown()
                    return
                    
            msg = Int8()
            msg.data = command
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: {}'.format(command))
            
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)



def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    rclpy.spin(node)
    node.get_logger().info("Shutting down keyboard control")
    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()  

