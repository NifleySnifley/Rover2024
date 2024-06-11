import rclpy
from rclpy.node import Node
import serial
import math

from std_msgs.msg import Float32
from rover_interfaces.msg import ArmPosition
from sensor_msgs.msg import Joy

def deadband(num, deadband):
    if abs(num) < deadband:
        return 0
    else:
        return (num + deadband)/(1.0-deadband) if (num < 0) else (num - deadband)/(1.0-deadband)

class WhackerCtl(Node):

    def __init__(self):
        super().__init__('whacker_control_node')
        self.declare_parameter('WHACKER_SPEED_AXIS', 2)
        self.declare_parameter('WHACKER_AXIS_ADD', -1.0)
        self.declare_parameter('WHACKER_AXIS_DEADBAND', 0.05)
        self.declare_parameter('WHACKER_AXIS_MUL', -0.5)
        self.declare_parameter('WHACKER_SPEED_BUTTON', 4)
        self.declare_parameter('WHACKER_BUTTON_VALUE', 1.0)
        self.declare_parameter('DEPLOY_TOGGLE_BUTTON', 2)
        self.declare_parameter('DEPLOY_UP_BUTTON', -1)
        self.declare_parameter('DEPLOY_DOWN_BUTTON', -1)
        self.declare_parameter('DEPLOY_AXIS', 7)
        
        self.WHACKER_SPEED_AXIS = self.get_parameter("WHACKER_SPEED_AXIS").value
        self.WHACKER_AXIS_ADD = self.get_parameter("WHACKER_AXIS_ADD").value
        self.WHACKER_AXIS_MUL = self.get_parameter("WHACKER_AXIS_MUL").value
        self.WHACKER_SPEED_BUTTON = self.get_parameter("WHACKER_SPEED_BUTTON").value
        self.WHACKER_BUTTON_VALUE = self.get_parameter("WHACKER_BUTTON_VALUE").value
        self.WHACKER_AXIS_DEADBAND = self.get_parameter("WHACKER_AXIS_DEADBAND").value
        self.DEPLOY_TOGGLE_BUTTON = self.get_parameter("DEPLOY_TOGGLE_BUTTON").value
        self.DEPLOY_UP_BUTTON = self.get_parameter("DEPLOY_UP_BUTTON").value
        self.DEPLOY_DOWN_BUTTON = self.get_parameter("DEPLOY_DOWN_BUTTON").value
        self.DEPLOY_AXIS = self.get_parameter("DEPLOY_AXIS").value
        
        self.whacker_speed_pub = self.create_publisher(Float32, "/arm/whacker", 10)
        self.whacker_deploy_pub = self.create_publisher(Float32, "/arm/whacker_deploy", 10)
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        
        self.deploy_state = -1.0
        self.prev_dep_btn = False
        
    def joy_callback(self, msg: Joy):
        whackspeed = Float32()
        whackspeed.data = max(-1.0, min(1.0,
            (deadband(msg.axes[self.WHACKER_SPEED_AXIS], self.WHACKER_AXIS_DEADBAND) + self.WHACKER_AXIS_ADD)*self.WHACKER_AXIS_MUL + 
            (self.WHACKER_BUTTON_VALUE if (msg.buttons[self.WHACKER_SPEED_BUTTON] and (self.WHACKER_SPEED_BUTTON != -1)) else 0.0)
        ))
        self.whacker_speed_pub.publish(whackspeed)
        
        depspeed = 0.0
        if (self.DEPLOY_DOWN_BUTTON != -1 and msg.buttons[self.DEPLOY_DOWN_BUTTON]):
            depspeed -= 2.0
        if (self.DEPLOY_UP_BUTTON != -1 and msg.buttons[self.DEPLOY_UP_BUTTON]):
            depspeed += 2.0
        if (self.DEPLOY_AXIS != -1):
            depspeed -= 2.0*msg.axes[self.DEPLOY_AXIS]
            
        if (self.DEPLOY_TOGGLE_BUTTON != -1):
            if (self.prev_dep_btn != msg.buttons[self.DEPLOY_TOGGLE_BUTTON]):
                self.prev_dep_btn = msg.buttons[self.DEPLOY_TOGGLE_BUTTON]
                if (msg.buttons[self.DEPLOY_TOGGLE_BUTTON]):
                    self.deploy_state *= -1
                    
        if (self.DEPLOY_TOGGLE_BUTTON != -1):
            depspeed += self.deploy_state
        
        dep = Float32()
        dep.data = max(-1.0, min(1.0, depspeed))
        self.whacker_deploy_pub.publish(dep)
        

def main(args=None):
    rclpy.init(args=args)
    wctl = WhackerCtl()

    rclpy.spin(wctl)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wctl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()