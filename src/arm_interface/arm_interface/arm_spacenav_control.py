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

class ArmSpacenav(Node):

    def __init__(self):
        super().__init__('arm_spacenav_controller_node')
        self.declare_parameter('SPRAYER_BUTTON', 0)
        self.declare_parameter('HOME_BUTTON', 1)
        self.declare_parameter('SPRAY_LEVEL', 1.0)
        self.declare_parameter('HOME_POSITION', "(0,10.0,6.0,90.0)") # X,Y,Z,R
        self.declare_parameter('MAX_EXTENSION', 40.0)
        self.declare_parameter('SPEED_MULTIPLIER', 20.0)
        self.declare_parameter('ROT_SPEED_MULTIPLIER', 40.0)
        self.declare_parameter('DEADBAND', 0.15)
        
        self.SPRAYER_BUTTON = self.get_parameter("SPRAYER_BUTTON").value
        self.HOME_BUTTON = self.get_parameter("HOME_BUTTON").value
        self.SPRAY_LEVEL = self.get_parameter("SPRAY_LEVEL").value
        self.MAX_EXTENSION = self.get_parameter("MAX_EXTENSION").value
        self.SPEED_MULTIPLIER = self.get_parameter("SPEED_MULTIPLIER").value
        self.ROT_SPEED_MULTIPLIER = self.get_parameter("ROT_SPEED_MULTIPLIER").value
        self.HOME_POSITION = eval(self.get_parameter("HOME_POSITION").value)
        self.DEADBAND = self.get_parameter("DEADBAND").value
        
        self.sprayer_pub = self.create_publisher(Float32, "/arm/sprayer", 10)
        self.position_pub = self.create_publisher(ArmPosition, "/arm/position", 10)
        self.joy_sub = self.create_subscription(Joy, "/spacenav/joy", self.joy_callback, 10)
        
        self.velocity = (0,0,0,0)
        self.position = list(self.HOME_POSITION)
        
        update_hz = 60.0
        
        self.dt = 1.0 / update_hz
        self.motionctl_timer = self.create_timer(self.dt, self.motion_timer_cb)
    
    def motion_timer_cb(self):
        self.position = [p + self.velocity[i]*self.dt for i,p in enumerate(self.position)]
        
        # Normed position vector
        extension = math.sqrt(self.position[0] ** 2 + self.position[1] ** 2 + self.position[2] ** 2)
        
        # Normalize if too big
        if (extension > self.MAX_EXTENSION):
            for i in range(3):
                self.position[i] = (self.position[i]/extension) * self.MAX_EXTENSION
                
        # Clamp wrist rotation
        self.position[3] = max(-90.0, min(90.0, self.position[3]))
        
        # Clamp Y to be forwards
        self.position[1] = max(0.0, self.position[1])
                
        posmsg = ArmPosition()
        posmsg.position.x = self.position[0]
        posmsg.position.y = self.position[1]
        posmsg.position.z = self.position[2]
        posmsg.effector_pitch = self.position[3]
        
        self.position_pub.publish(posmsg)
                        
    def joy_callback(self, msg: Joy):
        self.velocity = (
            -deadband(msg.axes[1], self.DEADBAND) * self.SPEED_MULTIPLIER, 
            deadband(msg.axes[0], self.DEADBAND) * self.SPEED_MULTIPLIER, 
            deadband(msg.axes[2], self.DEADBAND) * self.SPEED_MULTIPLIER, 
            -deadband(msg.axes[4], self.DEADBAND) * self.ROT_SPEED_MULTIPLIER
        )
        
        if msg.buttons[self.HOME_BUTTON]:
            self.position = list(self.HOME_POSITION)
        
        spray = self.SPRAY_LEVEL if msg.buttons[self.SPRAYER_BUTTON] else 0.0
        
        spray_msg = Float32()
        spray_msg.data = spray
        self.sprayer_pub.publish(spray_msg)

def main(args=None):
    rclpy.init(args=args)
    armctl = ArmSpacenav()

    rclpy.spin(armctl)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    armctl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()