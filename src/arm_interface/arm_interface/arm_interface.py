import rclpy
from rclpy.node import Node
import serial

from std_msgs.msg import Float32
from rover_interfaces.msg import ArmPosition


class ArmInterface(Node):

    def __init__(self):
        super().__init__('arm_interface_node')
        self.declare_parameter('PORT', '/dev/ttyACM0')
        
        self.portname = self.get_parameter('PORT').value
        self.serial = serial.Serial(self.portname, baudrate=115200)
        if (not self.serial.is_open):
            self.get_logger().error(f"Could not open port '{self.portname}'")
            exit(1)
        
        self.sprayer_sub = self.create_subscription(
            Float32,
            'sprayer',
            self.sprayer_callback,
            10 # QoS
        )
        
        self.position_sub = self.create_subscription(
            ArmPosition,
            'position',
            self.xyzr_callback,
            10 # QoS
        )
        
    def send_serial(self, s: str):
        self.serial.write(s.encode())
        self.serial.flush()

    def sprayer_callback(self, msg: Float32):
        power = max(0.0, min(1.0, float(msg.data)))
        self.send_serial(f"$S{round(power, ndigits=3)}^")
        
    def xyzr_callback(self, msg:ArmPosition):
        # print(msg.position, msg.effector_pitch)
        self.send_serial(f"$TX{round(msg.position.x, ndigits=3)}Y{round(msg.position.y, ndigits=3)}Z{round(msg.position.z, ndigits=3)}R{round(msg.effector_pitch, ndigits=3)}^")
        pass
        
    def destroy_node(self):
        self.serial.close()
        self.get_logger().info(f"Successfully closed port")
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    iface = ArmInterface()

    rclpy.spin(iface)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    iface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()