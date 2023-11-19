#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Bool
from geometry_msgs.msg import Twist

#inherit from Node for Node property
class Costant_Control(Node):
	def __init__(self):
		#node name
		super().__init__("constant_control")
		self.get_logger().info('Constant control node has been created')

		self.timer = self.create_timer(0.2,self.print_msg)
		self.pub = self.create_publisher(String,'/test_msg_string',15)
		self.pub2 = self.create_publisher(Twist,'/cmd_vel',15)
		self.sub = self.create_subscription(Bool,'/kill',self.callback,15)

	def print_msg(self):
		self.get_logger().info("sending constant control…")
		msg = String()
		msg.data = f"sending constant control…"
		msg1 = Twist()
		msg1.linear.x = 1.0
		msg1.angular.z = 1.0
		self.pub2.publish(msg1)
		
	def callback(self,msg):
		if(msg.data):
			self.timer.cancel()
			msg2 = Twist()
			self.pub2.publish(msg2)
			self.get_logger().info("Stop the robot…")
		


		
def main(args=None):
	rclpy.init(args=args)
	constant_control = Costant_Control()

	rclpy.spin(constant_control)

	rclpy.shutdown()


if __name__ == '__main__':
    main()