# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import getCommand


class velocityPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()

        

        msg.linear.x = 10.0


        
        self.publisher_.publish(msg)
        self.get_logger().info('Sending: "%s"' % msg)
        self.i += 1



def main(args=None):
    rclpy.init(args=args)

    velocity = velocityPublisher()  # creates publisher
    rclpy.spin(velocity)  # runs node continuously

    velocity.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
