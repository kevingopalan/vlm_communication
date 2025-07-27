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
from std_msgs.msg import String
import base64
import os


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        prompt = String()
        filePathsMsg = String()
        prompt.data = input("Enter the prompt you want to send: ")
        filePathsMsg.data = input("Enter the file path(s) to the file(s) you want to send, using commas to separate each path: ")
        msg = String()
        if filePathsMsg.data.strip() == "":
            msg.data = prompt.data
        else:
            cleanedPaths = [p.strip() for p in filePathsMsg.data.split(',') if p.strip()]
            image_data_list = []
            for path in cleanedPaths:
                if os.path.isfile(path):
                    with open(path, 'rb') as f:
                        image_bytes = f.read()
                    image_b64 = base64.b64encode(image_bytes).decode('utf-8')
                    image_data_list.append(image_b64)
                else:
                    self.get_logger().warn(f"File not found: {path}")
            # Join base64 images with a separator and add prompt
            msg.data = '|~|'.join(image_data_list) + '|~|' + prompt.data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
