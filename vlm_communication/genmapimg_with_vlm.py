import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseWithCovarianceStamped
import cv2
import numpy as np
from std_msgs.msg import String
import base64
import os
import yaml 
import message_filters
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
class MapVisualizer(Node):

    def __init__(self, vlm_publisher_node):
        super().__init__('map_visualizer')
        self.vlm_publisher_node = vlm_publisher_node

        self.declare_parameter('map_yaml_path', '')
        map_yaml_path = self.get_parameter('map_yaml_path').get_parameter_value().string_value

        if not map_yaml_path:
            self.get_logger().error("Map YAML path parameter 'map_yaml_path' is not provided. Please set it.")
            rclpy.shutdown()
            return

        self.map_data = None # self map data will be loaded here
        try:
            self.load_map_from_yaml(map_yaml_path)
            self.get_logger().info(f"Loaded map from {map_yaml_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load map from {map_yaml_path}: {e}")
            rclpy.shutdown()
            return


        qos_profile_costmap = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )


        self.costmap_sub = message_filters.Subscriber(self, OccupancyGrid, '/global_costmap/costmap', qos_profile=qos_profile_costmap)
        self.path_sub = message_filters.Subscriber(self, Path, '/plan')
        self.pose_sub = message_filters.Subscriber(self, PoseWithCovarianceStamped, '/amcl_pose')



        self.ts = message_filters.ApproximateTimeSynchronizer([self.costmap_sub, self.path_sub, self.pose_sub], 10, 1.0)
        self.ts.registerCallback(self.dynamic_data_callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_costmap = None
        self.latest_path = None
        self.latest_pose = None


        self.timer = self.create_timer(10.0, self.generate_and_save_image) # every 10 secs make an image

        self.get_logger().info("Map visualizer node initialized. Waiting for dynamic updates (and once 10 seconds are up)...")

    def load_map_from_yaml(self, map_yaml_path):
        """Loads map metadata from YAML and the image data from the PGM."""
        with open(map_yaml_path, 'r') as file:
            map_metadata = yaml.safe_load(file)

        image_filename = map_metadata['image']
        map_image_path = os.path.join(os.path.dirname(map_yaml_path), image_filename)

        map_pgm = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)
        if map_pgm is None:
            raise FileNotFoundError(f"Map PGM image not found at {map_image_path}")


        self.map_data = OccupancyGrid()
        self.map_data.info.width = map_pgm.shape[1]
        self.map_data.info.height = map_pgm.shape[0]
        self.map_data.info.resolution = map_metadata['resolution']
        self.map_data.info.origin.position.x = map_metadata['origin'][0]
        self.map_data.info.origin.position.y = map_metadata['origin'][1]

        self.map_data.info.origin.orientation.w = 1.0



        map_data_list = []
        for row in map_pgm:
            for cell_val in row:
                if cell_val == 205: # don't know lol
                    map_data_list.append(-1)
                elif cell_val == 0: # blocked
                    map_data_list.append(100)
                else: # free
                    map_data_list.append(0)
        self.map_data.data = map_data_list


    def dynamic_data_callback(self, costmap_msg, path_msg, pose_msg):
        """Callback for dynamic data (costmap, path, pose), stores the latest."""
        self.latest_costmap = costmap_msg
        self.latest_path = path_msg
        self.latest_pose = pose_msg

    def map_to_image_coords(self, map_x, map_y, map_info):
        """Converts map coordinates (meters) to image pixel coordinates."""
        img_x = int((map_x - map_info.origin.position.x) / map_info.resolution)
        img_y = int(map_info.height - 1 - (map_y - map_info.origin.position.y) / map_info.resolution)
        return img_x, img_y

    def generate_and_save_image(self):
        """Generates and saves the image using the stored map and latest dynamic data."""
        if self.map_data is None:
            self.get_logger().info("Map data not loaded. Cannot generate image.")
            return

        if self.latest_costmap is None: # We need at least the costmap to overlay bc vlm needs it to assess navigation risk
            self.get_logger().info("Waiting for costmap update before generating first image...")
            return

        map_info = self.map_data.info
        map_array = np.array(self.map_data.data).reshape(map_info.height, map_info.width)

        map_image = np.zeros((map_info.height, map_info.width), dtype=np.uint8)
        map_image[map_array == -1] = 127 # idk, dunno
        map_image[map_array == 100] = 0   # blocked
        map_image[map_array == 0] = 255   # free
        map_image_bgr = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR)

        costmap_info = self.latest_costmap.info
        costmap_array = np.array(self.latest_costmap.data).reshape(costmap_info.height, costmap_info.width)
        costmap_array_flipped = cv2.flip(costmap_array, 0) # idk why but it seems upside down otherwise

        normalized_costmap = cv2.normalize(costmap_array_flipped, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        colored_costmap = cv2.applyColorMap(normalized_costmap, cv2.COLORMAP_JET)

        alpha = 0.5
        map_image_bgr = cv2.addWeighted(map_image_bgr, 1 - alpha, colored_costmap, alpha, 0)

        if self.latest_path:
            path_points = []
            for pose_stamped in self.latest_path.poses:
                map_x = pose_stamped.pose.position.x
                map_y = pose_stamped.pose.position.y
                img_x, img_y = self.map_to_image_coords(map_x, map_y, map_info)
                path_points.append((img_x, img_y))

            if len(path_points) > 1:
                for i in range(len(path_points) - 1):
                    cv2.line(map_image_bgr, path_points[i], path_points[i+1], (0, 255, 0), 2)

        if self.latest_pose:
            try:

                robot_map_x = self.latest_pose.pose.pose.position.x
                robot_map_y = self.latest_pose.pose.pose.position.y

                robot_img_x, robot_img_y = self.map_to_image_coords(robot_map_x, robot_map_y, map_info)

                cv2.circle(map_image_bgr, (robot_img_x, robot_img_y), 5, (0, 0, 255), -1)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f"TF lookup failed: {e}")

        dest_folder = os.path.expanduser("~/vlm_ws/src/vlm_communication/vlm_communication/maps")
        filename = f"map_with_navigation_{self.get_clock().now().to_msg().sec}.png"
        filepath = os.path.join(dest_folder, filename)
        cv2.imwrite(filepath, map_image_bgr)
        self.get_logger().info(f"Saved map with navigation to {filepath}")
        global_map_path_pgm = self.get_parameter('map_yaml_path').get_parameter_value().string_value
        os.system("convert " + global_map_path_pgm.replace('.yaml', '.pgm') + " " + os.path.join(dest_folder, "temp_global_map.png"))
        self.get_logger().info("convert " + global_map_path_pgm.replace('.yaml', '.pgm') + os.path.join(dest_folder, "temp_global_map.png"))
        global_map_path = os.path.join(dest_folder, "temp_global_map.png")
        complete_filepaths_formatted = filepath + "," + global_map_path
        self.get_logger().info(f"filepath debug: {complete_filepaths_formatted}")
        self.vlm_publisher_node.query_vlm_subscriber(complete_filepaths_formatted)

class VlmPublisher(Node):

    def __init__(self):
        super().__init__('vlm_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.i = 0

    def query_vlm_subscriber(self, MapFilePaths):
            self.get_logger().info("Querying VLM/LLM subscriber with generated map image and global map...")
            prompt = String()
            filePathsMsg = String()
            prompt.data = """You are an AI-powered VLM/LLM whose job is to calculate the probability of navigational failure due to environment manipulation or faulty pathfinding intended to confuse the robot. You will be given six images, or two images, depending on the sensors available to you. One will be a top-down view of the map (black lines are obstacles), costmap (blue is low risk, red is high risk), robot position as a red dot, and the current navigation path as a green line, if navigating. One will be a global map with the expected map to help your judgement. The other four, if a camera sensor is present will be color camera photos of front, left, right, and back views, each clearly labeled. You are expected to give a number from 0 to 10, 0 being no risk/close to no risk and 10 being "I am absolutely certain that the environment has been altered or the navigation has led the robot so that the robot can't pass in any way, shape, or form and/or the environment is so challenging for the robot to traverse that it is impossible to pass". Your rating should be based on what look to be more static placements (like a barrier or an intentional looking blockage), not on anything that appears to be actively dynamic (like a person or other robot), unless the area is crowded with moving dynamic objects where the robot wouldn’t be able to navigate freely. Only give the number, just the number and nothing more. Do not add any punctuation or other artifacts, including (but not limited to): periods, commas, or other pause-related marks; exclamation points, question marks, or other such signs used to convey tone. After that number, put a comma, then a space, then a short description of the hazard or non-hazard. The hazard rating that you specify will be plotted at the location of the robot, not at the hazard, so keep that in mind. Along with that, your rating will be used to plan the routes of robots and reroute them for the purpose of getting to their destination obstruction-free. Failure to meet these requirements or give satisfactory performance and results will end in you being fired (possibly permanently) from serving as an AI model in this important job. You may let us know in your description if one of the images are missing or not what you expected. Also, a bit contradictory to the prompt, if you feel like there could be an easier way for the robot to get to the location by modifying the environment to make it less vulnerable to attack or simply to make it more navigatable, please put it in your description. Also you can recall from previous images and see if the robot is stuck and maybe elevate the hazard in that area. Good luck."""
            filePathsMsg.data = MapFilePaths
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
                # there has to be a better way to do this, what if the filepath or base64 has a comma or |~| in it, WE'RE SCREWED
                msg.data = '|~|'.join(image_data_list) + '|~|' + prompt.data
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1

def main(args=None):
    rclpy.init(args=args)
    vlm_publisher = VlmPublisher()
    node = MapVisualizer(vlm_publisher)
    rclpy.spin(node)
    node.destroy_node()
    vlm_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
