import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseWithCovarianceStamped
import cv2
import numpy as np
import os
import yaml 
import message_filters
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
class MapVisualizer(Node):

    def __init__(self):
        super().__init__('map_visualizer')


        self.declare_parameter('map_yaml_path', '')
        map_yaml_path = self.get_parameter('map_yaml_path').get_parameter_value().string_value

        if not map_yaml_path:
            self.get_logger().error("Map YAML path parameter 'map_yaml_path' is not provided. Please set it.")
            rclpy.shutdown()
            return

        self.map_data = None # Store the map data from the file
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
        self.path_sub = message_filters.Subscriber(self, Path, '/plan') # Default QoS
        self.pose_sub = message_filters.Subscriber(self, PoseWithCovarianceStamped, '/amcl_pose') # Default QoS



        self.ts = message_filters.ApproximateTimeSynchronizer([self.costmap_sub, self.path_sub, self.pose_sub], 10, 1.0)
        self.ts.registerCallback(self.dynamic_data_callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_costmap = None
        self.latest_path = None
        self.latest_pose = None


        self.timer = self.create_timer(10.0, self.generate_and_save_image) # Generate image every 1 second

        self.get_logger().info("Map visualizer node initialized. Waiting for dynamic updates (and once 60 seconds are up)...")

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
                if cell_val == 205: # Unknown
                    map_data_list.append(-1)
                elif cell_val == 0: # Occupied
                    map_data_list.append(100)
                else: # Free (usually 254)
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

        if self.latest_costmap is None: # We need at least the costmap to overlay
            self.get_logger().info("Waiting for costmap update before generating first image...")
            return

        map_info = self.map_data.info
        map_array = np.array(self.map_data.data).reshape(map_info.height, map_info.width)

        map_image = np.zeros((map_info.height, map_info.width), dtype=np.uint8)
        map_image[map_array == -1] = 127 # Unknown
        map_image[map_array == 100] = 0   # Occupied
        map_image[map_array == 0] = 255   # Free
        map_image_bgr = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR)

        costmap_info = self.latest_costmap.info
        costmap_array = np.array(self.latest_costmap.data).reshape(costmap_info.height, costmap_info.width)
        costmap_array_flipped = cv2.flip(costmap_array, 0) # Flip vertically

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


def main(args=None):
    rclpy.init(args=args)
    node = MapVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
