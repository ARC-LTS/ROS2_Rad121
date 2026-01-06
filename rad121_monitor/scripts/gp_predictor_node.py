#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import GPy
from scipy.optimize import minimize
from geometry_msgs.msg import Point
from rad121_monitor.srv import GPPredict, FindPeaks
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Float64MultiArray
from nav_msgs.msg import OccupancyGrid
import sensor_msgs_py.point_cloud2 as pc2
import struct
import threading
import tf2_ros
from tf2_geometry_msgs import PoseStamped
import rclpy.qos

class GPPredictor(Node):
    def __init__(self):
        super().__init__('gp_predictor_node')
        
        # Declare parameters
        self.declare_parameter('kernel.type', 'RBF')
        self.declare_parameter('kernel.lengthscale', 5.0)
        self.declare_parameter('publish_gp_cloud', True)
        self.declare_parameter('cloud_resolution_m', 0.25)
        self.declare_parameter('cloud_update_period_s', 10.0)
        self.declare_parameter('sensor_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')

        # Services
        self.predict_srv = self.create_service(GPPredict, 'predict_gp', self.predict_callback)
        self.peaks_srv = self.create_service(FindPeaks, 'find_rad_peaks', self.find_peaks_callback)
        
        # Publisher
        self.cloud_pub = self.create_publisher(PointCloud2, '/gp_surface_cloud', 10)

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Data Storage
        self.historical_samples = []
        self.data_lock = threading.Lock()
        self.map_metadata = None

        # Subscriber for Rad Data
        self.rad_sub = self.create_subscription(
            Float64MultiArray,
            '/rad',
            self.rad_callback,
            10
        )
        
        # Subscriber for Map
        # Use TRANSIENT_LOCAL durability to get the last published map
        map_qos_profile = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos_profile
        )

        # Timer for cloud generation
        update_period = self.get_parameter('cloud_update_period_s').get_parameter_value().double_value
        self.cloud_timer = self.create_timer(update_period, self.generate_cloud_callback)

        self.get_logger().info('GP Predictor Node has started.')
        self.get_logger().info('Services "predict_gp" and "find_rad_peaks" are ready.')
        self.get_logger().info(f"3D heatmap generation is {'ENABLED' if self.get_parameter('publish_gp_cloud').get_parameter_value().bool_value else 'DISABLED'}.")
        self.get_logger().info(f"Updating 3D heatmap every {update_period} seconds.")

    def map_callback(self, msg):
        """Stores the map metadata whenever it's updated."""
        if self.map_metadata is None or \
           msg.info.width != self.map_metadata['width'] or \
           msg.info.height != self.map_metadata['height'] or \
           msg.info.resolution != self.map_metadata['resolution']:
            
            self.get_logger().info(f"Map updated. New size: {msg.info.width}x{msg.info.height} with resolution {msg.info.resolution:.3f}m.")
            self.map_metadata = {
                'resolution': msg.info.resolution,
                'width': msg.info.width,
                'height': msg.info.height,
                'origin': msg.info.origin
            }


    def _value_to_rgb(self, value, min_val, max_val):
        # Normalize value to 0-1 range
        normalized_value = (value - min_val) / (max_val - min_val) if (max_val - min_val) > 0 else 0
        normalized_value = max(0.0, min(1.0, normalized_value))

        # Simple Jet-like colormap (blue-green-yellow-red)
        if normalized_value < 0.25: # Blue to Cyan
            r, g, b = 0.0, normalized_value * 4.0, 1.0
        elif normalized_value < 0.5: # Cyan to Green
            r, g, b = 0.0, 1.0, 1.0 - (normalized_value - 0.25) * 4.0
        elif normalized_value < 0.75: # Green to Yellow
            r, g, b = (normalized_value - 0.5) * 4.0, 1.0, 0.0
        else: # Yellow to Red
            r, g, b = 1.0, 1.0 - (normalized_value - 0.75) * 4.0, 0.0
        
        # Convert to 8-bit RGB and pack into a single integer
        r_int = int(r * 255)
        g_int = int(g * 255)
        b_int = int(b * 255)
        
        rgb_packed = (r_int << 16) | (g_int << 8) | b_int
        # Use struct to pack it into a float
        return struct.unpack('f', struct.pack('I', rgb_packed))[0]

    def rad_callback(self, msg):
        if len(msg.data) < 2:
            return

        try:
            sensor_frame = self.get_parameter('sensor_frame').get_parameter_value().string_value
            map_frame = self.get_parameter('map_frame').get_parameter_value().string_value

            pose_in_sensor = PoseStamped()
            pose_in_sensor.header.frame_id = sensor_frame
            pose_in_sensor.header.stamp = self.get_clock().now().to_msg()

            # The C++ version used rclcpp::Time(0), the Python equivalent is to use a timeout
            # to get the latest available transform.
            transform = self.tf_buffer.lookup_transform(map_frame, sensor_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.1))
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            rad_value = msg.data[1]

            with self.data_lock:
                self.historical_samples.append({'x': x, 'y': y, 'value': rad_value})

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF Error in rad_callback: {e}')

    def _build_gp_model(self, observed_x, observed_y, observed_vals_mean):
        if len(observed_x) == 0:
            self.get_logger().warn("No observed data provided, cannot train model.")
            return None, None

        training_points = np.vstack((observed_x, observed_y)).T
        training_values = observed_vals_mean[:, None]

        kernel_type = self.get_parameter('kernel.type').get_parameter_value().string_value
        lengthscale = self.get_parameter('kernel.lengthscale').get_parameter_value().double_value
        
        # Ensure training_values is 2D
        if training_values.ndim == 1:
            training_values = training_values[:, None]

        if kernel_type.lower() == 'matern52':
            kernel = GPy.kern.Matern52(input_dim=2, variance=1., lengthscale=lengthscale)
        elif kernel_type.lower() == 'rbf':
            kernel = GPy.kern.RBF(input_dim=2, variance=1., lengthscale=lengthscale)
        else:
            self.get_logger().error(f"Unsupported kernel type '{kernel_type}'. Defaulting to RBF.")
            kernel = GPy.kern.RBF(input_dim=2, variance=1., lengthscale=lengthscale)
        
        gp_model = GPy.models.GPRegression(training_points, training_values, kernel)
        return gp_model, training_points

    def find_peaks_callback(self, request, response):
        self.get_logger().info(f"Received peak finding request with {len(request.observed_x)} samples and {request.num_starts} starts.")
        
        observed_x = np.array(request.observed_x)
        observed_y = np.array(request.observed_y)
        observed_vals_mean = np.array(request.observed_values_mean)

        gp_model, training_points = self._build_gp_model(observed_x, observed_y, observed_vals_mean)
        if gp_model is None:
            return response

        # Define the objective function to MINIMIZE (negative of GP mean)
        def objective_function(x):
            x = np.atleast_2d(x)
            mean, _ = gp_model.predict(x)
            return -mean[0, 0]

        # Define bounds for the search space based on data
        min_x, min_y = training_points.min(axis=0)
        max_x, max_y = training_points.max(axis=0)
        bounds = [(min_x, max_x), (min_y, max_y)]

        found_peaks = []
        for i in range(request.num_starts):
            # Generate a random starting point within the bounds
            start_point = np.random.uniform(low=[min_x, min_y], high=[max_x, max_y])
            
            # Run the optimizer
            result = minimize(
                fun=objective_function,
                x0=start_point,
                method='L-BFGS-B',
                bounds=bounds
            )
            
            if result.success:
                peak_x, peak_y = result.x
                peak_val = -result.fun
                
                # Check if this peak is a duplicate of one we already found
                is_duplicate = False
                for existing_peak in found_peaks:
                    dist_sq = (existing_peak[0] - peak_x)**2 + (existing_peak[1] - peak_y)**2
                    if dist_sq < (self.get_parameter('kernel.lengthscale').get_parameter_value().double_value)**2:
                        is_duplicate = True
                        break
                
                if not is_duplicate:
                    self.get_logger().info(f"Found new peak at ({peak_x:.2f}, {peak_y:.2f}) with value {peak_val:.2f}")
                    found_peaks.append((peak_x, peak_y, peak_val))

        # Populate the response
        for peak in found_peaks:
            p = Point()
            p.x = peak[0]
            p.y = peak[1]
            p.z = peak[2] # Using z to store the peak's radiation value
            response.peaks.append(p)
            
        self.get_logger().info(f"Peak finding complete. Found {len(response.peaks)} unique peaks.")
        return response

    def predict_callback(self, request, response):
        self.get_logger().info(f"Received dynamic prediction request with {len(request.observed_x)} samples.")
        
        try:
            observed_x = np.array(request.observed_x)
            observed_y = np.array(request.observed_y)
            observed_vals_mean = np.array(request.observed_values_mean)
            
            gp_model, _ = self._build_gp_model(observed_x, observed_y, observed_vals_mean)
            if gp_model is None:
                return response

            points_to_predict = np.array([[p.x, p.y] for p in request.points_to_predict])

            if points_to_predict.shape[0] == 0:
                self.get_logger().warn("No points to predict were provided.")
                return response

            self.get_logger().info(f"Predicting values for {points_to_predict.shape[0]} points...")
            mean_prediction, var_prediction = gp_model.predict(points_to_predict)
            std_dev_prediction = np.sqrt(var_prediction)

            clamped_mean_prediction = np.maximum(0, mean_prediction)
            response.predicted_mean = clamped_mean_prediction.flatten().tolist() 
            clamped_std_dev = np.maximum(0, std_dev_prediction)           
            response.predicted_std_dev = clamped_std_dev.flatten().tolist()
            
            self.get_logger().info("Prediction successful.")

        except Exception as e:
            self.get_logger().error(f"Failed to process dynamic prediction request: {e}")
            response.predicted_mean = []
            response.predicted_std_dev = []

        return response

    def generate_cloud_callback(self):
        if not self.get_parameter('publish_gp_cloud').get_parameter_value().bool_value:
            return

        with self.data_lock:
            if not self.historical_samples:
                self.get_logger().info("No samples collected yet, skipping cloud generation.")
                return
            # Deep copy of samples for processing
            samples = list(self.historical_samples)

        self.get_logger().info(f"Generating GP surface cloud from {len(samples)} samples.")

        observed_x = np.array([s['x'] for s in samples])
        observed_y = np.array([s['y'] for s in samples])
        observed_vals_mean = np.array([s['value'] for s in samples])

        gp_model, _ = self._build_gp_model(observed_x, observed_y, observed_vals_mean)
        if gp_model is None:
            return

        # Check if map has been received
        if self.map_metadata is None:
            self.get_logger().warn("Map not received yet, skipping full-map cloud generation.")
            return

        # Define bounds for the grid based on the map dimensions
        min_x = self.map_metadata['origin'].position.x
        min_y = self.map_metadata['origin'].position.y
        max_x = min_x + self.map_metadata['width'] * self.map_metadata['resolution']
        max_y = min_y + self.map_metadata['height'] * self.map_metadata['resolution']

        resolution = self.get_parameter('cloud_resolution_m').get_parameter_value().double_value
        if resolution <= 0:
            self.get_logger().warn("Cloud resolution is <= 0, skipping generation.")
            return

        # Create a grid of points to predict
        grid_x = np.arange(min_x, max_x, resolution)
        grid_y = np.arange(min_y, max_y, resolution)
        
        if len(grid_x) == 0 or len(grid_y) == 0:
            self.get_logger().warn("Not enough spatial distribution in samples to generate a grid. Skipping cloud generation.")
            return

        xx, yy = np.meshgrid(grid_x, grid_y)
        points_to_predict = np.vstack([xx.ravel(), yy.ravel()]).T

        if points_to_predict.shape[0] > 0:
            # Predict the Z value (radiation) for each point on the grid
            mean_prediction, _ = gp_model.predict(points_to_predict)
            
            min_rad = mean_prediction.min()
            max_rad = mean_prediction.max()

            cloud_points = []
            for i in range(points_to_predict.shape[0]):
                x = points_to_predict[i, 0]
                y = points_to_predict[i, 1]
                z = mean_prediction[i, 0]
                rgb = self._value_to_rgb(z, min_rad, max_rad)
                cloud_points.append([x, y, z, rgb])
            
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            ]

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.get_parameter('map_frame').get_parameter_value().string_value

            cloud_msg = pc2.create_cloud(header, fields, cloud_points)
            self.cloud_pub.publish(cloud_msg)
            self.get_logger().info(f"Published GP surface with {len(cloud_points)} points.")

def main(args=None):
    rclpy.init(args=args)
    gp_predictor_node = GPPredictor()
    rclpy.spin(gp_predictor_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()