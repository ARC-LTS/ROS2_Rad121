#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import GPy
from scipy.optimize import minimize
from geometry_msgs.msg import Point
from ros2_rad_map.srv import GPPredict, FindPeaks
from std_msgs.msg import Float64MultiArray
import threading
import tf2_ros
from tf2_geometry_msgs import PoseStamped

class GPPredictor(Node):
    def __init__(self):
        super().__init__('gp_predictor_node')
        
        self.declare_parameter('kernel.type', 'RBF')
        self.declare_parameter('kernel.lengthscale', 5.0)
        self.declare_parameter('sensor_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')

        self.predict_srv = self.create_service(GPPredict, 'predict_gp', self.predict_callback)
        self.peaks_srv = self.create_service(FindPeaks, 'find_rad_peaks', self.find_peaks_callback)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.historical_samples = []
        self.data_lock = threading.Lock()

        self.rad_sub = self.create_subscription(
            Float64MultiArray,
            '/rad',
            self.rad_callback,
            10
        )
        
        self.get_logger().info('GP Predictor Node has started. Services "predict_gp" and "find_rad_peaks" are ready.')

    def rad_callback(self, msg):
        if len(msg.data) < 2:
            return

        try:
            sensor_frame = self.get_parameter('sensor_frame').get_parameter_value().string_value
            map_frame = self.get_parameter('map_frame').get_parameter_value().string_value

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
        
        if training_values.ndim == 1:
            training_values = training_values[:, None]

        if kernel_type.lower() == 'matern52':
            kernel = GPy.kern.Matern52(input_dim=2, variance=1., lengthscale=lengthscale)
        else:
            kernel = GPy.kern.RBF(input_dim=2, variance=1., lengthscale=lengthscale)
        
        gp_model = GPy.models.GPRegression(training_points, training_values, kernel)
        return gp_model, training_points

    def find_peaks_callback(self, request, response):
        with self.data_lock:
            samples = list(self.historical_samples)

        observed_x = np.array([s['x'] for s in samples])
        observed_y = np.array([s['y'] for s in samples])
        observed_vals_mean = np.array([s['value'] for s in samples])

        gp_model, training_points = self._build_gp_model(observed_x, observed_y, observed_vals_mean)
        if gp_model is None:
            return response

        def objective_function(x):
            x = np.atleast_2d(x)
            mean, _ = gp_model.predict(x)
            return -mean[0, 0]

        min_x, min_y = training_points.min(axis=0)
        max_x, max_y = training_points.max(axis=0)
        bounds = [(min_x, max_x), (min_y, max_y)]

        found_peaks = []
        for i in range(request.num_starts):
            start_point = np.random.uniform(low=[min_x, min_y], high=[max_x, max_y])
            
            result = minimize(
                fun=objective_function,
                x0=start_point,
                method='L-BFGS-B',
                bounds=bounds
            )
            
            if result.success:
                peak_x, peak_y = result.x
                peak_val = -result.fun
                
                is_duplicate = False
                for existing_peak in found_peaks:
                    dist_sq = (existing_peak[0] - peak_x)**2 + (existing_peak[1] - peak_y)**2
                    if dist_sq < (self.get_parameter('kernel.lengthscale').get_parameter_value().double_value)**2:
                        is_duplicate = True
                        break
                
                if not is_duplicate:
                    found_peaks.append((peak_x, peak_y, peak_val))

        for peak in found_peaks:
            p = Point()
            p.x = peak[0]
            p.y = peak[1]
            p.z = peak[2]
            response.peaks.append(p)
            
        return response

    def predict_callback(self, request, response):
        try:
            observed_x = np.array(request.observed_x)
            observed_y = np.array(request.observed_y)
            observed_vals_mean = np.array(request.observed_values_mean)
            
            gp_model, _ = self._build_gp_model(observed_x, observed_y, observed_vals_mean)
            if gp_model is None:
                return response

            points_to_predict = np.array([[p.x, p.y] for p in request.points_to_predict])

            if points_to_predict.shape[0] == 0:
                return response

            mean_prediction, var_prediction = gp_model.predict(points_to_predict)
            std_dev_prediction = np.sqrt(var_prediction)

            response.predicted_mean = np.maximum(0, mean_prediction).flatten().tolist() 
            response.predicted_std_dev = np.maximum(0, std_dev_prediction).flatten().tolist()
            
        except Exception as e:
            self.get_logger().error(f"Failed to process prediction request: {e}")
            response.predicted_mean = []
            response.predicted_std_dev = []

        return response

def main(args=None):
    rclpy.init(args=args)
    gp_predictor_node = GPPredictor()
    rclpy.spin(gp_predictor_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
