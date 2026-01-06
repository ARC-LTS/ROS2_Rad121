#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import GPy
from scipy.optimize import minimize
from geometry_msgs.msg import Point
from rad121_monitor.srv import GPPredict, FindPeaks

class GPPredictor(Node):
    def __init__(self):
        super().__init__('gp_predictor_node')
        
        # Declare parameters for kernel configuration
        self.declare_parameter('kernel.type', 'RBF') # Default to RBF for backward compatibility
        self.declare_parameter('kernel.lengthscale', 5.0)
        
        self.predict_srv = self.create_service(GPPredict, 'predict_gp', self.predict_callback)
        self.peaks_srv = self.create_service(FindPeaks, 'find_rad_peaks', self.find_peaks_callback)
        self.get_logger().info('GP services "predict_gp" and "find_rad_peaks" are ready.')

    def _build_gp_model(self, request):
        observed_x = np.array(request.observed_x)
        observed_y = np.array(request.observed_y)
        observed_vals_mean = np.array(request.observed_values_mean)

        if len(observed_x) == 0:
            self.get_logger().warn("No observed data received, cannot train model.")
            return None, None

        training_points = np.vstack((observed_x, observed_y)).T
        training_values = observed_vals_mean[:, None]

        kernel_type = self.get_parameter('kernel.type').get_parameter_value().string_value
        lengthscale = self.get_parameter('kernel.lengthscale').get_parameter_value().double_value

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
        
        gp_model, training_points = self._build_gp_model(request)
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
            gp_model, _ = self._build_gp_model(request)
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

def main(args=None):
    rclpy.init(args=args)
    gp_predictor_node = GPPredictor()
    rclpy.spin(gp_predictor_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()