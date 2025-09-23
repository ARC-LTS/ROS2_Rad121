#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import GPy 
from rad121_monitor.srv import GPPredict

class GPPredictor(Node):
    def __init__(self):
        super().__init__('gp_predictor_node')
        self.srv = self.create_service(GPPredict, 'predict_gp', self.predict_callback)
        self.get_logger().info('Dynamic GP Prediction service is ready.')

    def predict_callback(self, request, response):
        self.get_logger().info(f"Received dynamic prediction request with {len(request.observed_x)} samples.")
        
        try:
            observed_x = np.array(request.observed_x)
            observed_y = np.array(request.observed_y)
            observed_vals_mean = np.array(request.observed_values_mean)

            if len(observed_x) == 0:
                self.get_logger().warn("No observed data received, cannot train model.")
                return response # Return empty response

            training_points = np.vstack((observed_x, observed_y)).T
            training_values = observed_vals_mean[:, None] # Reshape for GPy

            self.get_logger().info("Defining new GP kernel...")
            kernel = GPy.kern.RBF(input_dim=2, variance=1., lengthscale=1.)
            
            self.get_logger().info("Creating new GP model...")
            gp_mean_model = GPy.models.GPRegression(training_points, training_values, kernel)
            
            self.get_logger().info("Optimizing model (this may be slow)...")
            gp_mean_model.optimize(messages=True)
            self.get_logger().info("Model optimization complete.")

            points_to_predict = np.array([[p.x, p.y] for p in request.points_to_predict])

            if points_to_predict.shape[0] == 0:
                return response

            self.get_logger().info(f"Predicting values for {points_to_predict.shape[0]} points...")
            mean_prediction, var_prediction = gp_mean_model.predict(points_to_predict)
            std_dev_prediction = np.sqrt(var_prediction)

            response.predicted_mean = mean_prediction.flatten().tolist()
            response.predicted_std_dev = std_dev_prediction.flatten().tolist()
            
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