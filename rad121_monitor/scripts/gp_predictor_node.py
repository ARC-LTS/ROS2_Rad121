#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pickle
import numpy as np
from rad121_monitor.srv import GPPredict

class GPPredictor(Node):
    def __init__(self):
        super().__init__('gp_predictor_node')
        
        self.declare_parameter('model_path', 'path/to/your/model.pkl')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.get_logger().info(f"Attempting to load GP models from: {model_path}")

        try:
            with open(model_path, 'rb') as f:
                model_data = pickle.load(f)
                self.gp_mean_model = model_data['gp_lo']
                self.gp_std_dev_model = model_data['gp_delta']
            self.get_logger().info('Successfully loaded GP models.')
        except Exception as e:
            self.get_logger().error(f"An error occurred while loading models: {e}")
            raise SystemExit

        self.srv = self.create_service(GPPredict, 'predict_gp', self.predict_callback)
        self.get_logger().info('GP Prediction service is ready.')

    def predict_callback(self, request, response):
        try:
            points_to_predict = np.array([[p.x, p.y] for p in request.points_to_predict])

            if points_to_predict.shape[0] == 0:
                response.predicted_mean = []
                response.predicted_std_dev = []
                return response

            # Use the loaded GP models to predict mean and standard deviation
            # GPy's predict method returns mean and variance, so we take the sqrt for std dev.
            mean_prediction, _ = self.gp_mean_model.predict(points_to_predict)
            std_dev_prediction, _ = self.gp_std_dev_model.predict(points_to_predict)

            response.predicted_mean = mean_prediction.flatten().tolist()
            # Ensure std deviation is non-negative
            response.predicted_std_dev = np.abs(std_dev_prediction.flatten()).tolist()
            
            self.get_logger().info(f"Processed prediction request for {len(request.points_to_predict)} points.")

        except Exception as e:
            self.get_logger().error(f"Failed to process prediction request: {e}")
            response.predicted_mean = []
            response.predicted_std_dev = []

        return response

def main(args=None):
    rclpy.init(args=args)
    try:
        gp_predictor_node = GPPredictor()
        rclpy.spin(gp_predictor_node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()