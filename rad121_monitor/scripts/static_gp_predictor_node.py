import rclpy
from rclpy.node import Node
import pickle
import numpy as np
from rad121_monitor.srv import GPPredict

class GPPredictor(Node):
    def __init__(self):
        super().__init__('gp_predictor_node')
        
        self.declare_parameter('model_path', '')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.get_logger().info(f"Attempting to load GP model from: {model_path}")

        try:
            with open(model_path, 'rb') as f:
                self.gp_model = pickle.load(f)['gp_lo'] 
            self.get_logger().info('Successfully loaded static GP model.')
        except Exception as e:
            self.get_logger().error(f"Failed to load static model: {e}")
            raise SystemExit

        self.srv = self.create_service(GPPredict, 'predict_gp', self.predict_callback)
        self.get_logger().info('Static GP Prediction service is ready on "predict_gp".')

    def predict_callback(self, request, response):
        try:
            points_to_predict = np.array([[p.x, p.y] for p in request.points_to_predict])

            if points_to_predict.shape[0] == 0:
                return response

            mean_pred, var_pred = self.gp_model.predict(points_to_predict)
            std_dev_pred = np.sqrt(var_pred)

            response.predicted_mean = mean_pred.flatten().tolist()
            response.predicted_std_dev = std_dev_pred.flatten().tolist()
            
        except Exception as e:
            self.get_logger().error(f"Failed to process static prediction request: {e}")

        return response

def main(args=None):
    rclpy.init(args=args)
    try:
        node = GPPredictor()
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()