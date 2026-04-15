#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import GPy
from scipy.optimize import minimize
from geometry_msgs.msg import Point
from ros2_rad_map.srv import GPPredict, FindPeaks
from std_msgs.msg import Float64MultiArray, Float64
import threading
import tf2_ros
from tf2_geometry_msgs import PoseStamped
import pickle
import os

class LowFiMapping(GPy.core.Mapping):
    def __init__(self, low_fi_model, initial_rho=1.0, max_prior=100.0):
        super().__init__(input_dim=2, output_dim=1)
        self.low_fi_model = low_fi_model
        # Create rho as a GPy parameter so it can be optimized
        self.rho = GPy.core.parameterization.Param('rho', initial_rho)
        self.max_prior = max_prior
        self.link_parameter(self.rho)

    def f(self, X):
        X = np.atleast_2d(X)
        m, _ = self.low_fi_model.predict(X)
        # Cap the low-fi prediction at max_prior to avoid biasing high-fi GP too much
        m_capped = np.minimum(m, self.max_prior)
        return self.rho * m_capped

    def update_gradients(self, dL_dF, X):
        X = np.atleast_2d(X)
        m, _ = self.low_fi_model.predict(X)
        m_capped = np.minimum(m, self.max_prior)
        # dL/drho = sum( dL/df * df/drho ) = sum( dL/df * m_capped )
        self.rho.gradient = np.sum(dL_dF * m_capped)

class GPPredictor(Node):
    def __init__(self):
        super().__init__('gp_predictor_node')
        
        self.declare_parameter('kernel.type', 'RBF')
        self.declare_parameter('kernel.lengthscale', 5.0)
        self.declare_parameter('fix_lengthscale', False)
        self.declare_parameter('sensor_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('sparse_threshold', 300) # Points before switching to Sparse GP
        self.declare_parameter('use_multi_fidelity', True)
        self.declare_parameter('model_path', '')
        self.declare_parameter('initial_rho', 1.0)
        self.declare_parameter('fix_rho', False)
        self.declare_parameter('low_fi_max_prior', 100.0) # New: Cap the prior from low-fi model
        self.declare_parameter('landmark_buffer_size', 500)
        self.declare_parameter('landmark_spatial_dist', 0.2)
        self.declare_parameter('peak_confidence_threshold', 1.0) # Max allowed std dev
        self.declare_parameter('rad_smoothing_window', 5)        # Rolling average before training
        self.declare_parameter('gp_noise_variance_min', 0.5)     # Floor on GP likelihood variance
        self.declare_parameter('landmark_background_min', 0.3)   # Min background used for landmark threshold

        self.predict_srv = self.create_service(GPPredict, 'predict_gp', self.predict_callback)
        self.peaks_srv = self.create_service(FindPeaks, 'find_rad_peaks', self.find_peaks_callback)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.historical_samples = []
        self.landmark_samples = [] # New: Buffer for high-radiation points
        self.last_sample_pos = None
        self.gp_model = None
        self.data_lock = threading.Lock()
        self.current_background = 0.0
        self._rad_smooth_buf = []   # Rolling window for pre-smoothing raw readings
        
        # Multi-fidelity state
        self.gp_lo = None
        self.load_low_fi_model()

        self.rad_sub = self.create_subscription(
            Float64MultiArray,
            '/rad',
            self.rad_callback,
            10
        )

        self.background_sub = self.create_subscription(
            Float64,
            '/rad_field_server/background_radiation',
            self.background_callback,
            10
        )
        
        use_multi_fidelity = self.get_parameter('use_multi_fidelity').get_parameter_value().bool_value
        if use_multi_fidelity:
            self.get_logger().info('GP Predictor Node initialized with Multi-Fidelity (ARGP) support.')
        else:
            self.get_logger().info('GP Predictor Node initialized without Multi-Fidelity (ARGP) support.')

    def load_low_fi_model(self):
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if not model_path or not os.path.exists(model_path):
            self.get_logger().warn(f"Model path '{model_path}' invalid or empty. Multi-fidelity disabled.")
            return

        try:
            with open(model_path, 'rb') as f:
                data = pickle.load(f)
            
            if isinstance(data, dict) and 'gp_lo' in data:
                self.gp_lo = data['gp_lo']
                self.get_logger().info(f"Loaded low-fidelity model from {model_path}")
            else:
                self.get_logger().error(f"Pickle at {model_path} does not contain 'gp_lo'.")
        except Exception as e:
            self.get_logger().error(f"Failed to load low-fi model: {e}")

    def background_callback(self, msg):
        # Handle both Float64 and Float64MultiArray (simulation might use MultiArray)
        if hasattr(msg, 'data'):
            if isinstance(msg.data, list) or isinstance(msg.data, np.ndarray):
                if len(msg.data) > 0:
                    self.current_background = msg.data[0]
            else:
                self.current_background = msg.data

    def rad_callback(self, msg):
        if len(msg.data) < 2:
            return

        try:
            sensor_frame = self.get_parameter('sensor_frame').get_parameter_value().string_value
            map_frame = self.get_parameter('map_frame').get_parameter_value().string_value

            transform = self.tf_buffer.lookup_transform(map_frame, sensor_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.1))

            x = transform.transform.translation.x
            y = transform.transform.translation.y
            raw_value = msg.data[1]

            # Rolling average to suppress transient Geiger-counter spikes before
            # they reach GP training data or landmark storage.
            win_size = self.get_parameter('rad_smoothing_window').get_parameter_value().integer_value
            self._rad_smooth_buf.append(raw_value)
            if len(self._rad_smooth_buf) > win_size:
                self._rad_smooth_buf.pop(0)
            rad_value = sum(self._rad_smooth_buf) / len(self._rad_smooth_buf)

            with self.data_lock:
                # Spatial downsampling for regular buffer
                if self.last_sample_pos is not None:
                    dist_sq = (x - self.last_sample_pos[0])**2 + (y - self.last_sample_pos[1])**2
                    if dist_sq < 0.04: # 0.2m move required
                        return

                # Use a copy for historical_samples so landmarks can be updated
                # independently without aliasing mutations back into history.
                sample = {'x': x, 'y': y, 'value': rad_value}
                self.historical_samples.append(sample)
                self.last_sample_pos = (x, y)

                # Landmark logic: Keep representative values for high-radiation locations.
                # Use a minimum background floor so we don't landmark every reading
                # when the background topic hasn't published yet.
                bg_min = self.get_parameter('landmark_background_min').get_parameter_value().double_value
                effective_bg = max(self.current_background, bg_min)
                if rad_value > effective_bg * 1.5:
                    is_far_from_existing = True
                    ls_min_dist_sq = (self.get_parameter('landmark_spatial_dist').get_parameter_value().double_value)**2

                    for ls in self.landmark_samples:
                        ls_dist_sq = (x - ls['x'])**2 + (y - ls['y'])**2
                        if ls_dist_sq < ls_min_dist_sq:
                            # Preserve the peak value seen at this location.
                            # The 5-sample rolling average upstream already suppresses transient
                            # spikes before they reach here, so the smoothed rad_value is a
                            # real sustained reading — it should never drive the landmark down.
                            # Average the position to converge toward the true source location,
                            # but keep the maximum radiation value so confirmed sources never fade.
                            ls['value'] = max(ls['value'], rad_value)
                            ls['x'] = (ls['x'] + x) / 2.0
                            ls['y'] = (ls['y'] + y) / 2.0
                            is_far_from_existing = False
                            break

                    if is_far_from_existing:
                        # Store a separate copy so mutations to the landmark dict
                        # don't alias back and corrupt the historical_samples entry.
                        self.landmark_samples.append({'x': x, 'y': y, 'value': rad_value})
                        # Cap landmark buffer by keeping highest values
                        if len(self.landmark_samples) > self.get_parameter('landmark_buffer_size').get_parameter_value().integer_value:
                            self.landmark_samples.sort(key=lambda s: s['value'], reverse=True)
                            self.landmark_samples.pop()

                # Cap the regular buffer
                if len(self.historical_samples) > 1000:
                    self.historical_samples.pop(0)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            pass

    def _build_gp_model(self, observed_x, observed_y, observed_vals_mean, mean_value=None):
        # Optimization: Use internal buffer if request is empty
        if len(observed_x) == 0:
            with self.data_lock:
                # Merge historical and landmarks
                samples = list(self.historical_samples)
                for ls in self.landmark_samples:
                    # Check if this landmark is already represented in recent samples
                    is_redundant = False
                    for s in samples:
                        if (ls['x'] - s['x'])**2 + (ls['y'] - s['y'])**2 < 0.01: # 0.1m overlap
                            is_redundant = True
                            break
                    if not is_redundant:
                        samples.append(ls)

            if not samples:
                return None, None
            observed_x = np.array([s['x'] for s in samples])
            observed_y = np.array([s['y'] for s in samples])
            observed_vals_mean = np.array([s['value'] for s in samples])
            if mean_value is None:
                mean_value = self.current_background

        training_points = np.vstack((observed_x, observed_y)).T
        training_values = observed_vals_mean[:, None]

        num_points = training_points.shape[0]
        sparse_thresh = self.get_parameter('sparse_threshold').get_parameter_value().integer_value
        
        kernel_type = self.get_parameter('kernel.type').get_parameter_value().string_value
        lengthscale = self.get_parameter('kernel.lengthscale').get_parameter_value().double_value

        if kernel_type.lower() == 'matern52':
            kernel = GPy.kern.Matern52(input_dim=2, variance=1., lengthscale=lengthscale)
        elif kernel_type.lower() == 'matern32':
            kernel = GPy.kern.Matern32(input_dim=2, variance=1., lengthscale=lengthscale)
        elif kernel_type.lower() == 'exponential' or kernel_type.lower() == 'matern12':
            kernel = GPy.kern.Exponential(input_dim=2, variance=1., lengthscale=lengthscale)
        else:
            kernel = GPy.kern.RBF(input_dim=2, variance=1., lengthscale=lengthscale)

        # Include a mean function
        mf = None
        use_mf = self.get_parameter('use_multi_fidelity').get_parameter_value().bool_value
        init_rho = self.get_parameter('initial_rho').get_parameter_value().double_value
        max_prior = self.get_parameter('low_fi_max_prior').get_parameter_value().double_value
        
        if use_mf and self.gp_lo is not None:
            # Multi-fidelity mode: use low-fi model as mean with trainable rho
            mf = LowFiMapping(self.gp_lo, initial_rho=init_rho, max_prior=max_prior)
        elif mean_value is not None:
            # Single-fidelity mode: use constant background mean
            mf = GPy.mappings.Constant(input_dim=2, output_dim=1, value=mean_value)

        if num_points > sparse_thresh:
            # Use Sparse GP for O(NM^2) instead of O(N^3)
            num_inducing = 50 # Constant complexity now
            gp_model = GPy.models.SparseGPRegression(training_points, training_values, kernel, num_inducing=num_inducing, mean_function=mf, normalizer=False)
        else:
            gp_model = GPy.models.GPRegression(training_points, training_values, kernel, mean_function=mf, normalizer=False)
        
        # Enforce a noise variance floor so optimize() cannot collapse noise to near-zero
        # and interpolate exactly through spike values.
        noise_min = self.get_parameter('gp_noise_variance_min').get_parameter_value().double_value
        if noise_min > 0.0:
            gp_model.likelihood.variance.constrain_bounded(noise_min, 1e4)

        # Apply Fixed Lengthscale if requested
        if self.get_parameter('fix_lengthscale').get_parameter_value().bool_value:
            gp_model.kern.lengthscale.fix()
            self.get_logger().info(f"Fixed kernel lengthscale to {lengthscale}", throttle_duration_sec=60.0)

        # Apply Fixed Rho / Mean if requested
        # Auto-fix rho if no significant radiation detected to avoid noise bias
        max_train_val = np.max(training_values) if training_values.size > 0 else 0.0
        if use_mf and hasattr(gp_model.mean_function, 'rho') and max_train_val < (mean_value * 1.5):
             gp_model.mean_function.rho.fix()
             # self.get_logger().info("Auto-fixed rho due to low signal-to-noise", throttle_duration_sec=60.0)

        if self.get_parameter('fix_rho').get_parameter_value().bool_value:
            if use_mf and hasattr(gp_model.mean_function, 'rho'):
                gp_model.mean_function.rho.fix()
                self.get_logger().info(f"Fixed Multi-Fi scaling (rho) to {init_rho}", throttle_duration_sec=60.0)
            elif not use_mf and hasattr(gp_model.mean_function, 'C'):
                gp_model.mean_function.C.fix()
                self.get_logger().info(f"Fixed Single-GP background mean to {mean_value}", throttle_duration_sec=60.0)

        return gp_model, training_points

    def find_peaks_callback(self, request, response):
        with self.data_lock:
            samples = list(self.historical_samples)

        observed_x = np.array([s['x'] for s in samples])
        observed_y = np.array([s['y'] for s in samples])
        observed_vals_mean = np.array([s['value'] for s in samples])

        gp_model, training_points = self._build_gp_model(observed_x, observed_y, observed_vals_mean, request.mean_value)
        
        target_model = None
        search_bounds = None

        if gp_model is not None:
            target_model = gp_model
            min_x, min_y = training_points.min(axis=0)
            max_x, max_y = training_points.max(axis=0)
            search_bounds = [(min_x, max_x), (min_y, max_y)]
        else:
            use_mf = self.get_parameter('use_multi_fidelity').get_parameter_value().bool_value
            if use_mf and self.gp_lo is not None:
                target_model = self.gp_lo
                # Use a default large bound for low-fi peak finding if no data
                search_bounds = [(-50.0, 50.0), (-50.0, 50.0)] 
            else:
                return response

        def objective_function(x):
            x = np.atleast_2d(x)
            mean, _ = target_model.predict(x)
            return -mean[0, 0]

        found_peaks = []
        for i in range(request.num_starts):
            start_point = np.random.uniform(low=[search_bounds[0][0], search_bounds[1][0]], 
                                           high=[search_bounds[0][1], search_bounds[1][1]])
            
            result = minimize(
                fun=objective_function,
                x0=start_point,
                method='L-BFGS-B',
                bounds=search_bounds
            )
            
            if result.success:
                peak_x, peak_y = result.x
                peak_val = -result.fun
                
                # Check confidence at peak
                _, peak_var = target_model.predict(np.atleast_2d([peak_x, peak_y]))
                peak_std = np.sqrt(peak_var[0, 0])
                conf_thresh = self.get_parameter('peak_confidence_threshold').get_parameter_value().double_value
                
                if peak_std > conf_thresh:
                    # self.get_logger().debug(f"Rejecting peak at ({peak_x:.2f}, {peak_y:.2f}) due to low confidence (std={peak_std:.2f})")
                    continue

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
            
            gp_model, _ = self._build_gp_model(observed_x, observed_y, observed_vals_mean, request.mean_value)
            
            points_to_predict = np.array([[p.x, p.y] for p in request.points_to_predict])
            if points_to_predict.shape[0] == 0:
                return response

            if gp_model is None:
                # ... (keep existing low-fi fallback)
                use_mf = self.get_parameter('use_multi_fidelity').get_parameter_value().bool_value
                if use_mf and self.gp_lo is not None:
                    mean_prediction, var_prediction = self.gp_lo.predict(points_to_predict)
                    std_dev_prediction = np.sqrt(var_prediction)
                    response.predicted_mean = np.maximum(0, mean_prediction).flatten().tolist() 
                    response.predicted_std_dev = np.maximum(0, std_dev_prediction).flatten().tolist()
                return response

            # 1. Optimize hyperparameters BEFORE prediction
            try:
                gp_model.optimize(messages=False, max_iters=50)
            except Exception as opt_e:
                self.get_logger().warn(f"Optimization failed: {opt_e}")

            # 2. Perform Prediction
            mean_prediction, var_prediction = gp_model.predict(points_to_predict)
            std_dev_prediction = np.sqrt(var_prediction)

            response.predicted_mean = np.maximum(0, mean_prediction).flatten().tolist() 
            response.predicted_std_dev = np.maximum(0, std_dev_prediction).flatten().tolist()

            # 3. Fill Diagnostics
            try:
                response.log_marginal_likelihood = float(gp_model.log_likelihood())
                response.kernel_lengthscale = float(gp_model.kern.lengthscale[0])
                response.kernel_variance = float(gp_model.kern.variance[0])
                if hasattr(gp_model, 'likelihood'):
                    response.noise_variance = float(gp_model.likelihood.variance[0])
                
                # Training MSE (Residual after GP fit)
                mu_train, _ = gp_model.predict(gp_model.X)
                response.training_mse = float(np.mean((mu_train - gp_model.Y)**2))
                
                # Prior MSE (Error if we only used the mean function)
                if gp_model.mean_function is not None:
                    mu_prior = gp_model.mean_function.f(gp_model.X)
                    response.prior_mse = float(np.mean((mu_prior - gp_model.Y)**2))
                    
                    # Extract learned rho or constant value
                    if isinstance(gp_model.mean_function, LowFiMapping):
                        response.rho = float(gp_model.mean_function.rho[0])
                    elif isinstance(gp_model.mean_function, GPy.mappings.Constant):
                        response.rho = float(gp_model.mean_function.C[0])
                    
                    self.get_logger().info(f"GP Fit: LML={response.log_marginal_likelihood:.2f}, Rho={response.rho:.3f}, Resid MSE={response.training_mse:.4e}", throttle_duration_sec=10.0)
                else:
                    response.prior_mse = response.training_mse
                    response.rho = 1.0
                
            except Exception as diag_e:
                self.get_logger().warn(f"Failed to calculate GP diagnostics: {diag_e}")
            
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
