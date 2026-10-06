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

class LowFidelityPrior:
    """The offline multi-fidelity model, exposed in live sensor units (mrem/hr).

    The offline pipeline fits its GPs to log(1+y) targets and models the
    high-fidelity field as

        log(1 + f_h(x)) = log(1 + f_l(x)) + delta(x),

    so the fused estimate in linear units is expm1(mu_l + mu_delta). Two things
    used to go wrong at this boundary, and both are handled here so that every
    consumer in this node sees the prior in the same units as /rad:

    1. No inverse transform was applied. A log-space quantity (range ~0.2 to
       2.9 for the UFTR model) was used directly as a mean function for
       observations in mrem/hr (range ~0.27 to 17.4), leaving the trainable rho
       to absorb the unit mismatch. rho consequently fitted to ~2.8-3.3 and
       carried no physical meaning; with the transform applied it fits to
       ~0.75-0.93, i.e. close to unity as expected. A side effect was that
       ``low_fi_max_prior`` was being compared against log-space values, so a
       cap intended as "100 mrem/hr" either never bound at all or, when tuned
       down to work around the symptom, clamped the prior to a constant.

    2. The discrepancy GP was ignored. The serialized model contains both
       ``gp_lo`` and ``gp_delta``, but only ``gp_lo`` was ever read, so the
       spatially varying correction learned from the A-11 control survey never
       reached the live map: only its global level was recovered, via rho.

    ``predict`` returns (mean, variance) in mrem/hr and is signature-compatible
    with a GPy model, so this object can stand in wherever the raw ``gp_lo``
    was used before.
    """

    def __init__(self, gp_lo, gp_delta=None, transform='log1p', use_discrepancy=True):
        self.gp_lo = gp_lo
        self.gp_delta = gp_delta if use_discrepancy else None
        if transform not in ('log1p', 'none'):
            raise ValueError(f"unknown prior transform '{transform}'")
        self.transform = transform

    @property
    def uses_discrepancy(self):
        return self.gp_delta is not None

    def predict(self, X):
        X = np.atleast_2d(X)
        mu, var = self.gp_lo.predict(X)
        if self.gp_delta is not None:
            mu_d, var_d = self.gp_delta.predict(X)
            mu = mu + mu_d
            # GP_lo and GP_delta are fitted on disjoint data and delta is
            # conditioned on GP_lo's point estimate, so the modular estimator
            # treats them as independent and the variances add.
            var = var + var_d
        if self.transform == 'log1p':
            # Delta-method back-transform: d/dm expm1(m) = exp(m).
            var = np.square(np.exp(mu)) * var
            mu = np.expm1(mu)
        # A dose rate cannot be negative. The fused log-space mean can dip below
        # zero where the discrepancy GP corrects a cold region downwards, which
        # after expm1 yields a small negative rate (about a quarter of the UFTR
        # cell, down to -0.28 mrem/hr). Left unclamped this biases the live fit
        # low exactly where the field is weakest, so the mean is floored here.
        # The variance is deliberately left untouched: the uncertainty at those
        # cells is real and should still widen the predictive interval.
        return np.maximum(mu, 0.0), var

    def predict_std(self, X):
        mu, var = self.predict(X)
        return mu, np.sqrt(np.maximum(var, 0.0))


class LowFiMapping(GPy.core.Mapping):
    """rho * min(prior(x), C_max) as a GPy mean function, in mrem/hr."""

    def __init__(self, prior, initial_rho=1.0, max_prior=100.0):
        super().__init__(input_dim=2, output_dim=1)
        self.prior = prior
        # Create rho as a GPy parameter so it can be optimized
        self.rho = GPy.core.parameterization.Param('rho', initial_rho)
        self.max_prior = max_prior
        self.link_parameter(self.rho)

    def _base(self, X):
        m, _ = self.prior.predict(np.atleast_2d(X))
        # Cap in mrem/hr, after the inverse transform, so the bound means what
        # its units say and cannot be reached by a log-space value.
        return np.minimum(m, self.max_prior)

    def f(self, X):
        return self.rho * self._base(X)

    def update_gradients(self, dL_dF, X):
        # dL/drho = sum( dL/df * df/drho ) = sum( dL/df * base )
        self.rho.gradient = np.sum(dL_dF * self._base(X))

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
        self.declare_parameter('low_fi_max_prior', 100.0) # Cap on the prior contribution, in mrem/hr
        # The offline pipeline trains on log(1+y), so the serialized model must
        # be inverse-transformed before it can be used against /rad readings in
        # mrem/hr. Set to 'none' only for a prior that was fitted in linear units.
        self.declare_parameter('prior_transform', 'log1p')
        # Whether to include the offline discrepancy GP (the A-11 correction) in
        # the prior mean. False reproduces the pre-fix behaviour of using the
        # low-fidelity process alone.
        self.declare_parameter('use_discrepancy', True)
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

        # Warm-start cache: last successfully optimized hyperparameters.
        # _build_gp_model seeds new models from these instead of fixed defaults so
        # optimize() converges in far fewer L-BFGS iterations each call, and so
        # find_peaks_callback (which never calls optimize() itself) searches against
        # a fitted model instead of default/untrained hyperparameters.
        self._warm_lengthscale = None
        self._warm_variance = None
        self._warm_noise_variance = None
        self._warm_rho = None

        # Multi-fidelity state
        self.gp_lo = None
        self.prior = None
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

        transform = self.get_parameter('prior_transform').get_parameter_value().string_value
        use_delta = self.get_parameter('use_discrepancy').get_parameter_value().bool_value

        try:
            with open(model_path, 'rb') as f:
                data = pickle.load(f)

            if not (isinstance(data, dict) and 'gp_lo' in data):
                self.get_logger().error(f"Pickle at {model_path} does not contain 'gp_lo'.")
                return

            gp_lo = data['gp_lo']
            gp_delta = data.get('gp_delta')
            if gp_delta is None and use_delta:
                self.get_logger().warn(
                    "Model contains no 'gp_delta'; the A-11 discrepancy correction "
                    "will not be applied and the prior is the low-fidelity field alone.")

            self.prior = LowFidelityPrior(gp_lo, gp_delta, transform=transform,
                                          use_discrepancy=use_delta)
            self.gp_lo = gp_lo   # retained for diagnostics only

            # Sanity-check the configured transform against the model's own
            # training targets. A log(1+y) model of this field tops out near 3;
            # a linear one near 17. Getting this wrong silently is exactly the
            # failure this check exists to catch, so warn loudly rather than
            # guessing.
            try:
                y_max = float(np.max(gp_lo.Y))
                if transform == 'log1p' and y_max > 10.0:
                    self.get_logger().warn(
                        f"prior_transform='log1p' but the model's training targets reach "
                        f"{y_max:.2f}, which looks like linear mrem/hr. Check the setting.")
                elif transform == 'none' and y_max < 5.0:
                    self.get_logger().warn(
                        f"prior_transform='none' but the model's training targets only reach "
                        f"{y_max:.2f}, which looks like log(1+y). Check the setting.")
                span = self.prior.predict(gp_lo.X)[0]
                self.get_logger().info(
                    f"Loaded prior from {model_path}: transform={transform}, "
                    f"discrepancy={'on' if self.prior.uses_discrepancy else 'off'}, "
                    f"stored target max={y_max:.3f}, "
                    f"prior range at training points = "
                    f"[{float(np.min(span)):.3f}, {float(np.max(span)):.3f}] mrem/hr")
            except Exception as diag_e:
                self.get_logger().info(f"Loaded prior from {model_path} "
                                       f"(range diagnostics unavailable: {diag_e})")
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

        # Warm-start from the last optimized fit (if any) instead of always
        # starting L-BFGS cold from the configured defaults. Skipped when
        # fix_lengthscale is set, since that value must stay exactly as configured.
        fix_lengthscale = self.get_parameter('fix_lengthscale').get_parameter_value().bool_value
        init_lengthscale = lengthscale if (fix_lengthscale or self._warm_lengthscale is None) else self._warm_lengthscale
        init_variance = 1. if self._warm_variance is None else self._warm_variance

        if kernel_type.lower() == 'matern52':
            kernel = GPy.kern.Matern52(input_dim=2, variance=init_variance, lengthscale=init_lengthscale)
        elif kernel_type.lower() == 'matern32':
            kernel = GPy.kern.Matern32(input_dim=2, variance=init_variance, lengthscale=init_lengthscale)
        elif kernel_type.lower() == 'exponential' or kernel_type.lower() == 'matern12':
            kernel = GPy.kern.Exponential(input_dim=2, variance=init_variance, lengthscale=init_lengthscale)
        else:
            kernel = GPy.kern.RBF(input_dim=2, variance=init_variance, lengthscale=init_lengthscale)

        # Include a mean function
        mf = None
        use_mf = self.get_parameter('use_multi_fidelity').get_parameter_value().bool_value
        init_rho = self.get_parameter('initial_rho').get_parameter_value().double_value
        max_prior = self.get_parameter('low_fi_max_prior').get_parameter_value().double_value
        fix_rho = self.get_parameter('fix_rho').get_parameter_value().bool_value
        if not fix_rho and self._warm_rho is not None:
            init_rho = self._warm_rho

        if use_mf and self.prior is not None:
            # Multi-fidelity mode: offline prior as mean function with trainable rho
            mf = LowFiMapping(self.prior, initial_rho=init_rho, max_prior=max_prior)
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
        # chap5 sharpness-RECOVERY pilot: env RAMONES_GP_NOISE_MIN lowers the noise
        # floor -> tighter interpolation -> sharper posterior-mean peaks -> peak argmax
        # closer to the true source -> better localization. Unlike the (invalid) gpv
        # kern-variance revert, this stays WITHIN the stable regime: the :422 kernel-
        # variance bound is untouched, so no std~100 collapse. Default (unset) = current.
        _nm_env = os.environ.get('RAMONES_GP_NOISE_MIN')
        if _nm_env is not None:
            try:
                noise_min = float(_nm_env)
                self.get_logger().warn(f"RAMONES_GP_NOISE_MIN={noise_min} (sharpness-recovery pilot)",
                                       throttle_duration_sec=60.0)
            except ValueError:
                pass
        if noise_min > 0.0:
            gp_model.likelihood.variance.constrain_bounded(noise_min, 1e4)
        if self._warm_noise_variance is not None:
            gp_model.likelihood.variance = max(self._warm_noise_variance, noise_min)

        # Kernel (signal) variance has no floor of its own -- with
        # fix_lengthscale and fix_rho both True by default, this is the ONLY
        # hyperparameter optimize() is actually free to move. Observed
        # directly: an unconstrained fit collapsed kernel variance toward
        # zero, which made the posterior predictive variance at any point
        # not essentially on top of a training sample explode (std ~100),
        # and since find_peaks_callback never re-optimizes and only warm-
        # starts from whatever predict_gp last cached, that one pathological
        # fit then silently rejected every peak candidate for low confidence
        # on every find_peaks call until a later predict_gp call happened to
        # recover it. Bounding it the same way as the noise floor above
        # prevents the optimizer from reaching that regime at all.
        #
        # ISOLATION TOGGLE (chap5 GP-vintage batch): set env RAMONES_GP_KERN_UNBOUNDED=1
        # to SKIP this bound and reproduce the pre-2026-07-20 pre-fix behaviour on the
        # current stack -- used to size the residual (non-escape) driver of the
        # rt->effret H1 flip. Default (unset/0) keeps the fix. Env-var, not a ROS param,
        # to avoid launch-file plumbing; propagates batch-shell -> run_trial -> launch -> node.
        if os.environ.get('RAMONES_GP_KERN_UNBOUNDED', '0') != '1':
            gp_model.kern.variance.constrain_bounded(1e-3, 1e4)
        else:
            self.get_logger().warn(
                "RAMONES_GP_KERN_UNBOUNDED=1: kernel-variance bound DISABLED "
                "(pre-fix GP vintage -- isolation experiment)",
                throttle_duration_sec=60.0)

        # Apply Fixed Lengthscale if requested
        if fix_lengthscale:
            gp_model.kern.lengthscale.fix()
            self.get_logger().info(f"Fixed kernel lengthscale to {lengthscale}", throttle_duration_sec=60.0)

        # Apply Fixed Rho / Mean if requested
        # Auto-fix rho if no significant radiation detected to avoid noise bias
        max_train_val = np.max(training_values) if training_values.size > 0 else 0.0
        if use_mf and hasattr(gp_model.mean_function, 'rho') and max_train_val < (mean_value * 1.5):
             gp_model.mean_function.rho.fix()
             # self.get_logger().info("Auto-fixed rho due to low signal-to-noise", throttle_duration_sec=60.0)

        if fix_rho:
            # Checked by attribute (mean function type), not use_mf: if multi-fidelity
            # was requested but the low-fi model failed to load, gp_model.mean_function
            # is still a Constant fallback and must be fixed too, or its 'C' offset
            # floats free and gets refit to the training data on every call.
            if hasattr(gp_model.mean_function, 'rho'):
                gp_model.mean_function.rho.fix()
                self.get_logger().info(f"Fixed Multi-Fi scaling (rho) to {init_rho}", throttle_duration_sec=60.0)
            elif hasattr(gp_model.mean_function, 'C'):
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
        self.get_logger().info(
            f"[DIAG find_peaks] samples={len(samples)} gp_model_built={gp_model is not None}",
            throttle_duration_sec=5.0)

        target_model = None
        search_bounds = None

        if gp_model is not None:
            target_model = gp_model
            min_x, min_y = training_points.min(axis=0)
            max_x, max_y = training_points.max(axis=0)
            # scipy's L-BFGS-B raises inside old_bound_to_new() on a degenerate
            # (min == max) bound, which happens whenever training points haven't
            # spread out on an axis yet (e.g. early in a mission, near the spawn
            # point). Pad so the search region is never zero-width.
            pad = 0.5
            if max_x - min_x < 1e-6:
                min_x -= pad
                max_x += pad
            if max_y - min_y < 1e-6:
                min_y -= pad
                max_y += pad
            search_bounds = [(min_x, max_x), (min_y, max_y)]
        else:
            use_mf = self.get_parameter('use_multi_fidelity').get_parameter_value().bool_value
            if use_mf and self.prior is not None:
                # LowFidelityPrior.predict is signature-compatible with a GPy
                # model and returns mrem/hr, so the peak thresholds downstream
                # are compared against the same units the sensor reports.
                target_model = self.prior
                # Use a default large bound for low-fi peak finding if no data
                search_bounds = [(-50.0, 50.0), (-50.0, 50.0)] 
            else:
                return response

        def objective_function(x):
            x = np.atleast_2d(x)
            mean, _ = target_model.predict(x)
            return -mean[0, 0]

        found_peaks = []
        n_opt_fail = 0
        n_conf_reject = 0
        n_dup_reject = 0
        max_peak_val_seen = float('-inf')
        min_std_at_success = float('inf')
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
                max_peak_val_seen = max(max_peak_val_seen, peak_val)

                # Check confidence at peak
                _, peak_var = target_model.predict(np.atleast_2d([peak_x, peak_y]))
                peak_std = np.sqrt(peak_var[0, 0])
                min_std_at_success = min(min_std_at_success, peak_std)
                conf_thresh = self.get_parameter('peak_confidence_threshold').get_parameter_value().double_value
                # chap5 recovery pilot: lowering the noise floor shifts the std
                # distribution, so allow co-tuning the accept gate to hold yield.
                _ct_env = os.environ.get('RAMONES_GP_CONF_THRESH')
                if _ct_env is not None:
                    try:
                        conf_thresh = float(_ct_env)
                    except ValueError:
                        pass

                # This threshold is in mrem/hr and was bench-tuned against the
                # fitted GP, whose std is floored near 0.71 by the noise-variance
                # bound. Before the prior units were corrected, the cold-start
                # branch above compared a *log-space* std (median 0.54 over the
                # cell) against the same number, so it was spuriously permissive
                # and accepted phantom peaks before any live data had arrived.
                # Both branches now report mrem/hr, so the filter means the same
                # thing in each; the cold-start branch is correspondingly
                # stricter than it used to be (linear std has median 0.93).
                if peak_std > conf_thresh:
                    n_conf_reject += 1
                    continue

                is_duplicate = False
                for existing_peak in found_peaks:
                    dist_sq = (existing_peak[0] - peak_x)**2 + (existing_peak[1] - peak_y)**2
                    if dist_sq < (self.get_parameter('kernel.lengthscale').get_parameter_value().double_value)**2:
                        is_duplicate = True
                        break

                if not is_duplicate:
                    found_peaks.append((peak_x, peak_y, peak_val))
                else:
                    n_dup_reject += 1
            else:
                n_opt_fail += 1

        self.get_logger().info(
            f"[DIAG find_peaks] starts={request.num_starts} opt_fail={n_opt_fail} "
            f"conf_reject={n_conf_reject} dup_reject={n_dup_reject} accepted={len(found_peaks)} "
            f"max_peak_val={max_peak_val_seen:.3f} min_std_at_success={min_std_at_success:.3f}",
            throttle_duration_sec=5.0)

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
                if use_mf and self.prior is not None:
                    mean_prediction, std_dev_prediction = self.prior.predict_std(points_to_predict)
                    response.predicted_mean = np.maximum(0, mean_prediction).flatten().tolist() 
                    response.predicted_std_dev = np.maximum(0, std_dev_prediction).flatten().tolist()
                return response

            # 1. Optimize hyperparameters BEFORE prediction
            try:
                gp_model.optimize(messages=False, max_iters=50)
                # Cache the fit so the next call (and find_peaks_callback, which
                # never optimizes on its own) warm-starts from here instead of
                # cold defaults.
                self._warm_lengthscale = float(gp_model.kern.lengthscale[0])
                self._warm_variance = float(gp_model.kern.variance[0])
                self._warm_noise_variance = float(gp_model.likelihood.variance[0])
                if isinstance(gp_model.mean_function, LowFiMapping):
                    self._warm_rho = float(gp_model.mean_function.rho[0])
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
