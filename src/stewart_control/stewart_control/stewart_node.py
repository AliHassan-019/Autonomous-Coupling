#!/home/rem/rtimulib-env/bin/python3

import numpy as np
import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from stewart_control.actuator_calibration import ActuatorCalibration
from stewart_control.inv_kinematics import StewartPlatform
from stewart_control.config_loader import get_config
from stewart_control.runtime_state import RuntimeStateManager


class StewartNode(Node):
    CM_TO_M = 0.01
    HOME_TOLERANCE_CM = 0.3
    HOME_STABLE_READS = 5
    SHUTDOWN_TIMEOUT_S = 20.0
    SERIAL_READ_MAX_LINES = 5
    FEEDBACK_STATE_SAVE_INTERVAL_S = 1.0

    def __init__(self):
        super().__init__("stewart_node")

        cfg = get_config()
        sp = cfg["stewart_platform"]
        ser_cfg = cfg["serial"]
        act = cfg["actuators"]
        auto_cfg = cfg.get("automatic_control", {})
        

        # Publisher pour les longueurs calculées à envoyer à l'Arduino
        self.publisher_ = self.create_publisher(
            Float32MultiArray, "stewart/longueurs", 1
        )

        # Publisher pour le feedback reçu de l'Arduino
        self.publisher_feedback = self.create_publisher(
            Float32MultiArray, "feedback_motors", 1
        )

        # Status messages for GUI monitoring
        self.status_publisher = self.create_publisher(String, "stewart_status", 10)

        # Subscribers
        self.pos_subscription = self.create_subscription(
            Float32MultiArray, "aruco_position", self.position_callback, 1
        )
        self.ori_subscription = self.create_subscription(
            Float32MultiArray,
            "aruco_orientation",
            self.aruco_orientation_callback,
            1,
        )
        self.fusion_subscription = self.create_subscription(
            Float32MultiArray, "F_orientation", self.fusion_callback, 1
        )
        self.fused_pose_subscription = self.create_subscription(
            Float32MultiArray, "F_pose", self.fused_pose_callback, 1
        )

        # Configuration UART vers Arduino
        self.ser = serial.Serial(
            ser_cfg["port"], ser_cfg["baudrate"], timeout=ser_cfg["timeout"]
        )
        time.sleep(ser_cfg["startup_delay"])

        # Vider le buffer série
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        # Configuration de la plateforme Stewart
        self.platform = StewartPlatform(
            sp["radius_base"],
            sp["radius_platform"],
            sp["gamma_base"],
            sp["gamma_platform"],
            home_position=sp["home_position"],
        )
        self.L0 = np.array(sp["L0"])

        # Variables d’état
        self.position = np.zeros(3)
        self.orientation = np.zeros(3)
        self.raw_position = np.zeros(3)
        self.raw_orientation = np.zeros(3)
        self.fused_orientation = None
        self.fused_position = None
        self.last_fused_pose_time = 0.0
        self.last_fusion_time = 0.0
        self.filtered_position = np.zeros(3)
        self.filtered_orientation = np.zeros(3)
        self.control_position = np.zeros(3)
        self.control_orientation = np.zeros(3)
        self.continuous = bool(auto_cfg.get("continuous", True))
        self.last_deplacement = [0.0] * 6
        self.last_sent_targets = np.zeros(ActuatorCalibration.MOTOR_COUNT, dtype=float)
        self.length_threshold = act["length_threshold"]
        self.calibration = ActuatorCalibration.from_config(act)
        self.home_vector = np.array(
            act.get("home_vector", [0.0] * ActuatorCalibration.MOTOR_COUNT),
            dtype=float,
        )
        self.stop_tolerance_cm = float(auto_cfg.get("stop_tolerance_cm", 3.0))
        self.invert_marker_pose = bool(auto_cfg.get("invert_marker_pose", True))
        self.use_marker_orientation = bool(auto_cfg.get("use_marker_orientation", True))
        self.use_fused_pose = bool(auto_cfg.get("use_fused_pose", True))
        self.use_fused_orientation = bool(auto_cfg.get("use_fused_orientation", True))
        self.use_orientation_control = bool(
            auto_cfg.get(
                "use_orientation_control",
                self.use_marker_orientation
                or self.use_fused_pose
                or self.use_fused_orientation,
            )
        )
        self.fusion_timeout_s = float(auto_cfg.get("fusion_timeout_s", 0.2))
        self.command_debounce_s = float(auto_cfg.get("command_debounce_s", 0.02))
        self.position_alpha = float(auto_cfg.get("position_alpha", 0.25))
        self.orientation_alpha = float(auto_cfg.get("orientation_alpha", 0.2))
        self.command_alpha = float(auto_cfg.get("command_alpha", 0.45))
        self.position_deadband_m = float(auto_cfg.get("position_deadband_m", 0.0015))
        self.orientation_deadband_deg = float(
            auto_cfg.get("orientation_deadband_deg", 0.35)
        )
        self.command_deadband_cm = float(auto_cfg.get("command_deadband_cm", 0.12))
        self.blind_approach_enabled = bool(
            auto_cfg.get("blind_approach_enabled", False)
        )
        self.blind_approach_distance_cm = float(
            auto_cfg.get("blind_approach_distance_cm", 4.0)
        )
        self.blind_approach_armed = False
        self.blind_approach_done = False
        self.last_position_update_time = 0.0
        self.pending_update = False
        self.last_input_time = 0.0
        self.last_stop_state = False
        self.pose_initialized = False
        self.limit_status_active = False
        self.runtime_state = RuntimeStateManager()
        self.bypass_runtime_state = bool(auto_cfg.get("bypass_runtime_state", False))
        self.motion_enabled = (
            self.bypass_runtime_state or self.runtime_state.begin_motion_session()
        )
        self.latest_feedback = None
        self.last_feedback_state_save_time = 0.0

        if self.bypass_runtime_state:
            self.get_logger().warn(
                "Automatic control runtime-state bypass is enabled. Motion will be "
                "allowed without trusted recovery."
            )
        elif self.motion_enabled:
            self.get_logger().info("Trusted startup state detected. Motion enabled.")
        else:
            self.get_logger().error(
                "Position unknown after previous shutdown. Manual recovery is required "
                "before automatic control is allowed."
            )

        # Timer pour lecture série
        self.create_timer(act["feedback_timer_period"], self.read_feedback)
        self.create_timer(0.01, self.flush_pending_update)
        self.create_timer(0.02, self.check_blind_approach)
        self.get_logger().info(
            "Blind final approach is "
            f"{'enabled' if self.blind_approach_enabled else 'disabled'} "
            f"with distance {self.blind_approach_distance_cm:.2f} cm."
        )

    def position_callback(self, msg):
        if len(msg.data) >= 3:
            self.raw_position = np.array(msg.data, dtype=float)
            if self._should_use_camera_position_fallback():
                self._update_selected_position(self.raw_position)

    def aruco_orientation_callback(self, msg):
        if len(msg.data) >= 3:
            self.raw_orientation = np.array(msg.data, dtype=float)
            if self._should_use_camera_orientation_fallback():
                self._update_selected_orientation(self.raw_orientation)

    def fusion_callback(self, msg):
        if len(msg.data) >= 3:
            self.fused_orientation = np.array(msg.data, dtype=float)
            self.last_fusion_time = time.monotonic()
            if self.use_fused_orientation and not self.use_fused_pose:
                self._update_selected_orientation(self.fused_orientation)

    def fused_pose_callback(self, msg):
        if len(msg.data) >= 6:
            pose = np.array(msg.data[:6], dtype=float)
            self.fused_position = pose[:3]
            self.fused_orientation = pose[3:]
            self.last_fused_pose_time = time.monotonic()
            self.last_fusion_time = self.last_fused_pose_time
            if self.use_fused_pose:
                self._update_selected_pose(self.fused_position, self.fused_orientation)

    def _should_use_camera_position_fallback(self):
        return (
            not self.use_fused_pose
            or self.fused_position is None
            or (time.monotonic() - self.last_fused_pose_time) > self.fusion_timeout_s
        )

    def _should_use_camera_orientation_fallback(self):
        if self.use_fused_pose:
            return self._should_use_camera_position_fallback()

        return (
            not self.use_fused_orientation
            or self.fused_orientation is None
            or (time.monotonic() - self.last_fusion_time) > self.fusion_timeout_s
        )

    def _update_selected_position(self, position):
        self.position = np.array(position, dtype=float)
        self.filtered_position = self._smooth_vector(
            self.filtered_position, self.position, self.position_alpha
        )
        self._update_blind_approach_state()
        self.schedule_update()

    def _update_selected_pose(self, position, orientation):
        self.position = np.array(position, dtype=float)
        self.orientation = np.array(orientation, dtype=float)
        self.filtered_position = self._smooth_vector(
            self.filtered_position, self.position, self.position_alpha
        )
        self.filtered_orientation = self._smooth_vector(
            self.filtered_orientation, self.orientation, self.orientation_alpha
        )
        self._update_blind_approach_state()
        self.schedule_update()

    def _update_selected_orientation(self, orientation):
        self.orientation = np.array(orientation, dtype=float)
        self.filtered_orientation = self._smooth_vector(
            self.filtered_orientation, self.orientation, self.orientation_alpha
        )
        self.schedule_update()

    @staticmethod
    def _smooth_vector(previous, current, alpha):
        alpha = min(max(float(alpha), 0.0), 1.0)
        previous = np.array(previous, dtype=float)
        current = np.array(current, dtype=float)
        return (1.0 - alpha) * previous + alpha * current

    def _smooth_command_targets(self, targets_cm):
        targets_cm = np.array(targets_cm, dtype=float)
        alpha = min(max(float(self.command_alpha), 0.0), 1.0)
        return (1.0 - alpha) * self.last_sent_targets + alpha * targets_cm

    @staticmethod
    def _apply_deadband(previous, current, threshold):
        threshold = max(float(threshold), 0.0)
        previous = np.array(previous, dtype=float)
        current = np.array(current, dtype=float)
        deltas = np.abs(current - previous)
        held = previous.copy()
        update_mask = deltas > threshold
        held[update_mask] = current[update_mask]
        return held

    def _update_control_pose(self):
        if not self.pose_initialized:
            self.control_position = self.filtered_position.copy()
            self.control_orientation = self.filtered_orientation.copy()
            self.pose_initialized = True
            return

        self.control_position = self._apply_deadband(
            self.control_position,
            self.filtered_position,
            self.position_deadband_m,
        )
        self.control_orientation = self._apply_deadband(
            self.control_orientation,
            self.filtered_orientation,
            self.orientation_deadband_deg,
        )

    def _update_blind_approach_state(self):
        self.last_position_update_time = time.monotonic()
        if not self.blind_approach_enabled:
            return

        distance_cm = float(np.linalg.norm(self.filtered_position) * 100.0)
        if distance_cm > self.blind_approach_distance_cm:
            self.blind_approach_armed = False
            self.blind_approach_done = False
            return

        if distance_cm < self.stop_tolerance_cm:
            self.blind_approach_armed = False
            return

        if not self.blind_approach_armed and not self.blind_approach_done:
            self.get_logger().info(
                "Blind final approach armed from last camera pose: "
                f"{distance_cm:.2f} cm <= {self.blind_approach_distance_cm:.2f} cm."
            )
        self.blind_approach_armed = True

    def check_blind_approach(self):
        if (
            not self.blind_approach_enabled
            or not self.blind_approach_armed
            or self.blind_approach_done
            or self.last_position_update_time <= 0.0
        ):
            return

        if (time.monotonic() - self.last_position_update_time) <= self.fusion_timeout_s:
            return

        distance_cm = float(np.linalg.norm(self.filtered_position) * 100.0)
        if distance_cm < self.stop_tolerance_cm:
            self.blind_approach_armed = False
            return

        if distance_cm > self.blind_approach_distance_cm:
            self.blind_approach_armed = False
            return

        text = (
            "Camera pose lost inside blind final approach window. "
            f"Sending one bounded final command from last pose ({distance_cm:.2f} cm)."
        )
        self.get_logger().warn(text)
        self._publish_status(text)
        self.blind_approach_done = True
        self.blind_approach_armed = False
        self.update_actuators(blind_approach=True, force_send=True)

    def _publish_status(self, message):
        try:
            status_msg = String(data=message)
            self.status_publisher.publish(status_msg)
        except Exception:
            self.get_logger().warn(f"Unable to publish status message: {message}")

    def _set_limit_status(self, active, message=""):
        active = bool(active)
        if active:
            self.limit_status_active = True
            self._publish_status(message)
            return
        if self.limit_status_active:
            self.limit_status_active = False
            self._publish_status("")

    def schedule_update(self):
        self.pending_update = True
        self.last_input_time = time.monotonic()

    def flush_pending_update(self):
        if not self.pending_update:
            return

        if (time.monotonic() - self.last_input_time) < self.command_debounce_s:
            return

        self.pending_update = False
        self.update_actuators()

    def update_actuators(self, blind_approach=False, force_send=False):
        if not self.motion_enabled:
            self.get_logger().warn("Motion blocked: runtime state requires recovery.")
            return

        self._update_control_pose()

        current_distance_cm = float(np.linalg.norm(self.control_position) * 100.0)
        if current_distance_cm < self.stop_tolerance_cm:
            if not self.last_stop_state:
                text = (
                    "Marker is within stop tolerance: "
                    f"{current_distance_cm:.2f} cm < {self.stop_tolerance_cm:.2f} cm. "
                    "No new motion command sent."
                )
                self.get_logger().info(text)
                self._publish_status(text)
            self.last_stop_state = True
            return

        self.last_stop_state = False

        if blind_approach and current_distance_cm > self.blind_approach_distance_cm:
            text = (
                "Blind final approach cancelled: last pose is outside configured "
                f"distance ({current_distance_cm:.2f} cm > "
                f"{self.blind_approach_distance_cm:.2f} cm)."
            )
            self.get_logger().warn(text)
            self._publish_status(text)
            return

        trans = (
            -self.control_position if self.invert_marker_pose else self.control_position
        )
        if self.use_orientation_control:
            rotation = (
                -self.control_orientation
                if self.invert_marker_pose
                else self.control_orientation
            )
        else:
            rotation = np.zeros(3)

        longueurs_m = self.platform.solve(trans, rotation)
        longueurs_cm = longueurs_m * 100
        logical_deplacement = longueurs_cm - self.L0

        send_update = any(
            abs(c - l) >= self.length_threshold
            for c, l in zip(logical_deplacement, self.last_deplacement)
        )

        if force_send:
            send_update = True

        if send_update:
            if not self.calibration.within_logical_range(logical_deplacement):
                violation = self.calibration.logical_violation_message(logical_deplacement)
                text = (
                    "Consigne logique hors débattement commun, envoi UART annulé : "
                    f"{violation}"
                )
                self.get_logger().warn(text)
                self._set_limit_status(True, text)
                return

            physical_targets = self.calibration.logical_to_relative_targets(
                logical_deplacement, self.home_vector
            )
            if not blind_approach:
                physical_targets = self._smooth_command_targets(physical_targets)

            if not force_send and np.all(
                np.abs(physical_targets - self.last_sent_targets) < self.command_deadband_cm
            ):
                self.last_deplacement = logical_deplacement.copy()
                return

            out_msg = Float32MultiArray()
            out_msg.data = physical_targets.tolist()
            self.publisher_.publish(out_msg)

            if self.calibration.within_relative_limits(physical_targets, self.home_vector):
                consigne = ",".join([f"{d:.2f}" for d in physical_targets])
                self.ser.write((consigne + "\n").encode())
                self.runtime_state.record_command(physical_targets)
                self.last_sent_targets = physical_targets.copy()
                if blind_approach:
                    self.get_logger().warn(
                        f"Blind final approach command sent: {consigne}"
                    )
                else:
                    self.get_logger().debug(f"Consigne envoyée : {consigne}")
                self._set_limit_status(False)
            else:
                violation = self.calibration.relative_violation_message(
                    physical_targets, self.home_vector
                )
                text = (
                    "Consigne hors limites moteur, envoi UART annulé : "
                    f"{violation}"
                )
                self.get_logger().warn(text)
                self._set_limit_status(True, text)

            self.last_deplacement = logical_deplacement.copy()

            if not self.continuous:
                self.get_logger().info("Consigne envoyée une fois, arrêt du callback.")
                self.destroy_subscription(self.pos_subscription)
                self.destroy_subscription(self.ori_subscription)
                self.destroy_subscription(self.fusion_subscription)
                self.destroy_subscription(self.fused_pose_subscription)

    def _record_feedback_state_if_due(self, feedback):
        now = time.monotonic()
        if (
            now - self.last_feedback_state_save_time
        ) < self.FEEDBACK_STATE_SAVE_INTERVAL_S:
            return

        self.runtime_state.record_feedback(feedback)
        self.last_feedback_state_save_time = now

    def read_feedback(self, max_lines=SERIAL_READ_MAX_LINES):
        """Lecture du feedback depuis Arduino et publication ROS2"""
        lines_read = 0
        while self.ser.in_waiting > 0 and (
            max_lines is None or lines_read < max_lines
        ):
            line = self.ser.readline().decode(errors="ignore").strip()
            lines_read += 1

            if not line or "," not in line:
                continue

            try:
                feedback = [float(x) for x in line.split(",") if x.strip() != ""]

                if len(feedback) != 6:
                    self.get_logger().warn(f"Ligne série incomplète ignorée : {line}")
                    continue

                msg = Float32MultiArray()
                msg.data = feedback
                self.publisher_feedback.publish(msg)
                self.latest_feedback = feedback
                self._record_feedback_state_if_due(feedback)
                self.get_logger().debug(f"Feedback reçu : {feedback}")

            except ValueError:
                self.get_logger().warn(f"Ligne série invalide : {line}")

    def _park_platform_before_exit(self):
        if not self.motion_enabled:
            self.runtime_state.mark_unclean_shutdown(
                "Shutdown while position was unknown. Recovery required."
            )
            return

        self.runtime_state.mark_shutdown_in_progress()
        home_targets = np.zeros(ActuatorCalibration.MOTOR_COUNT, dtype=float)
        command = ",".join(f"{value:.2f}" for value in home_targets) + "\n"
        deadline = time.monotonic() + self.SHUTDOWN_TIMEOUT_S
        stable_reads = 0

        while time.monotonic() < deadline:
            self.ser.write(command.encode())
            time.sleep(0.05)
            self.read_feedback(max_lines=None)

            if self.latest_feedback is None:
                continue

            if all(abs(value) <= self.HOME_TOLERANCE_CM for value in self.latest_feedback):
                stable_reads += 1
                if stable_reads >= self.HOME_STABLE_READS:
                    self.runtime_state.record_command(home_targets)
                    self.runtime_state.mark_clean_shutdown(self.latest_feedback)
                    self.get_logger().info(
                        "Clean shutdown completed: platform parked at home."
                    )
                    return
            else:
                stable_reads = 0

        self.runtime_state.mark_unclean_shutdown(
            "Clean shutdown failed: home position was not confirmed."
        )
        self.get_logger().error(
            "Unable to confirm home position before shutdown. Recovery will be required "
            "on next startup."
        )


def _safe_shutdown_node(node):
    try:
        if hasattr(node, "_park_platform_before_exit"):
            node._park_platform_before_exit()
    except Exception as exc:
        node.runtime_state.mark_unclean_shutdown(
            f"Shutdown error: {type(exc).__name__}"
        )
        node.get_logger().warn(
            f"Erreur pendant la fermeture sécurisée (non critique) : {exc}"
        )

    try:
        if hasattr(node, "ser") and node.ser is not None and node.ser.is_open:
            node.ser.close()
    except Exception as exc:
        node.get_logger().warn(f"SERIAL close failed: {exc}")

    try:
        if hasattr(node, "destroy_node"):
            node.destroy_node()
    except Exception as exc:
        node.get_logger().warn(f"Destroy node failed: {exc}")

    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception as exc:
        node.get_logger().warn(f"RCL shutdown failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = StewartNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _safe_shutdown_node(node)


if __name__ == "__main__":
    main()
