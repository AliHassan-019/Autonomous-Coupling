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
    COMMAND_DEBOUNCE_S = 0.02
    HOME_TOLERANCE_CM = 0.1
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
        self.pos_sub = self.create_subscription(
            Float32MultiArray, "manual_position", self.pos_callback, 1
        )
        self.ori_sub = self.create_subscription(
            Float32MultiArray, "manual_orientation", self.ori_callback, 1
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
        self.continuous = False
        self.last_deplacement = [0.0] * 6
        self.length_threshold = act["length_threshold"]
        self.calibration = ActuatorCalibration.from_config(act)
        self.home_vector = np.array(
            act.get("home_vector", [0.0] * ActuatorCalibration.MOTOR_COUNT),
            dtype=float,
        )
        self.pending_update = False
        self.last_input_time = 0.0
        self.limit_status_active = False
        self.runtime_state = RuntimeStateManager()
        self.motion_enabled = self.runtime_state.begin_motion_session()
        self.latest_feedback = None
        self.last_feedback_state_save_time = 0.0

        if self.motion_enabled:
            self.get_logger().info("Trusted startup state detected. Motion enabled.")
        else:
            self.get_logger().error(
                "Position unknown after previous shutdown. Manual recovery is required "
                "before commands are accepted."
            )

        # Timer pour lecture série
        self.create_timer(act["feedback_timer_period"], self.read_feedback)
        self.create_timer(0.01, self.flush_pending_update)

    def pos_callback(self, msg):
        if len(msg.data) >= 3:
            # GUI manual translations are entered in cm; IK expects meters.
            self.position = np.array(msg.data, dtype=float) * self.CM_TO_M
            self.schedule_update()

    def ori_callback(self, msg):
        if len(msg.data) >= 3:
            self.orientation = np.array(msg.data, dtype=float)
            self.schedule_update()

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

        if (time.monotonic() - self.last_input_time) < self.COMMAND_DEBOUNCE_S:
            return

        self.pending_update = False
        self.update_actuators()

    def update_actuators(self):
        if not self.motion_enabled:
            self.get_logger().warn("Motion blocked: runtime state requires recovery.")
            return

        rotation = self.orientation
        trans = self.position

        longueurs_m = self.platform.solve(trans, rotation)
        longueurs_cm = longueurs_m * 100
        logical_deplacement = longueurs_cm - self.L0

        send_update = any(
            abs(c - l) >= self.length_threshold
            for c, l in zip(logical_deplacement, self.last_deplacement)
        )

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

            out_msg = Float32MultiArray()
            out_msg.data = physical_targets.tolist()
            self.publisher_.publish(out_msg)

            if self.calibration.within_relative_limits(physical_targets, self.home_vector):
                consigne = ",".join([f"{d:.2f}" for d in physical_targets])
                self.ser.write((consigne + "\n").encode())
                self.runtime_state.record_command(physical_targets)
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
