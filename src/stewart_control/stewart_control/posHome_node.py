#!/home/rem/rtimulib-env/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time
from stewart_control.config_loader import get_config
from stewart_control.actuator_calibration import ActuatorCalibration
from stewart_control.runtime_state import RuntimeStateManager


class PosHomeNode(Node):
    HOME_TOLERANCE_CM = 0.3
    HOME_STABLE_READS = 5
    HOMING_TIMEOUT_S = 30.0
    COMMAND_INTERVAL_S = 0.1

    def __init__(self):
        super().__init__("pos_home_node")

        cfg = get_config()
        ser_cfg = cfg["serial"]
        act = cfg["actuators"]
        self.calibration = ActuatorCalibration.from_config(act)
        self.home_vector = act.get("home_vector", [0.0] * ActuatorCalibration.MOTOR_COUNT)
        self.runtime_state = RuntimeStateManager()

        # Publisher pour le feedback
        self.publisher_feedback = self.create_publisher(
            Float32MultiArray, "feedback_motors", 10
        )

        # Configuration UART
        self.ser = serial.Serial(
            ser_cfg["port"], ser_cfg["baudrate"], timeout=ser_cfg["timeout"]
        )
        time.sleep(ser_cfg["startup_delay"])
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.get_logger().info("Port UART ouvert et prêt.")

        # Calculate home target positions
        self.home_targets = self.calibration.logical_to_relative_targets(
            self.home_vector, self.home_vector
        )

        # Homing state
        self.homing_active = False
        self.homing_start_time = None
        self.stable_reads = 0
        self.last_feedback = None

        # Timer for homing process
        self.create_timer(act["feedback_timer_period"], self.homing_timer_callback)

        # Start homing if recovery is required
        if self.runtime_state.requires_recovery():
            self.start_homing()
        else:
            self.get_logger().info("Platform already homed. Monitoring position.")

    def start_homing(self):
        """Initiate the homing sequence."""
        if self.homing_active:
            return

        self.homing_active = True
        self.homing_start_time = time.monotonic()
        self.stable_reads = 0
        self.last_feedback = None

        self.runtime_state.mark_shutdown_in_progress()
        self.get_logger().info("Starting homing sequence...")

    def homing_timer_callback(self):
        """Main homing control loop."""
        # Always read feedback
        self.read_feedback()

        if self.homing_active:
            self.perform_homing_step()
        else:
            # Monitor position when not actively homing
            self.monitor_home_position()

    def perform_homing_step(self):
        """Execute one step of the homing process."""
        if not self.homing_active:
            return

        # Check timeout
        if time.monotonic() - self.homing_start_time > self.HOMING_TIMEOUT_S:
            self.abort_homing("Homing timeout exceeded")
            return

        # Send home command
        self.send_home_command()

        # Check if home position is reached
        if self.is_at_home_position():
            self.stable_reads += 1
            if self.stable_reads >= self.HOME_STABLE_READS:
                self.complete_homing()
        else:
            self.stable_reads = 0

    def send_home_command(self):
        """Send the home position command to Arduino."""
        message = ",".join([f"{v:.2f}" for v in self.home_targets]) + "\n"
        try:
            self.ser.write(message.encode())
            self.get_logger().debug(f"Home command sent: {message.strip()}")
        except Exception as e:
            self.get_logger().error(f"UART error during homing: {e}")
            self.abort_homing(f"Communication error: {e}")

    def read_feedback(self):
        """Read feedback from Arduino and publish it."""
        while self.ser.in_waiting > 0:
            line = self.ser.readline().decode(errors="ignore").strip()

            if not line or "," not in line:
                continue

            try:
                feedback = [float(x) for x in line.split(",") if x.strip() != ""]

                if len(feedback) != 6:
                    self.get_logger().warn(f"Incomplete serial line ignored: {line}")
                    continue

                # Publish feedback
                msg = Float32MultiArray()
                msg.data = feedback
                self.publisher_feedback.publish(msg)

                # Store for homing logic
                self.last_feedback = feedback
                self.get_logger().debug(f"Feedback received: {feedback}")

            except ValueError:
                self.get_logger().warn(f"Invalid serial line: {line}")

    def is_at_home_position(self):
        """Check if the platform is at the home position."""
        if self.last_feedback is None:
            return False

        # Check if all actuators are within tolerance of home position (0.0)
        return all(abs(value) <= self.HOME_TOLERANCE_CM for value in self.last_feedback)

    def complete_homing(self):
        """Mark homing as complete and update runtime state."""
        self.homing_active = False
        self.runtime_state.record_command(self.home_targets)
        self.runtime_state.mark_clean_shutdown(self.last_feedback)
        self.get_logger().info("Homing completed successfully. Platform is at home position.")

    def abort_homing(self, reason):
        """Abort homing due to error."""
        self.homing_active = False
        self.runtime_state.mark_unclean_shutdown(f"Homing failed: {reason}")
        self.get_logger().error(f"Homing aborted: {reason}")

    def monitor_home_position(self):
        """Monitor position when not actively homing."""
        if self.last_feedback is not None:
            if not self.is_at_home_position():
                self.get_logger().warn("Platform has moved from home position!")
                # Could trigger re-homing or alert here


def _safe_shutdown_node(node):
    try:
        if hasattr(node, 'homing_active') and node.homing_active:
            node.abort_homing("Shutdown during homing")
        if hasattr(node, '_park_platform_before_exit'):
            node._park_platform_before_exit()
    except Exception as exc:
        node.runtime_state.mark_unclean_shutdown(
            f"Shutdown error: {type(exc).__name__}"
        )
        node.get_logger().warn(
            f"Erreur pendant la fermeture sécurisée (non critique) : {exc}"
        )

    try:
        if hasattr(node, 'ser') and node.ser is not None and node.ser.is_open:
            node.ser.close()
    except Exception as exc:
        node.get_logger().warn(f"SERIAL close failed: {exc}")

    try:
        if hasattr(node, 'destroy_node'):
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
    node = PosHomeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _safe_shutdown_node(node)


if __name__ == "__main__":
    main()
