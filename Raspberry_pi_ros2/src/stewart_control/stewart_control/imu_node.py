#!/home/rem/rtimulib-env/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import math
import smbus
from imusensor.MPU9250 import MPU9250

# ==============================
# Fonctions utils
# ==============================
def compute_rpy(imu):
    Ax, Ay, Az = imu.AccelVals
    Mx, My, Mz = imu.MagVals

    roll  = math.atan2(Ay, Az)
    pitch = math.atan2(-Ax, math.sqrt(Ay**2 + Az**2))

    Xh = Mx * math.cos(pitch) + Mz * math.sin(pitch)
    Yh = (Mx * math.sin(roll) * math.sin(pitch) +
          My * math.cos(roll) -
          Mz * math.sin(roll) * math.cos(pitch))

    yaw = math.atan2(-Yh, Xh)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

def normalize_angle(angle):
    while angle > 180: angle -= 360
    while angle < -180: angle += 360
    return angle

def wrap_deg(a: float) -> float:
    # identique à ton fusion
    return (float(a) + 180.0) % 360.0 - 180.0

# ---------- QUATERNIONS ----------
def euler_zyx_to_quat(roll_deg, pitch_deg, yaw_deg):
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    yy = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, yy, z)

def quat_conj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)

def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return (w, x, y, z)

def quat_to_euler_zyx(q):
    w, x, y, z = q

    sinr_cosp = 2.0 * (w*x + y*z)
    cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w*y - z*x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi/2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (wrap_deg(math.degrees(roll)),
            wrap_deg(math.degrees(pitch)),
            wrap_deg(math.degrees(yaw)))

# ==============================
# Node ROS2
# ==============================
class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_error_node')

        self.pub_imu = self.create_publisher(Float32MultiArray, 'imu_error', 10)

        bus = smbus.SMBus(1)
        self.imu1 = MPU9250.MPU9250(bus, 0x69)
        self.imu2 = MPU9250.MPU9250(bus, 0x68)
        self.imu1.begin()
        self.imu2.begin()

        self.imu1.loadCalibDataFromFile("/home/rem/ros2_ws/src/stewart_control/stewart_control/calib1.json")
        self.imu2.loadCalibDataFromFile("/home/rem/ros2_ws/src/stewart_control/stewart_control/calib2.json")

        self.get_logger().info("IMU1 & IMU2 initialisées et calibrées ✅")

        # ✅ yaw offset AUTO (comme ton idée “yaw moy”)
        self.AUTO_CALIB_SECONDS = 3.0      # tu peux mettre 2.0 / 5.0
        self._t0 = time.time()
        self._yaw_sum = 0.0
        self._yaw_n = 0
        self.yaw_offset_deg = 0.0
        self.yaw_offset_ready = False

        self.timer = self.create_timer(0.1, self.publish_imu_error)

    def publish_imu_error(self):
        self.imu1.readSensor()
        self.imu2.readSensor()

        r1, p1, y1 = compute_rpy(self.imu1)
        r2, p2, y2 = compute_rpy(self.imu2)

        # relatif par quaternions
        q1 = euler_zyx_to_quat(r1, p1, y1)
        q2 = euler_zyx_to_quat(r2, p2, y2)
        q_rel = quat_mul(quat_conj(q2), q1)  # IMU1 wrt IMU2
        roll_rel, pitch_rel, yaw_rel_raw = quat_to_euler_zyx(q_rel)

        # ✅ AUTO CALIB yaw_offset sur les premières secondes (plateforme immobile)
        if not self.yaw_offset_ready:
            self._yaw_sum += float(yaw_rel_raw)
            self._yaw_n += 1
            if (time.time() - self._t0) >= self.AUTO_CALIB_SECONDS and self._yaw_n > 0:
                self.yaw_offset_deg = self._yaw_sum / self._yaw_n
                self.yaw_offset_ready = True
                self.get_logger().info(
                    f"Yaw offset auto-calibré = {self.yaw_offset_deg:.2f}° "
                    f"(sur {self._yaw_n} échantillons)"
                )

        # ✅ correction yaw (comme fusion)
        yaw_rel = wrap_deg(yaw_rel_raw - self.yaw_offset_deg)

        msg = Float32MultiArray()
        msg.data = [round(roll_rel, 2), round(pitch_rel, 2), round(yaw_rel, 2)]
        self.pub_imu.publish(msg)

        self.get_logger().info(
            f"Erreur orientation → Roll: {roll_rel:.2f}, Pitch: {pitch_rel:.2f}, "
            f"Yaw(raw): {yaw_rel_raw:.2f}, Yaw(corr): {yaw_rel:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
