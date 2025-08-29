#!/home/rem/rtimulib-env/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import RTIMU
import math

# ---------------------------
# Utils quaternions
# ---------------------------
def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, y, z)

def quaternion_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)

def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    )

def quaternion_to_euler(q):
    w, x, y, z = q
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return (roll, pitch, yaw)  # en radians

# Normalisation angle [-180,180]
def normalize_angle(angle):
    while angle > 180: angle -= 360
    while angle < -180: angle += 360
    return angle

# ---------------------------
# Node ROS2
# ---------------------------
class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_error_node')

        self.pub_imu = self.create_publisher(Float32MultiArray, 'imu_error', 10)

        # Initialisation IMU
        self.settings1 = RTIMU.Settings("RTIMULib_imu1.ini")
        self.imu1 = RTIMU.RTIMU(self.settings1)
        self.settings2 = RTIMU.Settings("RTIMULib_imu2.ini")
        self.imu2 = RTIMU.RTIMU(self.settings2)

        if not self.imu1.IMUInit():
            self.get_logger().error("IMU 1 Init Failed")
            exit(1)
        if not self.imu2.IMUInit():
            self.get_logger().error("IMU 2 Init Failed")
            exit(1)

        for imu in (self.imu1, self.imu2):
            imu.setSlerpPower(0.9)
            imu.setGyroEnable(True)
            imu.setAccelEnable(True)
            imu.setCompassEnable(True)

        self.timer = self.create_timer(0.01, self.publish_imu_error)  # 10 ms

    def publish_imu_error(self):
        roll1 = pitch1 = yaw1 = 0.0
        roll2 = pitch2 = yaw2 = 0.0

        if self.imu1.IMURead():
            data1 = self.imu1.getIMUData()
            roll1, pitch1, yaw1 = data1["fusionPose"]

        if self.imu2.IMURead():
            data2 = self.imu2.getIMUData()
            roll2, pitch2, yaw2 = data2["fusionPose"]

        # Convertir en quaternions
        q1 = euler_to_quaternion(roll1, pitch1, yaw1)
        q2 = euler_to_quaternion(roll2, pitch2, yaw2)

        # Erreur = cible * conj(actuel)
        q_err = quaternion_multiply(q2, quaternion_conjugate(q1))
        d_roll, d_pitch, d_yaw = quaternion_to_euler(q_err)

        # Conversion en degrés + normalisation
        d_roll  = normalize_angle(math.degrees(d_roll))
        d_pitch = normalize_angle(math.degrees(d_pitch))
        d_yaw   = normalize_angle(math.degrees(d_yaw))

        # Publier uniquement l'erreur
        msg = Float32MultiArray()
        msg.data = [round(d_roll, 2), round(d_pitch, 2), round(d_yaw, 2)]
        self.pub_imu.publish(msg)

        # Log
        self.get_logger().info(f"Erreur orientation: Roll={d_roll:.2f}, Pitch={d_pitch:.2f}, Yaw={d_yaw:.2f}")

# ---------------------------
def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
