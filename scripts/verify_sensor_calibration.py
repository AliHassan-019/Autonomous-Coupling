#!/usr/bin/env python3
"""
Sensor Calibration Verification Script

This script helps verify that camera and IMU reference orientations
are calibrated to the same physical reference frame.

CURRENT CALIBRATION STATUS:
- IMU reference: [-177.77, -44.60, -145.30] (calibrated for level platform)
- Camera reference: [6.14, -9.76, 21.85] (calibrated for level platform)

USAGE:
1. Place platform in level position (no tilt)
2. Run: python3 verify_sensor_calibration.py
3. Check that both sensors report ~0° for roll/pitch
4. Adjust reference_orientation_deg values in stewart_params.yaml if needed

EXPECTED OUTPUT:
- IMU and Camera should both report near 0° when platform is level
- If not, adjust reference orientations to match current readings
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import sys

class SensorVerifier(Node):
    def __init__(self):
        super().__init__('sensor_verifier')

        # Subscribe to processed sensor outputs
        self.sub_imu = self.create_subscription(
            Float32MultiArray, 'imu_error', self.imu_callback, 10
        )
        self.sub_cam = self.create_subscription(
            Float32MultiArray, 'aruco_orientation', self.cam_callback, 10
        )
        self.sub_fused = self.create_subscription(
            Float32MultiArray, 'F_orientation', self.fused_callback, 10
        )

        self.imu_data = None
        self.cam_data = None
        self.fused_data = None

        self.get_logger().info("Sensor Calibration Verifier Started")
        self.get_logger().info("Place platform level and wait for readings...")

    def imu_callback(self, msg):
        self.imu_data = msg.data

    def cam_callback(self, msg):
        self.cam_data = msg.data

    def fused_callback(self, msg):
        self.fused_data = msg.data

    def print_readings(self):
        print("\n" + "="*60)
        print("SENSOR CALIBRATION VERIFICATION")
        print("="*60)

        if self.imu_data:
            print(f"IMU (processed):     Roll={self.imu_data[0]:+6.2f}° "
                  f"Pitch={self.imu_data[1]:+6.2f}° Yaw={self.imu_data[2]:+6.2f}°")
        else:
            print("IMU: No data received")

        if self.cam_data:
            print(f"Camera (processed):  Roll={self.cam_data[0]:+6.2f}° "
                  f"Pitch={self.cam_data[1]:+6.2f}° Yaw={self.cam_data[2]:+6.2f}°")
        else:
            print("Camera: No data received")

        if self.fused_data:
            print(f"Fused (recommended): Roll={self.fused_data[0]:+6.2f}° "
                  f"Pitch={self.fused_data[1]:+6.2f}° Yaw={self.fused_data[2]:+6.2f}°")
        else:
            print("Fused: No data received")

        print("\nINSTRUCTIONS:")
        print("- Platform should be LEVEL (no tilt)")
        print("- All sensors should report ~0° for Roll/Pitch")
        print("- If not, adjust reference_orientation_deg in stewart_params.yaml")
        print("- Camera ref: [6.14, -9.76, 21.85] (currently calibrated)")
        print("- IMU ref: [-177.77, -44.60, -145.30] (currently calibrated)")
        print("- To recalibrate: set reference to current readings when level")

def main(args=None):
    rclpy.init(args=args)
    node = SensorVerifier()

    try:
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < 10:  # Run for 10 seconds
            rclpy.spin_once(node, timeout_sec=0.1)
            node.print_readings()
            time.sleep(2)  # Print every 2 seconds

        print("\n" + "="*60)
        print("FINAL READINGS (after 10 seconds):")
        node.print_readings()

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()