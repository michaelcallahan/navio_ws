import rclpy
from rclpy.node import Node
from navio.imu import IMU
import numpy as np

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        self.imu = IMU()
        self.imu.initialize()
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self.num_samples = 100  # Number of samples to average for calibration
        self.samples_collected = 0
        self.accel_data_list = []
        self.gyro_data_list = []
        self.timer = self.create_timer(0.1, self.calibration_callback)  # Collect data every 0.1 seconds

    def calibration_callback(self):
        if self.samples_collected < self.num_samples:
            # Gather IMU data for calibration purposes
            accel_data = self.imu.get_accel_data()
            gyro_data = self.imu.get_gyro_data()
            self.accel_data_list.append(np.array(accel_data))
            self.gyro_data_list.append(np.array(gyro_data))
            self.samples_collected += 1
            self.get_logger().info(f'Collecting sample {self.samples_collected}/{self.num_samples}')
        else:
            # Perform averaging to calculate biases
            self.accel_bias = np.mean(self.accel_data_list, axis=0)
            self.gyro_bias = np.mean(self.gyro_data_list, axis=0)
            
            # Log the calculated biases
            self.get_logger().info(f'Calibration complete! Accel Bias: {self.accel_bias}, Gyro Bias: {self.gyro_bias}')
            
            # Cancel the timer once calibration is complete
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()