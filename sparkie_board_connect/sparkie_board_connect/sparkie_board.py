#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sparkie_board_connect.sparkie_reader import SparkieDataReader
from std_msgs.msg import Int8, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField, Range, Temperature, RelativeHumidity, BatteryState
import serial
import threading
import argparse
import sys
import time
from math import sin, cos, pi


class SparkieBoardNode(Node):
    """
    ROS2 node for serial communication with an external board/device.
    Publishes data received from the serial port and allows sending commands.
    """



    us_map = {
        0: 'left',
        1: 'front',
        2: 'right'
    }
    BASE_FOOTPRINT = 'base_footprint'
    BASE_LINK = 'base_link'
    BOARD_LINK = 'board_link'
    
    def __init__(self, serial_port='/dev/ttyACM0', baud_rate=115200):
        super().__init__('board', namespace='sparkie')

        # Ultrasonic sensors filters
        self.us_prev_values = [None, None, None]
        self.us_alpha = 0.6  # Smoothing factor for low-pass filter

        # Node parameters
        self.declare_parameter('serial_port', serial_port)
        self.declare_parameter('baud_rate', baud_rate)
        self.declare_parameter('timeout', 1.0)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        
        # Publishers initialization
        self.imu_publisher = self.create_publisher(Imu, 'board/imu', 10)
        self.mag_publisher = self.create_publisher(MagneticField, 'board/mag', 10)
        self.us_publisher = [
            self.create_publisher(Range, f'board/us/{self.us_map[i]}', 10) for i in range(3)
        ]
        self.temp_publisher = self.create_publisher(Temperature, 'board/temperature', 10)
        self.humidity_publisher = self.create_publisher(RelativeHumidity, 'board/humidity', 10)

        self.left_wheel_pub = self.create_publisher(Float32, 'board/wheels/left', 10)
        self.right_wheel_pub = self.create_publisher(Float32, 'board/wheels/right', 10)
        self.odom_pub = self.create_publisher(Odometry, 'board/wheels/odom', 10)

        self.battery_pub = self.create_publisher(BatteryState, 'board/battery', 10)

        # Subcriptions initialization
        self.ledstrip_subscription = self.create_subscription(Int8, 'board/ledstrip', self.ledstrip_callback, 10)
        self.vel_subscription = self.create_subscription(Twist, 'board/cmd_vel', self.vel_callback, 10)
        self.servo_subscription = self.create_subscription(Float32, 'board/head/tilt', self.servo_callback, 10)

        # Initalize SparkieDataReader
        self.sparkie_reader = SparkieDataReader(
            port=self.serial_port,
            baudrate=self.baud_rate,
            logger=self.get_logger()  # Pass the logger to SparkieDataReader
        )

        # Regster callbacks
        self.sparkie_reader.register_message_handler('imuData', self.imu_callback)
        self.sparkie_reader.register_message_handler('ultrasonicData', self.us_callback)
        self.sparkie_reader.register_message_handler('ahtData', self.aht_callback)
        self.sparkie_reader.register_message_handler('motorStatus', self.motor_status_callback)
        self.sparkie_reader.register_message_handler('wheelOdometryData', self.odometry_callback)
        self.sparkie_reader.register_message_handler('batteryData', self.battery_callback)


        self.sparkie_reader.start_reading()

        # Information logs
        self.get_logger().info(f'SerialConnector node started')
        self.get_logger().info(f'Serial port: {self.serial_port}')
        self.get_logger().info(f'Baud rate: {self.baud_rate}')
        self.get_logger().info(f'Timeout: {self.timeout}s')
    
    def imu_callback(self, msg):
        """
        Callback to handle IMU data received from the SparkieDataReader.
        Publishes the IMU data to the 'imu' topic.
        """
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.BOARD_LINK

        imu_msg.orientation_covariance[0] = -1.0  # Use -1 to indicate orientation is not available

        imu_msg.linear_acceleration.x = msg.imuData.a_x
        imu_msg.linear_acceleration.y = msg.imuData.a_y
        imu_msg.linear_acceleration.z = msg.imuData.a_z

        imu_msg.angular_velocity.x = msg.imuData.g_x
        imu_msg.angular_velocity.y = msg.imuData.g_y
        imu_msg.angular_velocity.z = msg.imuData.g_z

        self.imu_publisher.publish(imu_msg)

        mag_msg = MagneticField()
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        mag_msg.header.frame_id = self.BOARD_LINK

        mag_msg.magnetic_field.x = msg.imuData.m_x
        mag_msg.magnetic_field.y = msg.imuData.m_y
        mag_msg.magnetic_field.z = msg.imuData.m_z

        self.mag_publisher.publish(mag_msg)

    def us_callback(self, msg):
        """
        Callback to handle Ultrasonic data received from the SparkieDataReader.
        Publishes the Ultrasonic data to the respective topics.
        """
        us_msg = Range()
        us_msg.header.stamp = self.get_clock().now().to_msg()
        us_msg.header.frame_id = 'us_' + self.us_map[msg.topicId]
        us_msg.radiation_type = Range.ULTRASOUND

        us_msg.field_of_view = 0.26
        us_msg.min_range = 0.02
        us_msg.max_range = 4.0
        
        us_range = msg.ultrasonicData.distance_cm / 100.0  # Convert cm to meters

        if self.us_prev_values[msg.topicId] is None:
            self.us_prev_values[msg.topicId] = us_range
        else:
            # Apply low-pass filter
            us_range = self.us_alpha * self.us_prev_values[msg.topicId] + (1 - self.us_alpha) * us_range
            self.us_prev_values[msg.topicId] = us_range

        us_msg.range = us_range

        self.us_publisher[msg.topicId].publish(us_msg)

    def aht_callback(self, msg):
        """
        Callback to handle AHT data received from the SparkieDataReader.
        Publishes the Temperature and Humidity data to the respective topics.
        """
        temp_msg = Temperature()
        temp_msg.header.stamp = self.get_clock().now().to_msg()
        temp_msg.header.frame_id = self.BOARD_LINK
        temp_msg.temperature = msg.ahtData.temp

        self.temp_publisher.publish(temp_msg)

        humidity_msg = RelativeHumidity()
        humidity_msg.header.stamp = self.get_clock().now().to_msg()
        humidity_msg.header.frame_id = self.BOARD_LINK
        humidity_msg.relative_humidity = msg.ahtData.hum

        self.humidity_publisher.publish(humidity_msg)

    def motor_status_callback(self, msg):
        """
        Callback to handle Motor Status data received from the SparkieDataReader.
        Publishes the left and right wheel speeds to their respective topics.
        """
        
        if(msg.topicId == 0):
            left_wheel_msg = Float32()
            left_wheel_msg.data = msg.motorStatus.current_speed
            self.left_wheel_pub.publish(left_wheel_msg)
        elif(msg.topicId == 1):
            right_wheel_msg = Float32()
            right_wheel_msg.data = msg.motorStatus.current_speed
            self.right_wheel_pub.publish(right_wheel_msg)

    def odometry_callback(self, msg):
        """
        Callback to handle Wheel Odometry data received from the SparkieDataReader.
        Publishes the odometry data to the 'board/wheels/odom' topic.
        """
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id ="odom"
        odom_msg.child_frame_id = "base_footprint"

        odom_msg.pose.pose.position.x = msg.wheelOdometryData.x
        odom_msg.pose.pose.position.y = msg.wheelOdometryData.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = sin((msg.wheelOdometryData.theta) / 2)  # Assuming theta is in radians
        odom_msg.pose.pose.orientation.w = cos((msg.wheelOdometryData.theta) / 2)  # Assuming theta is in radians
        odom_msg.twist.twist.linear.x = msg.wheelOdometryData.linear_velocity
        odom_msg.twist.twist.angular.z = msg.wheelOdometryData.angular_velocity

         # Covarianza per la posa (6x6 = 36 elementi)
        odom_msg.pose.covariance = [0.0] * 36
        # Imposta valori di covarianza ragionevoli (da regolare in base al tuo robot)
        odom_msg.pose.covariance[0] = 0.01   # var(x)
        odom_msg.pose.covariance[7] = 0.01   # var(y)
        odom_msg.pose.covariance[14] = 0.01  # var(z) 
        odom_msg.pose.covariance[21] = 0.01  # var(roll)
        odom_msg.pose.covariance[28] = 0.01  # var(pitch)
        odom_msg.pose.covariance[35] = 0.01  # var(yaw)
        
        # Covarianza per la velocità (6x6 = 36 elementi)
        odom_msg.twist.covariance = [0.0] * 36
        odom_msg.twist.covariance[0] = 0.01   # var(vx)
        odom_msg.twist.covariance[7] = 0.01   # var(vy)
        odom_msg.twist.covariance[14] = 0.01  # var(vz)
        odom_msg.twist.covariance[21] = 0.01  # var(vroll)
        odom_msg.twist.covariance[28] = 0.01  # var(vpitch)
        odom_msg.twist.covariance[35] = 0.01  # var(vyaw)

        self.odom_pub.publish(odom_msg)

    def battery_callback(self, msg):
        """
        Callback to handle Battery data received from the SparkieDataReader.
        Publishes the battery data to the 'board/battery' topic.
        """
        battery_msg = BatteryState()
        battery_msg.header.stamp = self.get_clock().now().to_msg()
        battery_msg.header.frame_id = self.BOARD_LINK

        battery_msg.voltage = msg.batteryData.voltage
        # battery_msg.current = None
        # battery_msg.percentage = msg.batteryData.voltage / (12.6 * 100.0)

        self.battery_pub.publish(battery_msg)

    def ledstrip_callback(self, msg):
        """
        Callback to handle LED strip commands.
        Sends the command to the SparkieDataReader.
        """
        self.sparkie_reader.send_ledstrip_command(msg.data)

    def vel_callback(self, msg):
        """
        Callback to handle velocity commands.
        Sends the command to the SparkieDataReader.
        """
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        self.sparkie_reader.send_speed_command(linear_x, angular_z)

    def servo_callback(self, msg):
        """
        Callback to handle servo angle commands.
        Sends the command to the SparkieDataReader.
        """
        angle = msg.data
        self.sparkie_reader.send_servo_command(angle)

    def destroy_node(self):
        """Cleanup when the node is destroyed"""
        self.sparkie_reader.stop_reading()
        time.sleep(0.1)
        super().destroy_node()


def main(args=None):
    # Command line arguments parsing
    parser = argparse.ArgumentParser(description='ROS2 node for serial connection')
    parser.add_argument('--serial-port', default='/dev/ttyACM0',
                       help='Serial port to use (default: /dev/ttyACM0)')
    parser.add_argument('--baud-rate', type=int, default=115200,
                       help='Serial connection baud rate (default: 115200)')
    
    # Parse only known arguments to avoid conflicts with ROS2
    known_args, remaining_args = parser.parse_known_args()
    
    # Initialize ROS2
    rclpy.init(args=remaining_args)
    
    try:
        # Create the node with specified parameters
        node = SparkieBoardNode(
            serial_port=known_args.serial_port,
            baud_rate=known_args.baud_rate
        )
        
        # Spin the node
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in node execution: {e}')
    finally:
        # Cleanup
        if 'node' in locals():
            node.destroy_node()


if __name__ == '__main__':
    main()
