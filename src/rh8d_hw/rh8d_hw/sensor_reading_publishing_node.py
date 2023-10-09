#!usr/bin/env python3
import rclpy
from rclpy.node import Node

from rh8d_msgs.msg import AllSensors, SensorUserCommand, LoneSensor

import serial
import sys
import numpy as np
import time


class RH8DSensorNode(Node):
    def __init__(self):
        self.name = 'rh8d_sensor'
        super().__init__(self.name + '_node')

        # Declare and get all Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('port', "/dev/ttyUSB1"),
                ('hand_polarity', 'L_'),
                ('sensor_number', 5),
            ])
        try:
            self.port = self.get_parameter('port').value
            self.get_logger().info("Port name is: %s" % self.port)
        except KeyError:
            self.get_logger().info("Could not find port parameter, setting to default port (/dev/ttyUSB1)")
            pass

        try:
            self.polarity = self.get_parameter('hand_polarity').value
            self.get_logger().info("Hand polarity is : %s" % self.polarity)
        except KeyError:
            self.get_logger().info("Could not find hand polarity parameter, setting to default (L_)")
            pass

        try:
            self.sensor_number = int(self.get_parameter('sensor_number').value)
            self.get_logger().info("Sensor number is : %s" % self.sensor_number)
        except KeyError:
            self.get_logger().info("Could not find sensor number parameter, setting to default (5)")
            pass

        self.baudrate = 1000000

        # Timer
        self.rate1 = self.create_rate(10)
        self.sensor_rate = self.create_rate(50)

        # publishers and subscribers
        self.sensor_publisher = self.create_publisher(AllSensors, self.polarity + 'AllSensors', 10)

        self.sensor_usr_cmd = self.create_subscription(SensorUserCommand,
                                                       self.polarity + 'sensor_usr_cmd',
                                                       self.sensor_usr_cmd_callback,
                                                       10)
        self.sensor_usr_cmd

        timer_period_ = 0.02
        self.timer = self.create_timer(timer_period_, self.loop_callback)

        try:
            self.sensor_read = serial.Serial(self.port, self.baudrate, timeout=1.5, write_timeout=1)

        except serial.SerialException:
            self.get_logger().error("Could not open serial port %s" % self.port)
            quit()

        self.get_logger().info('Serial connection to RH8D established!')

        self.sensors = [Sensor() for i in range(self.sensor_number)]

        # Calibrate sensors
        self.sensor_read.write('calibrate\r\n'.encode('ascii'))
        time.sleep(0.1)
        self.get_logger().info('RH8D Sensors calibration completed!')

        # Set the sensor's epoch
        secs_ = self.get_clock().now()
        msecs_ = self.get_clock().now().nanoseconds / 1000000
        msecs_ = int(msecs_)
        set_epoch_cmd_ = 'setepoch,' + str(secs_) + ',' + str(msecs_) + '\r\n'
        self.sensor_read.write(set_epoch_cmd_.encode('ascii'))
        self.get_logger().info('Sensors epoch is set up!')

        self.get_logger().info('RH8D Sensor node is initialized!')

    def sensor_usr_cmd_callback(self, data):

        if data.calibrate:
            self.sensor_read.write('calibrate\r\n'.encode('ascii'))
            self.get_logger().info("Sensors calibrated")

        if data.setepoch:
            secs_ = data.epoch_sec
            msecs_ = data.epoch_msec
            timestamp_ = secs_ + ',' + msecs_
            set_epoch_cmd_ = 'setepoch,' + secs_ + ',' + msecs_ + '\r\n'

            # Send the command to the sensor via serial port
            self.sensor_read.write(set_epoch_cmd_.encode('ascii'))
            self.get_logger().info("Command sent to sent epoch to %s" % timestamp_)
            self.rate1.sleep()

        if data.diagnosis_request:
            # Send the dump sensors command to get information
            self.sensor_read.write('dumpsensors\r\n'.encode('ascii'))
            self.rate1.sleep()
            # Send the command to get the hardware version of the sensors
            self.sensor_read.write('getversion\r\n'.encode('ascii'))
            self.rate1.sleep()
            # Send the command to get the software version of the sensors
            self.sensor_read.write('softwareversion\r\n'.encode('ascii'))
            self.rate1.sleep()
            # Send the command to get info about the board
            self.sensor_read.write('dumponboard\r\n'.encode('ascii'))
            self.rate1.sleep()
            # Send the resume command to get the sensors go back to their usual job
            self.sensor_read.write('resume\r\n'.encode('ascii'))

        if data.set_frequency:
            period_ms_ = (
                                 1 / data.frequency) * 1000  # User sets frequency, converting to ms to send the message to the sensors
            self.get_logger().info("requested period is :" + str(period_ms_))
            set_freq_str_ = "setperiod," + str(period_ms_)
            self.sensor_read.write(set_freq_str_.encode('ascii'))
            # Check if the requested frequency can be handled by the sensors
            if period_ms_ >= 20:
                self.get_logger().info("Sensor frequency set to %s" % str(data.frequency))
            else:
                self.get_logger().info(
                    "Tried to change frequency above the limit. Try again with a value between 1 and 50")
            self.rate1.sleep()

        if data.raw_string:  # If the user wants to send a custom command
            raw_ = data.raw  # Read the command
            self.sensor_read.write(raw_.encode('ascii'))  # Send it to the sensors
            self.get_logger().info("Command %s sent" % data.raw)
            self.get_logger().info("raw data sent")
            self.rate1.sleep()

    def parse_data_into_obj(self, data):
        # Check if the current line is correct
        if data[0] == '@':
            # Defining timestamp with the 2 first data : time (s) and time (ms)
            timestamp = data[1] + "," + data[2]
            for i, sensor in enumerate(self.sensors):
                if (data[5 + 3 * i] == ''):  # If Fx or Fy field is empty on the data stream
                    sensor.id = i  # Then put all fields to 0
                    sensor.fx = 0
                    sensor.fy = 0
                    sensor.fz = 0
                    sensor.is_present = False  # And warn that there is a sensor missing
                # If only the fz field is filled
                elif (data[5 + 3 * i] != '' and (data[3 + 3 * i] == '' or data[4 + 3 * i] == '')):
                    sensor.id = i
                    sensor.fx = 0
                    sensor.fy = 0
                    sensor.fz = data[5 + 3 * i]
                    sensor.is_3D = False  # Then it's not a 3D sensor but a 1D sensor
                else:  # Usual case : this is a 3D sensor
                    # Fill all the attributes
                    fx = int(data[3 + 3 * i])
                    fy = int(data[4 + 3 * i])
                    fz = int(data[5 + 3 * i])
                    sensor.id = i
                    sensor.fx = fx
                    sensor.fy = fy
                    sensor.fz = fz
                    r, theta, phi = cart2sph(fx, fy, fz)  # Convert cartesian coordinates to polar coordinates
                    sensor.abs = r  # Fill in the polar coordinates attributes
                    sensor.pitch = theta
                    sensor.yaw = phi
            return timestamp
        elif data[0] == '#OK':  # If the line starts with a #OK, then it's a response to a command
            message = ""
            for elem in data:  # Concatenate the message into a string
                message += elem
            self.get_logger().info(message)  # Log it into the ROS logs
            return None
        else:  # If the line doesn't start neither with a @ nor a #OK then it's an error
            # Display the read data
            try:
                self.get_logger().info("Serial data is :", data)

            except:
                pass

            # Log a ROS waring
            self.get_logger().warn("A sensor line could not be read : wrong start character" + str(data))
            return None

    def parse_data(self):
        # Read the first line
        line_ = self.sensor_read.readline()
        if line_:
            # Decode the first line from bytes to str
            line_str_ = line_.decode("utf-8")
            # Split the line into a str list
            data_ = line_str_.split(",")
            # Call to the function that put the str list into Sensor objects attributes
            timestamp_ = self.parse_data_into_obj(data_)
        else:
            return None
        return timestamp_

    def loop_callback(self):
        lone_sensor_msgs_ = [LoneSensor() for i in range(self.sensor_number)]
        final_msg_ = AllSensors()

        timestamp_ = self.parse_data()
        filtered_sensors_ = [sensor for sensor in self.sensors if sensor.is_present]

        # Loop that goes through the list of Sensor objects
        # Then fills each messages of the lone_sensor_msgs list with the Sensor objects' attributes
        for message, sensor in zip(lone_sensor_msgs_, filtered_sensors_):
            message.id = sensor.id
            message.fx = sensor.fx
            message.fy = sensor.fy
            message.fz = sensor.fz
            message.abs = sensor.abs
            message.yaw = sensor.yaw
            message.pitch = sensor.pitch
            message.is_present = sensor.is_present

        # Fill in the AllSensors message
        final_msg_.length = self.sensor_number  # Fill the length
        final_msg_.header.stamp = self.get_clock().now().to_msg()  # Get the timestamp
        final_msg_.data = lone_sensor_msgs_

        self.sensor_publisher.publish(final_msg_)


class Sensor:
    def __init__(self, id_=0, fx=0, fy=0, fz=0):
        self.id = id_
        self.fx = fx
        self.fy = fy
        self.fz = fz

        self.abs = 0.0
        self.yaw = 0.0
        self.pitch = 0.0
        self.is_present = True
        self.is_3d = True


# Defining a function to convert Cartesian coordinates to spherical coordinates
def cart2sph(x, y, z):
    hxy = np.hypot(x, y)
    r = np.hypot(hxy, z)
    el = np.pi / 2 - np.arctan2(hxy, z)
    az = np.arctan2(y, x)
    return r, el, az


def main(args=None):
    rclpy.init(args=args)

    sensor_node = RH8DSensorNode()

    rclpy.spin(sensor_node)

    sensor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
