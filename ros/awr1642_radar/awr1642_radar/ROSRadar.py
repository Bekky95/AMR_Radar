#!/usr/bin/env python3
"""AWR1642 Radar ROS2 Node"""

import time
import numpy as np
import serial
import sys
import glob
import serial
import re

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

# from example_interfaces.srv import GetFLoat64, GetInt64
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from awr1642_radar.radar.readData_AWR1642 import RadarParser

# def autodetect(ports: list):
#     dataPattern = r"(COM7|ACM1)"
#     configPattern = r"(COM8|ACM0)"
#     result = {}
#     for port in ports:
#         if re.search(configPattern, port):
#             result["config"] = port
#         elif re.search(dataPattern, port):
#             result["data"] = port
#
#     if "config" not in result and "data" not in result:
#         raise ValueError("No Device is connected. Make sure to plug in the device")
#
#     return result


# def serial_ports():
#     """Lists serial port names
#
#     :raises EnvironmentError:
#         On unsupported or unknown platforms
#     :returns:
#         A list of the serial ports available on the system
#     """
#     if sys.platform.startswith("win"):
#         ports = ["COM%s" % (i + 1) for i in range(256)]
#     elif sys.platform.startswith("linux") or sys.platform.startswith("cygwin"):
#         # this excludes your current terminal "/dev/tty"
#         ports = glob.glob("/dev/tty[A-Za-z]*")
#     elif sys.platform.startswith("darwin"):
#         ports = glob.glob("/dev/tty.*")
#     else:
#         raise EnvironmentError("Unsupported platform")
#
#     result = []
#     for port in ports:
#         try:
#             s = serial.Serial(port)
#             s.close()
#             result.append(port)
#         except (OSError, serial.SerialException):
#             pass
#     return autodetect(result)


class AWR1642Node(Node):
    def __init__(self):
        super().__init__("awr1642_radar")

        self.declare_parameter("config_file", "1642config.cfg")
        self.declare_parameter("frame_id", "radar")
        self.declare_parameter("timer_period", 0.033)

        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._config_file = self.get_parameter("config_file").get_parameter_value().string_value
        timer_period = self.get_parameter("timer_period").get_parameter_value().double_value

        self.get_logger().info("initialisiere Radar")
        self._awr1642Radar = RadarParser(self._config_file)
        self.get_logger().info("Radar initialisiert. Gerät sendet Daten")

        self._cli = None
        self._data = None

        # Publisher
        self._pub = self.create_publisher(PointCloud2, "/radar", 10)
        self._pubDist = self.create_publisher(Float32, "/distance", 10)

        # Init
        self.create_timer(timer_period, self._timer_cb)

    def _timer_cb(self):
        data_ok, det_obj = self._awr1642Radar.readAndParseData16xx()
        if not data_ok or det_obj.get("numObj", 0) == 0:
            return
        self._currentFrame = det_obj
        self._publish(det_obj)
        self.get_logger().debug(f"{det_obj['numObj']} Objekte")

    def _publish(self, det_obj: dict):
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="doppler", offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name="range", offset=20, datatype=PointField.FLOAT32, count=1),
        ]
        points = np.column_stack(
            [
                det_obj["x"],
                det_obj["y"],
                det_obj["z"],
                det_obj["peakVal"],
                det_obj["doppler"],
                det_obj["range"],
            ]
        ).astype("float32")

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._frame_id
        self._pub.publish(pc2.create_cloud(header, fields, points.tolist()))

        message = Float32()
        if not det_obj or det_obj.get("numObj", 0) == 0:
            message.data = -1.0
        else:
            message.data = float(np.min(det_obj["range"]))
        self._pubDist.publish(message)

    def destroy_node(self):
        self.get_logger().info("Stoppe Radar...")
        try:
            self._awr1642Radar.send_stop_command(0.3)
            self._awr1642Radar.close_serialports()
        except Exception as e:
            self.get_logger().error(str(e))
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AWR1642Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
