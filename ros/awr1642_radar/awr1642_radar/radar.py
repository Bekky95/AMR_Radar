#!/usr/bin/env python3
"""AWR1642 Radar ROS2 Node"""

import time
import numpy as np
import serial

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

# from example_interfaces.srv import GetFLoat64, GetInt64
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2


class AWR1642Node(Node):
    MAGIC_WORD = np.array([2, 1, 4, 3, 6, 5, 8, 7], dtype="uint8")
    WORD = [1, 2**8, 2**16, 2**24]
    TLV_DETECTED_POINTS = 1
    OBJ_STRUCT_BYTES = 12
    MAX_BUFFER_SIZE = 2**15
    MAX_OBJECTS = 200

    def __init__(self):
        super().__init__("awr1642_radar")

        # Parameter
        self.declare_parameter("config_file", "1642config.cfg")
        self.declare_parameter("frame_id", "radar")
        self.declare_parameter("cli_port", "/dev/ttyACM0")
        self.declare_parameter("data_port", "/dev/ttyACM1")
        self.declare_parameter("timer_period", 0.033)

        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._config_file = self.get_parameter("config_file").get_parameter_value().string_value
        self._cli_port = self.get_parameter("cli_port").get_parameter_value().string_value
        self._data_port = self.get_parameter("data_port").get_parameter_value().string_value
        timer_period = self.get_parameter("timer_period").get_parameter_value().double_value

        # State
        self._cli = None
        self._data = None
        self._cfg = {}
        self._buf = np.zeros(self.MAX_BUFFER_SIZE, dtype="uint8")
        self._buf_len = 0
        self._currentFrame = {}

        # Publisher
        self._pub = self.create_publisher(PointCloud2, "/radar", 10)

        # Services
        # self._closestObject = self.create_service(
        #     GetFLoat64, "~/closestDistance", self.closestObjectCallback
        # )
        #
        # self._closestObject = self.create_service(
        #     GetInt64, "~/objectCount", self.getObjectCountCallback
        # )

        # TODO: Action Server hier registrieren
        # self._action_server = ActionServer(self, MyAction, "my_action", self._handle_action)

        # Init
        self._init_radar()
        self.create_timer(timer_period, self._timer_cb)

    # ------------------------------------------------------------------
    # INIT
    # ------------------------------------------------------------------

    def _init_radar(self):
        """Öffnet Ports, sendet Config, parst Parameter."""
        try:
            self._cli = serial.Serial(self._cli_port, 115200, timeout=0.2)
            self._data = serial.Serial(self._data_port, 921600, timeout=0.05)
        except serial.SerialException as e:
            self.get_logger().fatal(f"Port nicht gefunden: {e}")
            raise SystemExit(1)

        self._flush_all()
        self._send_cmd("sensorStop", delay=0.1, timeout=1.0)
        self._flush_all()
        time.sleep(0.3)

        with open(self._config_file, "r", encoding="utf-8") as f:
            lines = [l.rstrip() for l in f]

        for line in lines:
            cmd = line.strip()
            if not cmd or cmd.startswith("%"):
                continue
            if cmd == "sensorStart":
                self._flush_all()
                self._send_cmd(cmd, delay=0.1, timeout=2.0)
                time.sleep(0.3)
                self._flush_all()
            elif cmd in ("sensorStop", "flushCfg"):
                self._send_cmd(cmd, delay=0.05, timeout=1.0)
                self._flush_all()
            else:
                self._send_cmd(cmd, delay=0.05, timeout=0.3)

        self._cfg = self._parse_config(lines)
        self.get_logger().info("Radar initialisiert.")

    # ------------------------------------------------------------------
    # SERIAL UTILITIES
    # ------------------------------------------------------------------

    def _flush_all(self):
        for port in (self._cli, self._data):
            if port and port.is_open:
                port.reset_input_buffer()
                port.reset_output_buffer()
        self._buf[:] = 0
        self._buf_len = 0

    def _send_cmd(self, cmd: str, delay=0.05, timeout=1.0) -> str:
        self._cli.reset_input_buffer()
        self._cli.write((cmd + "\r\n").encode())
        self._cli.flush()
        time.sleep(delay)

        end, response = time.time() + timeout, ""
        while time.time() < end:
            if self._cli.in_waiting:
                response += self._cli.read(self._cli.in_waiting).decode(errors="ignore")
            time.sleep(0.01)

        last = response.splitlines()[-1] if response else "NO RESPONSE"
        self.get_logger().info(f"{cmd:<50} -> {last}")
        if "Error" in response:
            self.get_logger().warn(f"Radar Fehler bei '{cmd}': {response}")
        return response

    # ------------------------------------------------------------------
    # CONFIG PARSING
    # ------------------------------------------------------------------

    @staticmethod
    def _parse_config(lines: list[str]) -> dict:
        cfg, NUM_TX = {}, 2
        for line in lines:
            w = line.split()
            if not w:
                continue
            if "profileCfg" in w[0]:
                start_freq = int(float(w[2]))
                idle_time = int(w[3])
                ramp_end = float(w[5])
                freq_slope = float(w[8])
                num_adc = int(w[10])
                sample_rate = int(w[11])
                num_adc_r2 = 1
                while num_adc > num_adc_r2:
                    num_adc_r2 *= 2
            elif "frameCfg" in w[0]:
                chirp_start = int(w[1])
                chirp_end = int(w[2])
                num_loops = int(w[3])

        num_chirps = (chirp_end - chirp_start + 1) * num_loops
        num_doppler = num_chirps / NUM_TX

        cfg["numRangeBins"] = num_adc_r2
        cfg["rangeIdxToMeters"] = (3e8 * sample_rate * 1e3) / (2 * freq_slope * 1e12 * num_adc_r2)
        cfg["dopplerResolutionMps"] = (3e8) / (
            2 * start_freq * 1e9 * (idle_time + ramp_end) * 1e-6 * num_doppler * NUM_TX
        )
        cfg["maxRange"] = (300 * 0.9 * sample_rate) / (2 * freq_slope * 1e3)
        cfg["maxVelocity"] = (3e8) / (4 * start_freq * 1e9 * (idle_time + ramp_end) * 1e-6 * NUM_TX)
        return cfg

    # ------------------------------------------------------------------
    # BYTE HELPERS
    # ------------------------------------------------------------------

    @staticmethod
    def _u16(buf, i) -> int:
        return int(buf[i]) + (int(buf[i + 1]) << 8)

    @staticmethod
    def _i16(buf, i) -> int:
        v = int(buf[i]) + (int(buf[i + 1]) << 8)
        return v - 65536 if v > 32767 else v

    def _u32(self, buf, i) -> int:
        return int(np.matmul(buf[i : i + 4], self.WORD))

    # ------------------------------------------------------------------
    # UART PARSING
    # ------------------------------------------------------------------

    def _parse_frame(self) -> tuple[int, int, dict]:
        """
        Liest Daten vom Datenport, sucht Magic Word und extrahiert Objekte.

        Returns:
            (dataOK, frameNumber, detObj)
        """
        # Puffer befüllen
        raw = self._data.read(self._data.in_waiting)
        vec = np.frombuffer(raw, dtype="uint8")
        n = len(vec)
        if self._buf_len + n < self.MAX_BUFFER_SIZE:
            self._buf[self._buf_len : self._buf_len + n] = vec
            self._buf_len += n

        if self._buf_len <= 16:
            return 0, 0, {}

        # Magic Word finden und Puffer ausrichten
        hits = [
            i
            for i in np.where(self._buf == self.MAGIC_WORD[0])[0]
            if np.array_equal(self._buf[i : i + 8], self.MAGIC_WORD)
        ]
        if not hits:
            return 0, 0, {}

        if hits[0] > 0:
            s = hits[0]
            self._buf[: self._buf_len - s] = self._buf[s : self._buf_len]
            self._buf[self._buf_len - s :] = 0
            self._buf_len -= s

        total_len = self._u32(self._buf, 12)
        if self._buf_len < total_len:
            return 0, 0, {}

        # Header lesen
        i = 8  # skip magic
        i += 4  # version
        i += 4  # totalPacketLen
        i += 4  # platform
        frame_num = self._u32(self._buf, i)
        i += 4
        i += 4  # timeCpuCycles
        i += 4  # numDetectedObj
        num_tlvs = self._u32(self._buf, i)
        i += 4
        i += 4  # subFrameNumber

        data_ok, det_obj = 0, {}

        for _ in range(num_tlvs):
            try:
                tlv_type = self._u32(self._buf, i)
                i += 4
                tlv_len = self._u32(self._buf, i)
                i += 4
            except Exception:
                break

            if tlv_type != self.TLV_DETECTED_POINTS:
                i += int(tlv_len)
                continue

            num_obj = self._u16(self._buf, i)
            i += 2
            xyz_q = self._u16(self._buf, i)
            i += 2

            if not (0 <= xyz_q <= 15) or not (0 < num_obj <= self.MAX_OBJECTS):
                break
            if i + num_obj * self.OBJ_STRUCT_BYTES > total_len:
                break

            scale = 2**xyz_q
            fields_raw = np.zeros((num_obj, 6), dtype="int32")

            for k in range(num_obj):
                fields_raw[k] = [
                    self._u16(self._buf, i),  # rangeIdx
                    self._i16(self._buf, i + 2),  # dopplerIdx
                    self._u16(self._buf, i + 4),  # peakVal
                    self._i16(self._buf, i + 6),  # x
                    self._i16(self._buf, i + 8),  # y
                    self._i16(self._buf, i + 10),  # z
                ]
                i += self.OBJ_STRUCT_BYTES

            det_obj = {
                "numObj": num_obj,
                "range": fields_raw[:, 0] * self._cfg["rangeIdxToMeters"],
                "doppler": fields_raw[:, 1] * self._cfg["dopplerResolutionMps"],
                "peakVal": fields_raw[:, 2].astype(float),
                "x": -fields_raw[:, 3] / scale,  # gespiegelt wie Original
                "y": fields_raw[:, 4] / scale,
                "z": fields_raw[:, 5] / scale,
            }
            data_ok = 1

        # Verarbeitete Bytes entfernen
        rem = self._buf_len - int(total_len)
        if rem > 0:
            self._buf[:rem] = self._buf[int(total_len) : self._buf_len]
        self._buf[max(rem, 0) :] = 0
        self._buf_len = max(rem, 0)

        return data_ok, frame_num, det_obj

    # ------------------------------------------------------------------
    # PUBLISH
    # ------------------------------------------------------------------

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

    # def closestObjectCallback(self, request, response):
    #     if not self._last_det_obj or self._last_det_obj.get("numObj", 0) == 0:
    #         response.data = -1.0
    #         return response
    #     response.data = float(np.min(self._currentFrame["range"]))
    #     return response
    #
    # def getObjectCountCallback(self, request, response):
    #     if not self._last_det_obj:
    #         response.data = 0
    #         return response
    #     response.data = int(self._currentFrame.get("numObj"))
    #     return response
    #
    # ------------------------------------------------------------------
    # TIMER
    # ------------------------------------------------------------------

    def _timer_cb(self):
        data_ok, frame_num, det_obj = self._parse_frame()
        if not data_ok or det_obj.get("numObj", 0) == 0:
            return
        self._currentFrame = det_obj
        self._publish(det_obj)
        self.get_logger().debug(f"Frame {frame_num}: {det_obj['numObj']} Objekte")

    # ------------------------------------------------------------------
    # SHUTDOWN
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info("Stoppe Radar...")
        try:
            self._send_cmd("sensorStop", delay=0.3, timeout=1.0)
            for port in (self._cli, self._data):
                if port and port.is_open:
                    port.close()
        except Exception as e:
            self.get_logger().error(str(e))
        super().destroy_node()


# ------------------------------------------------------------------
# ENTRY POINT
# ------------------------------------------------------------------


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
