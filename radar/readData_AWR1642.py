from typing import Any

import serial
import time
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets
import logging

from radar.helper.byte_converter import bytes_to_uint16, bytes_to_int16
from radar.helper.config_file_parser import parseConfigFile
from radar.helper.serial_helper import serial_port_closer, reset_serialport_buffer
from radar.helper.PortFinder import serial_ports
from radar.helper.fixingdata import PointCloudLowPassFilter

MAGIC_WORD = b"\x02\x01\x04\x03\x06\x05\x08\x07"


class RadarParser:

    def __init__(self, configFileName: str, buffer_size: int = 2**15) -> None:
        """
        :param configFileName: <name>.cfg file
        :param buffer_size: default 2 ** 15 (32768)
        """
        # Byte-Puffer für eingehende UART-Daten:
        self._buffer_size = buffer_size
        self._buffer = np.zeros(self._buffer_size, dtype=np.uint8)
        self._length = 0
        self._obj_struct_size_bytes = 12
        self._mmwdemo_uart_msg_detected_points = 1

        # Schnittstellen für Steuerbefehle und Datenstrom:
        self._CLIport: serial.Serial
        self._Dataport: serial.Serial

        pg.setConfigOptions(useOpenGL=False, antialias=False)

        # Radarparameter aus der cfg-Datei berechnen:
        # self.configParameters = self.parseConfigFile(configFileName)
        self.configParameters = parseConfigFile(configFileName)

        # Serielle Ports konfigurieren und Radar starten:
        self._CLIport, self._Dataport = self._serialConfig(configFileName)
        self._resetParserBuffer()

    def __del__(self):
        serial_port_closer(self._Dataport)
        serial_port_closer(self._CLIport)

    # PRIVATE-METHODS------------------------------------------------------------------
    def _serialConfig(self, configFileName) -> tuple[serial.Serial, serial.Serial]:
        """
        Öffnet die seriellen Schnittstellen zum AWR1642-Radarboard und sendet die
        Konfiguration kontrolliert an das Board.

        Die Funktion stoppt zuerst einen eventuell noch laufenden Sensor, leert alle
        seriellen Puffer und sendet anschließend jede cfg-Zeile einzeln an das Board.
        Nach jedem Befehl wird die Antwort des Boards gelesen, damit Fehler beim
        Konfigurieren sichtbar werden.

        Args:
            configFileName (str): Pfad zur Radar-Konfigurationsdatei.

        Returns:
            tuple[serial.Serial, serial.Serial]: Geöffneter CLI-Port und Datenport.
        """
        serialPort = serial_ports()
        self._CLIport = serial.Serial(serialPort["config"], 115200, timeout=0.2)
        self._Dataport = serial.Serial(serialPort["data"], 921600, timeout=0.05)

        self._reset_all_buffers()

        time.sleep(0.3)

        # Falls der Sensor noch aus einem vorherigen Lauf streamt, sauber stoppen.
        self.send_stop_command(delay=0.2)
        self.reset_dataport_parser_buffers()

        time.sleep(0.3)

        with open(configFileName, "r", encoding="utf-8") as configFile:
            config = [line.rstrip("\r\n") for line in configFile]

        for line in config:
            cmd = line.strip()

            if cmd == "" or cmd.startswith("%"):
                continue

            if cmd == "sensorStart":
                # Vor sensorStart nochmal sicherstellen, dass keine alten Bytes im
                # Datenport oder Parser liegen.
                self.reset_dataport_parser_buffers()
                self.send_start_command(delay=0.1, timeout=0.5)
                # Erste unvollständige Frames nach Start verwerfen.
                time.sleep(0.3)
                self.reset_dataport_parser_buffers()

            elif cmd == "sensorStop":
                self.send_stop_command(delay=0.05)
                self.reset_dataport_parser_buffers()

            elif cmd == "flushCfg":
                self.send_flush_command(delay=0.05, timeout=0.3)
                self.reset_dataport_parser_buffers()

            else:
                self._send_cli_command(cmd, delay=0.05, timeout=0.3)
                logging.debug(f"command that is not sensorStart/-Stop or flushCfg: {cmd}")

        return self._CLIport, self._Dataport

    def _resetParserBuffer(self):
        """
        Leert den globalen UART-Parser-Puffer.

        Diese Funktion wird verwendet, wenn der Sensor gestoppt, neu gestartet oder
        neu konfiguriert wird. Dadurch bleiben keine alten oder unvollständigen
        Radar-Pakete im Parser zurück.
        """
        self._buffer = np.zeros(self._buffer_size, dtype="uint8")
        self._length = 0

    def _read_cli_response(self, timeout=1.0):
        """
        Liest die Antwort des Radarboards vom CLI-Port.

        Das Radarboard antwortet auf Konfigurationsbefehle typischerweise mit Text,
        z. B. Echo des Befehls, 'Done', 'Error' oder dem Prompt 'mmwDemo:/>'.

        Args:
            timeout (float): Maximale Wartezeit in Sekunden.

        Returns:
            str: Gelesene Antwort des Radarboards.
        """
        endTime = time.time() + timeout
        response = ""

        while time.time() < endTime:
            if self._CLIport.in_waiting > 0:
                response += self._CLIport.read(self._CLIport.in_waiting).decode(
                    errors="ignore"
                )

            time.sleep(0.01)

        return response.strip()

    def _send_cli_command(self, command, delay=0.05, timeout=1.0) -> str:
        """
        Sendet einen CLI-Befehl an das Radarboard und liest anschließend die Antwort.

        Im Gegensatz zu einem einfachen `CLIport.write()` wartet diese Funktion kurz
        auf die Rückmeldung des Boards. Dadurch erkennt man, ob ein Befehl wirklich
        angenommen wurde oder ob das Board einen Fehler meldet.

        Args:
            command (str): CLI-Befehl, z. B. 'sensorStop' oder 'sensorStart'.
            delay (float): Kurze Wartezeit nach dem Senden.
            timeout (float): Maximale Wartezeit auf die Antwort.

        Returns:
            str: Antwort des Radarboards.
        """
        self._CLIport.reset_input_buffer()
        self._CLIport.write((command + "\r\n").encode())
        self._CLIport.flush()

        time.sleep(delay)

        response = self._read_cli_response(timeout=timeout)

        if response:
            lastLine = response.splitlines()[-1]
        else:
            lastLine = "NO RESPONSE"

        logging.info(f"{command:<60} -> {lastLine}")

        if "Error" in response:
            logging.error("Radarboard meldet Fehler bei Befehl:", command)
            logging.error(response)

        return response

    def _reset_all_buffers(self) -> None:
        """
        resets input and output buffers for all Serial Ports and parser Buffer
        """
        self._CLIport.reset_input_buffer()
        self._CLIport.reset_output_buffer()
        self._Dataport.reset_input_buffer()
        self._Dataport.reset_output_buffer()
        self._resetParserBuffer()

    def _read_dataport_buffer(self) -> None:
        """
        reads Dataport Buffer, converts it to np.ndarray
        """
        readBuffer = self._Dataport.read(size=self._Dataport.in_waiting)
        if readBuffer is None:
            raise ValueError("Data buffer empty")

        byteVec = np.frombuffer(readBuffer, dtype="uint8")
        byteCount = len(byteVec)

        self._adjust_buffer_length(byteCount, byteVec)

    def _adjust_buffer_length(self, in_byteCount, in_byteVec) -> None:
        if (self._length + in_byteCount) < self._buffer_size:
            self._buffer[self._length: self._length + in_byteCount] = in_byteVec[:in_byteCount]
            self._length += in_byteCount

    # PUBLIC-METHODS------------------------------------------------------------------
    def get_dataport_in_waiting(self) -> int:
        return self._Dataport.in_waiting

    def close_serialports(self) -> None:
        serial_port_closer(self._Dataport)
        serial_port_closer(self._CLIport)

    def reset_dataport_parser_buffers(self) -> None:
        """
        Resets Dataport-Buffer and Byte-Buffer
        """
        reset_serialport_buffer(self._Dataport)
        self._resetParserBuffer()

    def send_stop_command(self, delay: float | int):
        """ calls _send_cli_command() for "sensorStop", timeout ist default 1.0 """
        self._send_cli_command("sensorStop", delay=delay, timeout=1.0)

    def send_start_command(self, delay: float | int, timeout: float | int):
        """ calls _send_cli_command() for "sensorStart" """
        self._send_cli_command("sensorStart", delay=delay, timeout=timeout)

    def send_flush_command(self, delay: float | int, timeout: float | int):
        """ calls _send_cli_command() for "flushCfg" """
        self._send_cli_command("flushCfg", delay=delay, timeout=timeout)

    def readAndParseData16xx(self) -> tuple[int, dict]:
        """
        Liest neue UART-Daten vom Datenport ein, sucht nach einem vollständigen
        Radar-Datenpaket und extrahiert erkannte Objekte.

        Das AWR1642-Radar sendet Datenpakete, die mit einem festen Magic Word
        beginnen:

            [2, 1, 4, 3, 6, 5, 8, 7]

        Danach folgt ein Header mit Informationen wie Paketlänge, Frame-Nummer,
        Anzahl erkannter Objekte und Anzahl der TLV-Blöcke.

        TLV steht für Type-Length-Value. Diese Funktion verarbeitet aktuell vor
        allem den TLV-Typ 1:

            MMWDEMO_UART_MSG_DETECTED_POINTS

        Dieser TLV enthält erkannte Punkte mit Range-Index, Doppler-Index,
        Peak-Wert und den kartesischen Koordinaten x, y und z.

        Returns:
            tuple[int, int, dict]:
                dataOK:
                    1, wenn gültige Objektdaten gelesen wurden, sonst 0.
                frameNumber:
                    Nummer des aktuellen Radarframes.
                detObj:
                    Dictionary mit erkannten Objekten.
        """

        # TODO: MAGIC_WORD einbauen und separate Funktion
        magicWord = [2, 1, 4, 3, 6, 5, 8, 7]

        magicOK = 0
        dataOK = 0
        detObj = {}
        tlv_type = 0

        self._read_dataport_buffer()

        if self._length > 16:
            possibleLocs = np.where(self._buffer == magicWord[0])[0]

            startIdx = []
            for loc in possibleLocs:
                check = self._buffer[loc: loc + 8]

                if np.all(check == magicWord):
                    startIdx.append(loc)

            if startIdx:
                if 0 < startIdx[0] < self._length:
                    self._buffer[: self._length - startIdx[0]] = self._buffer[
                        startIdx[0]: self._length
                    ]
                    self._buffer[self._length - startIdx[0]:] = np.zeros(
                        len(self._buffer[self._length - startIdx[0]:]), dtype="uint8"
                    )
                    self._length -= startIdx[0]

                if self._length < 0:
                    self._length = 0

                word = [1, 2**8, 2**16, 2**24]

                totalPacketLen = np.matmul(self._buffer[12: 12 + 4], word)

                if (self._length >= totalPacketLen) and (self._length != 0):
                    magicOK = 1

        if magicOK:
            word = [1, 2**8, 2**16, 2**24]

            idx = 12  # magicNumber (8 Bytes) + version (4 Bytes ) skippen

            totalPacketLen = np.matmul(self._buffer[idx: idx + 4], word)
            idx += (
                5 * 4
            )  # skip 4 felder (platform, frameNumber, timeCpuCycles, numDetectedObj)* 4 bytes + 4 Bytes für totalPacketLen

            numTLVs = np.matmul(self._buffer[idx: idx + 4], word)
            idx += 2 * 4  # skip 2 felder * 4 bytes

            for _ in range(numTLVs):
                word = [1, 2**8, 2**16, 2**24]

                try:
                    tlv_type = np.matmul(self._buffer[idx: idx + 4], word)
                    idx += 2 * 4  # skip ( tlv_length 4 bytes)

                except Exception as e:
                    logging.error("TLV Header konnte nicht geladen werden: ", e)
                    return 0, {}

                if tlv_type == self._mmwdemo_uart_msg_detected_points:
                    tlv_numObj = bytes_to_uint16(self._buffer, idx)
                    idx += 2

                    xyzQ = bytes_to_uint16(self._buffer, idx)
                    idx += 2

                    if xyzQ < 0 or xyzQ > 15:
                        logging.error("Ungültiges xyzQFormat:", xyzQ)
                        return 0, {}

                    tlv_xyzQFormat = 2**xyzQ

                    if tlv_numObj < 0 or tlv_numObj > 200:
                        logging.error("Ungültige Objektanzahl: ", tlv_numObj)
                        return 0, {}

                    neededBytes = tlv_numObj * self._obj_struct_size_bytes

                    if idx + neededBytes > totalPacketLen:
                        logging.error("Unvollständiges Paket:", tlv_numObj, "Objekte")
                        return 0, {}

                    rangeIdx = np.zeros(tlv_numObj, dtype="uint16")
                    dopplerIdx = np.zeros(tlv_numObj, dtype="int16")
                    peakVal = np.zeros(tlv_numObj, dtype="uint16")
                    x = np.zeros(tlv_numObj, dtype="int16")
                    y = np.zeros(tlv_numObj, dtype="int16")
                    # unused, because it needs the Dev Board, which has a second Antenna
                    # and this second antenna ist essencial für 3D Data
                    # --> Dev Board = ODS "Boost" Board
                    # https://github.com/ibaiGorordo/AWR1843-Read-Data-Python-MMWAVE-SDK-3-/issues/1
                    # !!! z var has to be read otherwise the point cloud is wrong !!!
                    z = np.zeros(tlv_numObj, dtype="int16")

                    for objectNum in range(tlv_numObj):
                        rangeIdx[objectNum] = bytes_to_uint16(self._buffer, idx)
                        idx += 2

                        dopplerIdx[objectNum] = bytes_to_int16(self._buffer, idx)
                        idx += 2

                        peakVal[objectNum] = bytes_to_uint16(self._buffer, idx)
                        idx += 2

                        x[objectNum] = bytes_to_int16(self._buffer, idx)
                        idx += 2

                        y[objectNum] = bytes_to_int16(self._buffer, idx)
                        idx += 2

                        z[objectNum] = bytes_to_int16(self._buffer, idx)
                        idx += 2

                    rangeVal = rangeIdx * self.configParameters["rangeIdxToMeters"]
                    dopplerVal = (
                        dopplerIdx * self.configParameters["dopplerResolutionMps"]
                    )

                    x = x / tlv_xyzQFormat
                    y = y / tlv_xyzQFormat
                    z = z / tlv_xyzQFormat

                    detObj = {
                        "numObj": tlv_numObj,
                        "rangeIdx": rangeIdx,
                        "range": rangeVal,
                        "dopplerIdx": dopplerIdx,
                        "doppler": dopplerVal,
                        "peakVal": peakVal,
                        "x": x,
                        "y": y,
                        "z": z,
                    }

                    dataOK = 1

            # Remove already processed data from the buffer.
            # Wichtig: Das Paket muss auch entfernt werden, wenn byteBufferLength
            # genau gleich totalPacketLen ist.
            if 0 < totalPacketLen <= self._length:
                shiftSize = int(totalPacketLen)
                remainingBytes = self._length - shiftSize

                if remainingBytes > 0:
                    self._buffer[:remainingBytes] = self._buffer[
                        shiftSize: self._length
                    ]

                self._buffer[remainingBytes:] = np.zeros(
                    len(self._buffer[remainingBytes:]), dtype="uint8"
                )

                self._length = remainingBytes

            else:
                # Falls die Paketlänge unplausibel ist, Buffer zurücksetzen,
                # damit der Parser nicht dauerhaft auf kaputten Daten hängen bleibt.
                self._buffer[:] = np.zeros(len(self._buffer), dtype="uint8")
                self._length = 0

        return dataOK, detObj

    # UPDATE------------------------------------------------------------------
    def update_with_filter(
        self, cur_s, cur_visualizationFilter: PointCloudLowPassFilter
    ) -> tuple[int, dict]:
        """
        Liest neue Radardaten ein und aktualisiert den Scatter-Plot.

        Der Plot wird nur aktualisiert, wenn ein vollständiges Radar-Paket gelesen
        wurde. Wenn gerade kein vollständiges Paket verfügbar ist, bleibt der letzte
        Plot-Zustand erhalten. Dadurch verschwinden Messpunkte nicht sofort zwischen
        zwei Radarframes.

        Returns:
            int: 1, wenn ein gültiges Datenpaket verarbeitet wurde, sonst 0.
        """
        dataOk, cur_detObj = self.readAndParseData16xx()

        if dataOk:
            if cur_detObj.get("numObj", 0) > 0:
                raw_x = -cur_detObj["x"]
                raw_y = cur_detObj["y"]

                # Nur die Visualisierung glätten. detObj und frameData enthalten
                # weiterhin die unveränderten Rohdaten des Radars.
                filtered_x, filtered_y = cur_visualizationFilter.apply(
                    raw_x, raw_y
                )
                cur_s.setData(filtered_x, filtered_y)  # , filtered_z)
            else:
                # Einen gültigen leeren Frame an den Filter weitergeben. Dadurch
                # verschwinden bestätigte Punkte erst nach max_missing_frames und
                # die Darstellung flackert bei einzelnen Aussetzern weniger.
                filtered_x, filtered_y = cur_visualizationFilter.apply([], [])
                cur_s.setData(filtered_x, filtered_y)

        # Wichtig: Nicht bei dataOk == 0 leeren.
        QtWidgets.QApplication.processEvents()

        return dataOk, cur_detObj
