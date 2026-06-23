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
    # TODO DataClass Configparser draus machen!!!
    def __init__(self, configFileName: str, buffer_size: int = 2 ** 15) -> None:
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
        #self.configParameters = self.parseConfigFile(configFileName)
        self.configParameters = parseConfigFile(configFileName)

        # Serielle Ports konfigurieren und Radar starten:
        self._CLIport, self._Dataport = self.serialConfig(configFileName)
        # TODO sobald separate file config_handler.py steht das einkommentieren:
        # self._resetParserBuffer()

    def __del__(self):
        serial_port_closer(self._Dataport)
        serial_port_closer(self._CLIport)

    def close_serialports(self):
        serial_port_closer(self._Dataport)
        serial_port_closer(self._CLIport)

    # ------------------------------------------------------------------
    # SERIAL CONFIGURATION
    # ------------------------------------------------------------------
    def _resetParserBuffer(self):
        """
        Leert den globalen UART-Parser-Puffer.

        Diese Funktion wird verwendet, wenn der Sensor gestoppt, neu gestartet oder
        neu konfiguriert wird. Dadurch bleiben keine alten oder unvollständigen
        Radar-Pakete im Parser zurück.
        """
        self._buffer = np.zeros(self._buffer_size, dtype="uint8")
        self._length = 0

    def reset_buffers(self) -> None:
        """
        Resets Dataport-Buffer and Byte-Buffer
        """
        reset_serialport_buffer(self._Dataport)
        self._resetParserBuffer()

    def read_cli_response(self, timeout=1.0):
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
                response += self._CLIport.read(self._CLIport.in_waiting).decode(errors="ignore")

            time.sleep(0.01)

        return response.strip()

    def send_cli_command(self, command, delay=0.05, timeout=1.0) -> str:
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

        response = self.read_cli_response(timeout=timeout)

        if response:
            lastLine = response.splitlines()[-1]
        else:
            lastLine = "NO RESPONSE"

        logging.info(f"{command:<60} -> {lastLine}")

        if "Error" in response:
            logging.error("Radarboard meldet Fehler bei Befehl:", command)
            logging.error(response)

        return response

    def serialConfig(self, configFileName) -> tuple[serial.Serial, serial.Serial]:
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
        # TODO: aus dem Spaß eine eigene Funktion machen:
        serialPort = serial_ports()

        self._CLIport = serial.Serial(serialPort["config"], 115200, timeout=0.2)
        self._Dataport = serial.Serial(serialPort["data"], 921600, timeout=0.05)

        # TODO: Buffer reset allgemeine Funktion damit das nicht so aussieht
        self._CLIport.reset_input_buffer()
        self._CLIport.reset_output_buffer()
        self._Dataport.reset_input_buffer()
        self._Dataport.reset_output_buffer()
        self._resetParserBuffer()

        time.sleep(0.3)

        # Falls der Sensor noch aus einem vorherigen Lauf streamt, sauber stoppen.
        self.send_cli_command("sensorStop", delay=0.2, timeout=1.0)

        self._Dataport.reset_input_buffer()
        self._Dataport.reset_output_buffer()
        self._resetParserBuffer()

        time.sleep(0.3)

        with open(configFileName, "r", encoding="utf-8") as configFile:
            config = [line.rstrip("\r\n") for line in configFile]

        for line in config:
            cmd = line.strip()

            if cmd == "" or cmd.startswith("%"):
                continue

            # TODO: jeweils Inhalt der ifs in eigene Methoden:
            if cmd == "sensorStart":
                # Vor sensorStart nochmal sicherstellen, dass keine alten Bytes im
                # Datenport oder Parser liegen.
                self._Dataport.reset_input_buffer()
                self._resetParserBuffer()

                self.send_cli_command(cmd, delay=0.1, timeout=0.5)

                # Erste unvollständige Frames nach Start verwerfen.
                time.sleep(0.3)
                self._Dataport.reset_input_buffer()
                self._resetParserBuffer()

            # TODO: jeweils Inhalt der ifs in eigene Methoden:
            elif cmd == "sensorStop":
                self.send_cli_command(cmd, delay=0.05, timeout=0.3)
                self._Dataport.reset_input_buffer()
                self._resetParserBuffer()

            # TODO: jeweils Inhalt der ifs in eigene Methoden:
            elif cmd == "flushCfg":
                self.send_cli_command(cmd, delay=0.05, timeout=0.3)
                self._Dataport.reset_input_buffer()
                self._resetParserBuffer()

            else:
                self.send_cli_command(cmd, delay=0.05, timeout=0.3)

        return self._CLIport, self._Dataport

    # def parseConfigFile(self, configFileName):
    #     """
    #     Liest die wichtigsten Radarparameter aus der cfg-Datei und berechnet daraus
    #     weitere Messgrößen für die Auswertung.
    #
    #     Die Funktion wertet insbesondere die Zeilen `profileCfg` und `frameCfg` aus.
    #     Daraus werden unter anderem berechnet:
    #     - Anzahl der Range-Bins
    #     - Anzahl der Doppler-Bins
    #     - Range-Auflösung in Metern
    #     - Doppler-Auflösung in m/s
    #     - maximale Reichweite
    #     - maximale Geschwindigkeit
    #
    #     Diese Werte werden später verwendet, um Rohdaten aus dem UART-Datenstrom in
    #     physikalische Größen umzuwandeln.
    #
    #     Args:
    #         configFileName (str): Pfad zur Radar-Konfigurationsdatei.
    #
    #     Returns:
    #         dict: Dictionary mit berechneten Radarparametern.
    #     """
    #     configParameters = {}
    #
    #     with open(configFileName, "r", encoding="utf-8") as configFile:
    #         config = [line.rstrip("\r\n") for line in configFile]
    #
    #     numTxAnt = 2
    #     startFreq: int = 0
    #     idleTime: int = 0
    #     rampEndTime: float = 0.0
    #     freqSlopeConst: float = 0.0
    #     numAdcSamples: int = 0
    #     numAdcSamplesRoundTo2: int = 1
    #     digOutSampleRate: int = 0
    #     chirpStartIdx: int = 0
    #     chirpEndIdx: int = 0
    #     numLoops: int = 0
    #     numChirpsPerFrame: int = 0
    #
    #     profileFound = False
    #     frameFound = False
    #
    #     for line in config:
    #         splitWords = line.split(" ")
    #
    #         if len(splitWords) == 0:
    #             continue
    #
    #         # Anzahl der Antennen für die verwendete AWR1642-Konfiguration.
    #         # numRxAnt = 4
    #
    #         if "profileCfg" in splitWords[0]:
    #             startFreq = int(float(splitWords[2]))
    #             idleTime = int(splitWords[3])
    #             rampEndTime = float(splitWords[5])
    #             freqSlopeConst = float(splitWords[8])
    #             numAdcSamples = int(splitWords[10])
    #
    #             numAdcSamplesRoundTo2 = 1
    #             while numAdcSamples > numAdcSamplesRoundTo2:
    #                 numAdcSamplesRoundTo2 *= 2
    #
    #             digOutSampleRate = int(splitWords[11])
    #             profileFound = True
    #
    #         elif "frameCfg" in splitWords[0]:
    #             chirpStartIdx = int(splitWords[1])
    #             chirpEndIdx = int(splitWords[2])
    #             numLoops = int(splitWords[3])
    #             numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    #
    #             frameFound = True
    #
    #     if not profileFound or not frameFound:
    #         raise ValueError(f"Config missing profileCfg {profileFound} or frameCfg {frameFound}")
    #
    #     configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
    #     configParameters["numRangeBins"] = numAdcSamplesRoundTo2
    #
    #     configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (
    #             2 * freqSlopeConst * 1e12 * numAdcSamples
    #     )
    #
    #     configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (
    #             2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"]
    #     )
    #
    #     configParameters["dopplerResolutionMps"] = 3e8 / (
    #             2
    #             * startFreq
    #             * 1e9
    #             * (idleTime + rampEndTime)
    #             * 1e-6
    #             * configParameters["numDopplerBins"]
    #             * numTxAnt
    #     )
    #
    #     configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate) / (2 * freqSlopeConst * 1e3)
    #
    #     configParameters["maxVelocity"] = (3e8) / (
    #             4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt
    #     )
    #
    #     return configParameters

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

        # TODO: Buffer lesen in eigene Funktion und alles auf
        # TODO: Klassenvariablen übergeben
        readBuffer = self._Dataport.read(self._Dataport.in_waiting)
        if readBuffer is None:
            raise ValueError("Data buffer empty")

        byteVec = np.frombuffer(readBuffer, dtype="uint8")
        byteCount = len(byteVec)

        if (self._length + byteCount) < self._buffer_size:
            self._buffer[self._length: self._length + byteCount] = byteVec[:byteCount]
            self._length += byteCount

        if self._length > 16:
            possibleLocs = np.where(self._buffer == magicWord[0])[0]

            startIdx = []
            for loc in possibleLocs:
                check = self._buffer[loc: loc + 8]

                if np.all(check == magicWord):
                    startIdx.append(loc)

            if startIdx:
                if startIdx[0] > 0 and startIdx[0] < self._length:
                    self._buffer[: self._length - startIdx[0]] = self._buffer[
                                                                 startIdx[0]: self._length
                                                                 ]
                    self._buffer[self._length - startIdx[0]:] = np.zeros(
                        len(self._buffer[self._length - startIdx[0]:]), dtype="uint8"
                    )
                    self._length -= startIdx[0]

                if self._length < 0:
                    self._length = 0

                word = [1, 2 ** 8, 2 ** 16, 2 ** 24]

                totalPacketLen = np.matmul(self._buffer[12: 12 + 4], word)

                if (self._length >= totalPacketLen) and (self._length != 0):
                    magicOK = 1

        if magicOK:
            word = [1, 2 ** 8, 2 ** 16, 2 ** 24]

            idx = 12  # magicNumber (8 Bytes) + version (4 Bytes ) skippen

            totalPacketLen = np.matmul(self._buffer[idx: idx + 4], word)
            idx += (
                    5 * 4
            )  # skip 4 felder (platform, frameNumber, timeCpuCycles, numDetectedObj)* 4 bytes + 4 Bytes für totalPacketLen

            numTLVs = np.matmul(self._buffer[idx: idx + 4], word)
            idx += 2 * 4  # skip 2 felder * 4 bytes

            for _ in range(numTLVs):
                word = [1, 2 ** 8, 2 ** 16, 2 ** 24]

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

                    tlv_xyzQFormat = 2 ** xyzQ

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
                    dopplerVal = dopplerIdx * self.configParameters["dopplerResolutionMps"]

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
                    self._buffer[:remainingBytes] = self._buffer[shiftSize: self._length]

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

    # ------------------------------------------------------------------
    # PLOT UPDATE
    # ------------------------------------------------------------------

    def update_without_filter(self, cur_detObj, cur_s) -> tuple[int, dict]:
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
                x = -cur_detObj["x"]
                y = cur_detObj["y"]

                cur_s.setData(x, y)
            else:
                # Nur leeren, wenn ein gültiger Frame kam, aber keine Objekte enthält.
                cur_s.setData([], [])

        # Wichtig: Nicht bei dataOk == 0 leeren.
        QtWidgets.QApplication.processEvents()

        return dataOk, cur_detObj

    def update_with_filter(
            self, cur_detObj, cur_s, cur_visualizationFilter: PointCloudLowPassFilter
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
                # raw_z = cur_detObj["z"]  # Parameter existiert definitiv

                # z wird auch gefiltert:

                # Nur die Visualisierung glätten. detObj und frameData enthalten
                # weiterhin die unveränderten Rohdaten des Radars.
                filtered_x, filtered_y = cur_visualizationFilter.apply(raw_x, raw_y)  # , raw_z)
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
