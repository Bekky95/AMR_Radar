import serial
import time
import numpy as np
import pyqtgraph as pg

from radar.helper.byte_converter import bytes_to_uint16, bytes_to_int16
from readPortAutomatically import serial_ports

pg.setConfigOptions(useOpenGL=False, antialias=False)

from pyqtgraph.Qt import QtWidgets

import logging


# ------------------------------------------------------------------
# SERIAL CONFIGURATION
# ------------------------------------------------------------------
def resetParserBuffer():
    """
    Leert den globalen UART-Parser-Puffer.

    Diese Funktion wird verwendet, wenn der Sensor gestoppt, neu gestartet oder
    neu konfiguriert wird. Dadurch bleiben keine alten oder unvollständigen
    Radar-Pakete im Parser zurück.
    """
    global byteBuffer, byteBufferLength

    byteBuffer[:] = np.zeros(len(byteBuffer), dtype="uint8")
    byteBufferLength = 0


def read_cli_response(timeout=1.0):
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
        if CLIport.in_waiting > 0:
            response += CLIport.read(CLIport.in_waiting).decode(errors="ignore")

        time.sleep(0.01)

    return response.strip()


def send_cli_command(command, delay=0.05, timeout=1.0):
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
    CLIport.reset_input_buffer()

    CLIport.write((command + "\r\n").encode())
    CLIport.flush()

    time.sleep(delay)

    response = read_cli_response(timeout=timeout)

    if response:
        lastLine = response.splitlines()[-1]
    else:
        lastLine = "NO RESPONSE"

    logging.info(f"{command:<60} -> {lastLine}")

    if "Error" in response:
        logging.error("Radarboard meldet Fehler bei Befehl:", command)
        logging.error(response)

    return response


def serialConfig(configFileName):
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
    global CLIport
    global Dataport

    serialPort = serial_ports()

    CLIport = serial.Serial(serialPort["config"], 115200, timeout=0.2)
    Dataport = serial.Serial(serialPort["data"], 921600, timeout=0.05)

    CLIport.reset_input_buffer()
    CLIport.reset_output_buffer()
    Dataport.reset_input_buffer()
    Dataport.reset_output_buffer()
    resetParserBuffer()

    time.sleep(0.3)

    # Falls der Sensor noch aus einem vorherigen Lauf streamt, sauber stoppen.
    send_cli_command("sensorStop", delay=0.2, timeout=1.0)

    Dataport.reset_input_buffer()
    Dataport.reset_output_buffer()
    resetParserBuffer()

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
            Dataport.reset_input_buffer()
            resetParserBuffer()

            response = send_cli_command(cmd, delay=0.1, timeout=0.5)

            # Erste unvollständige Frames nach Start verwerfen.
            time.sleep(0.3)
            Dataport.reset_input_buffer()
            resetParserBuffer()

        elif cmd == "sensorStop":
            response = send_cli_command(cmd, delay=0.05, timeout=0.3)
            Dataport.reset_input_buffer()
            resetParserBuffer()

        elif cmd == "flushCfg":
            response = send_cli_command(cmd, delay=0.05, timeout=0.3)
            Dataport.reset_input_buffer()
            resetParserBuffer()

        else:
            response = send_cli_command(cmd, delay=0.05, timeout=0.3)

    return CLIport, Dataport


# ------------------------------------------------------------------
# CONFIG PARSING
# ------------------------------------------------------------------


def parseConfigFile(configFileName):
    """
    Liest die wichtigsten Radarparameter aus der cfg-Datei und berechnet daraus
    weitere Messgrößen für die Auswertung.

    Die Funktion wertet insbesondere die Zeilen `profileCfg` und `frameCfg` aus.
    Daraus werden unter anderem berechnet:
    - Anzahl der Range-Bins
    - Anzahl der Doppler-Bins
    - Range-Auflösung in Metern
    - Doppler-Auflösung in m/s
    - maximale Reichweite
    - maximale Geschwindigkeit

    Diese Werte werden später verwendet, um Rohdaten aus dem UART-Datenstrom in
    physikalische Größen umzuwandeln.

    Args:
        configFileName (str): Pfad zur Radar-Konfigurationsdatei.

    Returns:
        dict: Dictionary mit berechneten Radarparametern.
    """
    configParameters = {}

    with open(configFileName, "r", encoding="utf-8") as configFile:
        config = [line.rstrip("\r\n") for line in configFile]

    for line in config:
        splitWords = line.split(" ")

        if len(splitWords) == 0:
            continue

        # Anzahl der Antennen für die verwendete AWR1642-Konfiguration.
        numRxAnt = 4
        numTxAnt = 2

        if "profileCfg" in splitWords[0]:
            startFreq = int(float(splitWords[2]))
            idleTime = int(splitWords[3])
            rampEndTime = float(splitWords[5])
            freqSlopeConst = float(splitWords[8])
            numAdcSamples = int(splitWords[10])

            numAdcSamplesRoundTo2 = 1
            while numAdcSamples > numAdcSamplesRoundTo2:
                numAdcSamplesRoundTo2 *= 2

            digOutSampleRate = int(splitWords[11])

        elif "frameCfg" in splitWords[0]:
            chirpStartIdx = int(splitWords[1])
            chirpEndIdx = int(splitWords[2])
            numLoops = int(splitWords[3])
            numFrames = int(splitWords[4])
            framePeriodicity = int(splitWords[5])

    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops

    configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
    configParameters["numRangeBins"] = numAdcSamplesRoundTo2

    configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (
        2 * freqSlopeConst * 1e12 * numAdcSamples
    )

    configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (
        2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"]
    )

    configParameters["dopplerResolutionMps"] = (3e8) / (
        2
        * startFreq
        * 1e9
        * (idleTime + rampEndTime)
        * 1e-6
        * configParameters["numDopplerBins"]
        * numTxAnt
    )

    configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate) / (
        2 * freqSlopeConst * 1e3
    )

    configParameters["maxVelocity"] = (3e8) / (
        4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt
    )

    return configParameters


# ------------------------------------------------------------------
# UART PARSING
# ------------------------------------------------------------------


def readAndParseData16xx(Dataport, configParameters):
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

    Args:
        Dataport (serial.Serial): Serieller Datenport des Radarboards.
        configParameters (dict): Berechnete Radarparameter aus der cfg-Datei.

    Returns:
        tuple[int, int, dict]:
            dataOK:
                1, wenn gültige Objektdaten gelesen wurden, sonst 0.
            frameNumber:
                Nummer des aktuellen Radarframes.
            detObj:
                Dictionary mit erkannten Objekten.
    """
    global byteBuffer
    global byteBufferLength

    OBJ_STRUCT_SIZE_BYTES = 12
    MMWDEMO_UART_MSG_DETECTED_POINTS = 1
    maxBufferSize = 2**15
    magicWord = [2, 1, 4, 3, 6, 5, 8, 7]

    magicOK = 0
    dataOK = 0
    frameNumber = 0
    detObj = {}
    tlv_type = 0

    readBuffer = Dataport.read(Dataport.in_waiting)
    byteVec = np.frombuffer(readBuffer, dtype="uint8")
    byteCount = len(byteVec)

    if (byteBufferLength + byteCount) < maxBufferSize:
        byteBuffer[byteBufferLength : byteBufferLength + byteCount] = byteVec[
            :byteCount
        ]
        byteBufferLength += byteCount

    if byteBufferLength > 16:
        possibleLocs = np.where(byteBuffer == magicWord[0])[0]

        startIdx = []
        for loc in possibleLocs:
            check = byteBuffer[loc : loc + 8]

            if np.all(check == magicWord):
                startIdx.append(loc)

        if startIdx:
            if startIdx[0] > 0 and startIdx[0] < byteBufferLength:
                byteBuffer[: byteBufferLength - startIdx[0]] = byteBuffer[
                    startIdx[0] : byteBufferLength
                ]
                byteBuffer[byteBufferLength - startIdx[0] :] = np.zeros(
                    len(byteBuffer[byteBufferLength - startIdx[0] :]), dtype="uint8"
                )
                byteBufferLength -= startIdx[0]

            if byteBufferLength < 0:
                byteBufferLength = 0

            word = [1, 2**8, 2**16, 2**24]

            totalPacketLen = np.matmul(byteBuffer[12 : 12 + 4], word)

            if (byteBufferLength >= totalPacketLen) and (byteBufferLength != 0):
                magicOK = 1

    if magicOK:
        word = [1, 2**8, 2**16, 2**24]

        idX = 0

        magicNumber = byteBuffer[idX : idX + 8]
        idX += 8

        version = format(np.matmul(byteBuffer[idX : idX + 4], word), "x")
        idX += 4

        totalPacketLen = np.matmul(byteBuffer[idX : idX + 4], word)
        idX += 4

        platform = format(np.matmul(byteBuffer[idX : idX + 4], word), "x")
        idX += 4

        frameNumber = np.matmul(byteBuffer[idX : idX + 4], word)
        idX += 4

        timeCpuCycles = np.matmul(byteBuffer[idX : idX + 4], word)
        idX += 4

        numDetectedObj = np.matmul(byteBuffer[idX : idX + 4], word)
        idX += 4

        numTLVs = np.matmul(byteBuffer[idX : idX + 4], word)
        idX += 4

        subFrameNumber = np.matmul(byteBuffer[idX : idX + 4], word)
        idX += 4

        for tlvIdx in range(numTLVs):
            word = [1, 2**8, 2**16, 2**24]

            try:
                tlv_type = np.matmul(byteBuffer[idX : idX + 4], word)
                idX += 4

                tlv_length = np.matmul(byteBuffer[idX : idX + 4], word)
                idX += 4

            except Exception:
                pass  # TODO: exception

            if tlv_type == MMWDEMO_UART_MSG_DETECTED_POINTS:
                tlv_numObj = bytes_to_uint16(byteBuffer, idX)
                idX += 2

                xyzQ = bytes_to_uint16(byteBuffer, idX)
                idX += 2

                if xyzQ < 0 or xyzQ > 15:
                    logging.error("Ungültiges xyzQFormat:", xyzQ)
                    return 0, frameNumber, {}

                tlv_xyzQFormat = 2**xyzQ

                if tlv_numObj < 0 or tlv_numObj > 200:
                    logging.error("Ungültige Objektanzahl: ", tlv_numObj)
                    return 0, frameNumber, {}

                neededBytes = tlv_numObj * OBJ_STRUCT_SIZE_BYTES

                if idX + neededBytes > totalPacketLen:
                    logging.error("Unvollständiges Paket:", tlv_numObj, "Objekte")
                    return 0, frameNumber, {}

                rangeIdx = np.zeros(tlv_numObj, dtype="uint16")
                dopplerIdx = np.zeros(tlv_numObj, dtype="int16")
                peakVal = np.zeros(tlv_numObj, dtype="uint16")
                x = np.zeros(tlv_numObj, dtype="int16")
                y = np.zeros(tlv_numObj, dtype="int16")
                z = np.zeros(tlv_numObj, dtype="int16")

                for objectNum in range(tlv_numObj):
                    rangeIdx[objectNum] = bytes_to_uint16(byteBuffer, idX)
                    idX += 2

                    dopplerIdx[objectNum] = bytes_to_int16(byteBuffer, idX)
                    idX += 2

                    peakVal[objectNum] = bytes_to_uint16(byteBuffer, idX)
                    idX += 2

                    x[objectNum] = bytes_to_int16(byteBuffer, idX)
                    idX += 2

                    y[objectNum] = bytes_to_int16(byteBuffer, idX)
                    idX += 2

                    z[objectNum] = bytes_to_int16(byteBuffer, idX)
                    idX += 2

                rangeVal = rangeIdx * configParameters["rangeIdxToMeters"]
                dopplerVal = dopplerIdx * configParameters["dopplerResolutionMps"]

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
        if totalPacketLen > 0 and byteBufferLength >= totalPacketLen:
            shiftSize = int(totalPacketLen)
            remainingBytes = byteBufferLength - shiftSize

            if remainingBytes > 0:
                byteBuffer[:remainingBytes] = byteBuffer[shiftSize:byteBufferLength]

            byteBuffer[remainingBytes:] = np.zeros(
                len(byteBuffer[remainingBytes:]), dtype="uint8"
            )

            byteBufferLength = remainingBytes

        else:
            # Falls die Paketlänge unplausibel ist, Buffer zurücksetzen,
            # damit der Parser nicht dauerhaft auf kaputten Daten hängen bleibt.
            byteBuffer[:] = np.zeros(len(byteBuffer), dtype="uint8")
            byteBufferLength = 0

    return dataOK, frameNumber, detObj


# ------------------------------------------------------------------
# PLOT UPDATE
# ------------------------------------------------------------------


def update_without_filter(cur_Dataport, cur_configParameters, cur_detObj, cur_s):
    """
    Liest neue Radardaten ein und aktualisiert den Scatter-Plot.

    Der Plot wird nur aktualisiert, wenn ein vollständiges Radar-Paket gelesen
    wurde. Wenn gerade kein vollständiges Paket verfügbar ist, bleibt der letzte
    Plot-Zustand erhalten. Dadurch verschwinden Messpunkte nicht sofort zwischen
    zwei Radarframes.

    Returns:
        int: 1, wenn ein gültiges Datenpaket verarbeitet wurde, sonst 0.
    """
    dataOk, frameNumber, cur_detObj = readAndParseData16xx(
        cur_Dataport, cur_configParameters
    )

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
