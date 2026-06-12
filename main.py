import numpy as np
from PyQt5 import QtWidgets
import pyqtgraph as pg
import time

from radar.helper.fixingdata import PointCloudLowPassFilter

# file with filter:
from radar.readData_AWR1642_TPF import update_with_filter

# original file without filter:
from radar.readData_AWR1642 import RadarParser

# LOGGING --------------------------------------------------------------------

import logging

# logging.basicConfig(level=logging.DEBUG)
logging.basicConfig(level=logging.INFO)

# GLOBAL VAR ------------------------------------------------------------------

# Name der Radar-Konfigurationsdatei
configFileName = "1642config.cfg"

# Globale Schnittstellen für Steuerbefehle und Datenstrom
CLIport = {}
Dataport = {}

# Globaler Byte-Puffer für eingehende UART-Daten
byteBuffer = np.zeros(2**15, dtype="uint8")
byteBufferLength = 0

radar_parser = RadarParser()

# -----------------------------------------------------------------------------
# MAIN PROGRAM
# -----------------------------------------------------------------------------

# Serielle Ports konfigurieren und Radar starten.
CLIport, Dataport = radar_parser.serialConfig(configFileName)

# Radarparameter aus der cfg-Datei berechnen.
configParameters = radar_parser.parseConfigFile(configFileName)

# Qt-Anwendung für den Plot erzeugen.
app = QtWidgets.QApplication([])

# Plot konfigurieren.
pg.setConfigOption("background", "w")

win = pg.GraphicsLayoutWidget(title="2D scatter plot")
p = win.addPlot()

p.setXRange(-0.5, 0.5)
p.setYRange(0, 1.5)

p.setLabel("left", text="Y position (m)")
p.setLabel("bottom", text="X position (m)")

s = p.plot([], [], pen=None, symbol="o")

# Zustandsbehafteter Tiefpass ausschließlich für die Darstellung.
# Kleinere alpha-Werte glätten stärker, reagieren aber langsamer.
visualizationFilter = PointCloudLowPassFilter(
    alpha=0.35,
    max_match_distance=0.20,
    min_confirmations=2,
    max_missing_frames=1,
)

win.show()


# Speicher für die letzten Frames.
detObj = {}
frameData = {}
currentIndex = 0
MAX_FRAMES = 300

lastDebugTime = time.time()
lastFrameNumber = 0

while True:
    try:
        dataOk, detObj = update_with_filter(
            Dataport, configParameters, detObj, s, visualizationFilter
        )

        if dataOk:
            frameData[currentIndex % MAX_FRAMES] = detObj
            currentIndex += 1
            lastFrameNumber = currentIndex

        # Einmal pro Sekunde Debug-Ausgabe.
        if time.time() - lastDebugTime > 1:
            numObj = detObj.get("numObj", 0) if isinstance(detObj, dict) else 0
            logging.debug(
                f"bytes={Dataport.in_waiting}, "
                f"dataOk={dataOk}, "
                f"numObj={numObj}, "
                f"frames={currentIndex}"
            )
            lastDebugTime = time.time()

        QtWidgets.QApplication.processEvents()
        time.sleep(0.03)

    except KeyboardInterrupt:
        logging.info("\nBeende Programm sauber...")

        try:
            radar_parser.send_cli_command("sensorStop", delay=0.3, timeout=1.0)
            time.sleep(0.3)

            Dataport.reset_input_buffer()
            radar_parser.resetParserBuffer()

        except Exception as error:
            logging.error("Fehler beim Stoppen des Sensors:", error)

        try:
            CLIport.close()
            Dataport.close()
            win.close()
        except Exception as error:
            logging.error("Fehler beim Schließen der Ports:", error)

        logging.info("Ports geschlossen.")
        break


# Version without TPF:
# # Serielle Ports konfigurieren und Radar starten.
# CLIport, Dataport = serialConfig(configFileName)
#
# # Radarparameter aus der cfg-Datei berechnen.
# configParameters = parseConfigFile(configFileName)
#
# # Qt-Anwendung für den Plot erzeugen.
# app = QtWidgets.QApplication([])
#
# # Plot konfigurieren.
# pg.setConfigOption("background", "w")
#
# win = pg.GraphicsLayoutWidget(title="2D scatter plot")
# p = win.addPlot()
#
# p.setXRange(-0.5, 0.5)
# p.setYRange(0, 1.5)
#
# p.setLabel("left", text="Y position (m)")
# p.setLabel("bottom", text="X position (m)")
#
# s = p.plot([], [], pen=None, symbol="o")
#
# win.show()
#
#
# # Speicher für die letzten Frames.
# detObj = {}
# frameData = {}
# currentIndex = 0
# MAX_FRAMES = 300
#
# lastDebugTime = time.time()
# lastFrameNumber = 0
#
# while True:
#     try:
#         dataOk = update()
#
#         if dataOk:
#             frameData[currentIndex % MAX_FRAMES] = detObj
#             currentIndex += 1
#             lastFrameNumber = currentIndex
#
#         # Einmal pro Sekunde Debug-Ausgabe.
#         if time.time() - lastDebugTime > 1:
#             numObj = detObj.get("numObj", 0) if isinstance(detObj, dict) else 0
#             print(
#                 f"bytes={Dataport.in_waiting}, "
#                 f"dataOk={dataOk}, "
#                 f"numObj={numObj}, "
#                 f"frames={currentIndex}"
#             )
#             lastDebugTime = time.time()
#
#         QtWidgets.QApplication.processEvents()
#         time.sleep(0.03)
#
#     except KeyboardInterrupt:
#         print("\nBeende Programm sauber...")
#
#         try:
#             send_cli_command("sensorStop", delay=0.3, timeout=1.0)
#             time.sleep(0.3)
#
#             Dataport.reset_input_buffer()
#             resetParserBuffer()
#
#         except Exception as error:
#             print("Fehler beim Stoppen des Sensors:", error)
#
#         try:
#             CLIport.close()
#             Dataport.close()
#             win.close()
#         except Exception as error:
#             print("Fehler beim Schließen der Ports:", error)
#
#         print("Ports geschlossen.")
#         break
