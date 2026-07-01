import numpy as np
from PyQt5 import QtWidgets
import pyqtgraph as pg
import logging
import time
from radar.helper.debug_helper import debug_output
from radar.helper.fixingdata import PointCloudLowPassFilter
from radar.readData_AWR1642 import RadarParser

# logging.basicConfig(level=logging.DEBUG)
logging.basicConfig(level=logging.INFO)


# Name der Radar-Konfigurationsdatei
configFileName = "1642config.cfg" # profileCfg 0 77 447 7 40 0 0 100 1 64 2000 0 0 30
radar_parser = RadarParser(configFileName)

# Qt-Anwendung für den Plot erzeugen.
app = QtWidgets.QApplication([])

# Plot konfigurieren.
pg.setConfigOption("background", "w")

win = pg.GraphicsLayoutWidget(title="2D scatter plot AWR1642")
p = win.addPlot()

p.setXRange(-0.5, 0.5)
p.setYRange(0, 1.5)

p.setLabel("left", text="Y position (m)")
p.setLabel("bottom", text="X position (m)")

s = p.plot([], [], pen=None, symbol="o")

# Zustandsbehafteter Tiefpass ausschließlich für die Darstellung.
# Kleinere alpha-Werte glätten stärker, reagieren aber langsamer.
# TODO: in die __init__ bzw eigene Methode?
visualizationFilter = PointCloudLowPassFilter(
    alpha=0.35,
    max_match_distance=0.20,
    min_confirmations=2,
    max_missing_frames=1,
)

win.show()

# Speicher für die letzten Frames.
# TODO: in die __init__?
# TODO: vll eher nicht --> zurückgeben --> weiterverwenden
detObj = {}
frameData = {}
currentIndex = 0
MAX_FRAMES = 300

lastDebugTime = time.time()
lastFrameNumber = 0

while True:
    try:
        dataOk, detObj = radar_parser.update_with_filter(s, visualizationFilter)

        if dataOk:
            frameData[currentIndex % MAX_FRAMES] = detObj
            currentIndex += 1
            lastFrameNumber = currentIndex

        lastDebugTime = debug_output(radar_parser, dataOk, currentIndex, detObj, lastDebugTime, 1)

        # TODO: -----------------------------------------------------!!!!
        # ----------------------------------------------------------!!!!
        QtWidgets.QApplication.processEvents()
        time.sleep(0.03)

    except KeyboardInterrupt:
        logging.info("\nBeende Programm sauber...")

        try:
            radar_parser.send_stop_command(delay=0.3)
            time.sleep(0.3)
            radar_parser.reset_dataport_parser_buffers()

        except Exception as error:
            logging.error("Fehler beim Stoppen des Sensors:", error)
            raise error

        try:
            radar_parser.close_serialports()
            win.close()

        except Exception as error:
            logging.error("Fehler beim Schließen der Ports:", error)
            raise error

        logging.info("Ports geschlossen.")
        break
