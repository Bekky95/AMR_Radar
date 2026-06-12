import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets

pg.setConfigOptions(useOpenGL=False, antialias=False)

from radar.readData_AWR1642 import readAndParseData16xx


# ------------------------------------------------------------------
# PLOT UPDATE
# ------------------------------------------------------------------

def update_with_filter(
    cur_Dataport, cur_configParameters, cur_detObj, cur_s, cur_visualizationFilter
):
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
            raw_x = -cur_detObj["x"]
            raw_y = cur_detObj["y"]

            # Nur die Visualisierung glätten. detObj und frameData enthalten
            # weiterhin die unveränderten Rohdaten des Radars.
            filtered_x, filtered_y = cur_visualizationFilter.apply(raw_x, raw_y)
            cur_s.setData(filtered_x, filtered_y)
        else:
            # Einen gültigen leeren Frame an den Filter weitergeben. Dadurch
            # verschwinden bestätigte Punkte erst nach max_missing_frames und
            # die Darstellung flackert bei einzelnen Aussetzern weniger.
            filtered_x, filtered_y = cur_visualizationFilter.apply([], [])
            cur_s.setData(filtered_x, filtered_y)

    # Wichtig: Nicht bei dataOk == 0 leeren.
    QtWidgets.QApplication.processEvents()

    return dataOk, cur_detObj
