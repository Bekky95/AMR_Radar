import time
import logging


def debug_output(
    radar_parser, dataOk, currentIndex, detObj, lastDebugTime, sec: int = 1
) -> float:
    """
     Einmal pro sec Sekunden Debug-Ausgabe.

    :param radar_parser:
    :param dataOk:
    :param currentIndex:
    :param detObj:
    :param lastDebugTime:
    :param sec: Sekunden Intervall

    :return: current time
    """
    if time.time() - lastDebugTime > sec:
        numObj = detObj.get("numObj", 0) if isinstance(detObj, dict) else 0
        logging.debug(
            f"bytes={radar_parser.get_dataport_in_waiting()}, "
            f"dataOk={dataOk}, "
            f"numObj={numObj}, "
            f"frames={currentIndex}"
        )

    return time.time()
