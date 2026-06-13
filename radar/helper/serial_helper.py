import logging
from operator import truediv

import serial


def serial_port_closer(port_to_close: serial.Serial) -> bool:
    try:
        port_to_close.close()
        return True
    except Exception as e:
        logging.error("Trouble closing Serialport: ", e)
        raise e
