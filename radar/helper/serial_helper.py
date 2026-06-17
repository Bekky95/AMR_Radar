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


def reset_serialport_buffer(port_to_reset: serial.Serial) -> bool:
    try:
        port_to_reset.reset_input_buffer()
        # if port_to_reset.buffer.len == 0:  # pseudocode
        #     return True
        # else:
        #     return False
    except Exception as e:
        logging.error("Could not reset port buffer: ", e)
        # raise e
        return False
