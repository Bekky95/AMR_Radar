import logging
import serial


def serial_port_closer(port_to_close: serial.Serial):
    try:
        if port_to_close.is_open:
            port_to_close.close()
    except Exception as e:
        logging.error("Trouble closing Serialport: ", e)
        raise e


def reset_serialport_buffer(port_to_reset: serial.Serial):
    """ resets serial port input buffer """
    try:
        port_to_reset.reset_input_buffer()
    except Exception as e:
        logging.error("Could not reset port buffer: ", e)
        raise e
