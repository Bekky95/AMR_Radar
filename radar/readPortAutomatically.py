import sys
import glob
import serial
import re

import logging

def autodetect(ports: list):
    dataPattern = r"(COM7|ACM1)"
    configPattern = r"(COM8|ACM0)"
    result = {}
    for port in ports:
        if re.search(configPattern, port):
            result["config"] = port
            logging.info(f"config {port}")
        elif re.search(dataPattern, port):
            result["data"] = port
            logging.info(f"data {port}")

    return result


def serial_ports():
    """Lists serial port names

    :raises EnvironmentError:
        On unsupported or unknown platforms
    :returns:
        A list of the serial ports available on the system
    """
    if sys.platform.startswith("win"):
        ports = ["COM%s" % (i + 1) for i in range(256)]
    elif sys.platform.startswith("linux") or sys.platform.startswith("cygwin"):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob("/dev/tty[A-Za-z]*")
    elif sys.platform.startswith("darwin"):
        ports = glob.glob("/dev/tty.*")
    else:
        raise EnvironmentError("Unsupported platform")

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return autodetect(result)


if __name__ == "__main__":
    port_list = serial_ports()
    print(port_list)

    # if len(port_list) > 1:
    #     for port in range(1, len(port_list)):
    #         if int(port_list[port-1][-1])+1 == int(port_list[port][-1]):
    #             print(f"aufsteigend")
    #         else:
    #             print("Nö")
    # else:
    #     print("Not enough Ports")
