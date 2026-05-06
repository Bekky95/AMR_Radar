# pip install pyusb

import usb.core
import usb.util

dev = usb.core.find(idVendor=0x0451)  # TI Vendor ID
#dev.set_configuration()

print(f"Port Number: {dev.port_number}")
