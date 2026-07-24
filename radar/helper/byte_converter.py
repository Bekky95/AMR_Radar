import struct


def bytes_to_uint16(byteBuffer, idx):
    return struct.unpack_from("<H", byteBuffer, idx)[0]  # signed, Little-Endian


def bytes_to_int16(byteBuffer, idx):
    return struct.unpack_from("<h", byteBuffer, idx)[0]  # signed, Little-Endian
