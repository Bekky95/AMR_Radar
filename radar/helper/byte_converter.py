def bytes_to_uint16(byteBuffer, idx):
    """
    Wandelt zwei Bytes aus dem UART-Puffer in einen vorzeichenlosen 16-Bit-Wert um.

    Die Daten des Radars liegen im Little-Endian-Format vor. Das niederwertige
    Byte steht also zuerst.

    Args:
        byteBuffer (np.ndarray): Empfangspuffer mit uint8-Werten.
        idx (int): Startindex des ersten Bytes.

    Returns:
        int: Vorzeichenloser 16-Bit-Wert.
    """
    return int(byteBuffer[idx]) + (int(byteBuffer[idx + 1]) << 8)


def bytes_to_int16(byteBuffer, idx):
    """
    Wandelt zwei Bytes aus dem UART-Puffer in einen vorzeichenbehafteten
    16-Bit-Wert um.

    Werte größer als 32767 werden als negative int16-Werte interpretiert.

    Args:
        byteBuffer (np.ndarray): Empfangspuffer mit uint8-Werten.
        idx (int): Startindex des ersten Bytes.

    Returns:
        int: Vorzeichenbehafteter 16-Bit-Wert.
    """
    value = bytes_to_uint16(byteBuffer, idx)

    if value > 32767:
        value -= 65536

    return value