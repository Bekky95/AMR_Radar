import serial
import time
import numpy as np

configFileName = "1642config.cfg"

CLIport = {}
Dataport = {}
byteBuffer = np.zeros(2**15, dtype="uint8")
byteBufferLength = 0


def serialConfig(configFileName):
    global CLIport, Dataport

    CLIport = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
    Dataport = serial.Serial("/dev/ttyACM1", 921600, timeout=1)

    print("Warte auf Board...")
    timeout = time.time() + 10
    while time.time() < timeout:
        if CLIport.in_waiting:
            data = CLIport.read_all().decode(errors="ignore")
            if "odsDemo:/>" in data or "mmwDemo:/>" in data:
                print("Board bereit!")
                break
        time.sleep(0.1)

    time.sleep(0.5)

    config = [line.rstrip("\r\n") for line in open(configFileName)]
    for i in config:
        if i.strip() == "" or i.startswith("%"):
            continue
        CLIport.write((i + "\n").encode())
        time.sleep(0.05)
        resp = CLIport.read_all().decode(errors="ignore").strip()
        print(f"{i[:50]:<50}  →  {resp[:30]}")

    return CLIport, Dataport  # ← dieser return muss außerhalb aller Schleifen stehen


def parseConfigFile(configFileName):
    configParameters = {}
    numTxAnt = 2

    config = [line.rstrip("\r\n") for line in open(configFileName)]
    for i in config:
        splitWords = i.split(" ")

        if "profileCfg" in splitWords[0]:
            startFreq = int(float(splitWords[2]))
            idleTime = int(splitWords[3])
            rampEndTime = float(splitWords[5])
            freqSlopeConst = float(splitWords[8])
            numAdcSamples = int(splitWords[10])
            numAdcSamplesRoundTo2 = 1
            while numAdcSamples > numAdcSamplesRoundTo2:
                numAdcSamplesRoundTo2 *= 2
            digOutSampleRate = int(splitWords[11])

        elif "frameCfg" in splitWords[0]:
            chirpStartIdx = int(splitWords[1])
            chirpEndIdx = int(splitWords[2])
            numLoops = int(splitWords[3])

    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
    configParameters["numRangeBins"] = numAdcSamplesRoundTo2
    configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (
        2 * freqSlopeConst * 1e12 * numAdcSamplesRoundTo2
    )
    configParameters["dopplerResolutionMps"] = 3e8 / (
        2
        * startFreq
        * 1e9
        * (idleTime + rampEndTime)
        * 1e-6
        * configParameters["numDopplerBins"]
        * numTxAnt
    )
    configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate) / (
        2 * freqSlopeConst * 1e3
    )
    configParameters["maxVelocity"] = 3e8 / (
        4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt
    )

    return configParameters


def readAndParseData16xx(Dataport, configParameters):
    global byteBuffer, byteBufferLength

    MMWDEMO_UART_MSG_DETECTED_POINTS = 1
    maxBufferSize = 2**15
    magicWord = [2, 1, 4, 3, 6, 5, 8, 7]

    magicOK = 0
    dataOK = 0
    frameNumber = 0
    detObj = {}

    waiting = Dataport.in_waiting or 1
    readBuffer = Dataport.read(waiting)
    byteVec = np.frombuffer(readBuffer, dtype="uint8")
    byteCount = len(byteVec)

    if (byteBufferLength + byteCount) < maxBufferSize:
        byteBuffer[byteBufferLength : byteBufferLength + byteCount] = byteVec[
            :byteCount
        ]
        byteBufferLength += byteCount

    if byteBufferLength > 16:
        possibleLocs = np.where(byteBuffer == magicWord[0])[0]
        startIdx = []
        for loc in possibleLocs:
            if np.all(byteBuffer[loc : loc + 8] == magicWord):
                startIdx.append(loc)

        if startIdx:
            if 0 < startIdx[0] < byteBufferLength:
                byteBuffer[: byteBufferLength - startIdx[0]] = byteBuffer[
                    startIdx[0] : byteBufferLength
                ]
                byteBuffer[byteBufferLength - startIdx[0] :] = 0
                byteBufferLength -= startIdx[0]

            byteBufferLength = max(byteBufferLength, 0)

            word = [1, 2**8, 2**16, 2**24]
            totalPacketLen = np.matmul(byteBuffer[12:16], word)

            if byteBufferLength >= totalPacketLen > 0:
                magicOK = 1

    if magicOK:
        word = [1, 2**8, 2**16, 2**24]
        idX = 0

        idX += 8  # magic
        idX += 4  # version
        totalPacketLen = np.matmul(byteBuffer[idX : idX + 4], word)
        idX += 4
        idX += 4  # platform
        frameNumber = np.matmul(byteBuffer[idX : idX + 4], word)
        idX += 4
        idX += 4  # timeCpuCycles
        numDetectedObj = np.matmul(byteBuffer[idX : idX + 4], word)
        idX += 4
        numTLVs = np.matmul(byteBuffer[idX : idX + 4], word)
        idX += 4
        idX += 4  # subFrameNumber

        for _ in range(numTLVs):
            try:
                tlv_type = np.matmul(byteBuffer[idX : idX + 4], word)
                idX += 4
                tlv_length = np.matmul(byteBuffer[idX : idX + 4], word)
                idX += 4
            except Exception:
                break

            if tlv_type == MMWDEMO_UART_MSG_DETECTED_POINTS:
                word2 = [1, 2**8]

                tlv_numObj = np.matmul(byteBuffer[idX : idX + 2], word2)
                idX += 2
                tlv_xyzQFormat = 2 ** np.matmul(byteBuffer[idX : idX + 2], word2)
                idX += 2

                rangeIdx = np.zeros(tlv_numObj, dtype="int32")
                dopplerIdx = np.zeros(tlv_numObj, dtype="int32")
                peakVal = np.zeros(tlv_numObj, dtype="int32")
                x = np.zeros(tlv_numObj, dtype="int32")
                y = np.zeros(tlv_numObj, dtype="int32")
                z = np.zeros(tlv_numObj, dtype="int32")

                for obj in range(tlv_numObj):
                    rangeIdx[obj] = np.matmul(byteBuffer[idX : idX + 2], word2)
                    idX += 2
                    dopplerIdx[obj] = np.matmul(byteBuffer[idX : idX + 2], word2)
                    idX += 2
                    peakVal[obj] = np.matmul(byteBuffer[idX : idX + 2], word2)
                    idX += 2
                    x[obj] = np.matmul(byteBuffer[idX : idX + 2], word2)
                    idX += 2
                    y[obj] = np.matmul(byteBuffer[idX : idX + 2], word2)
                    idX += 2
                    z[obj] = np.matmul(byteBuffer[idX : idX + 2], word2)
                    idX += 2

                # int16 Overflow korrigieren
                x[x > 32767] -= 65536
                y[y > 32767] -= 65536
                z[z > 32767] -= 65536

                dopplerIdx[
                    dopplerIdx > (configParameters["numDopplerBins"] / 2 - 1)
                ] -= 65535

                rangeVal = rangeIdx * configParameters["rangeIdxToMeters"]
                dopplerVal = dopplerIdx * configParameters["dopplerResolutionMps"]
                x = x / tlv_xyzQFormat
                y = y / tlv_xyzQFormat
                z = z / tlv_xyzQFormat

                detObj = {
                    "numObj": tlv_numObj,
                    "rangeIdx": rangeIdx,
                    "range": rangeVal,
                    "dopplerIdx": dopplerIdx,
                    "doppler": dopplerVal,
                    "peakVal": peakVal,
                    "x": x,
                    "y": y,
                    "z": z,
                }
                dataOK = 1

            else:
                idX += tlv_length  # unbekannte TLVs überspringen

        # Verarbeitete Daten entfernen
        if byteBufferLength > totalPacketLen:
            byteBuffer[: byteBufferLength - totalPacketLen] = byteBuffer[
                totalPacketLen:byteBufferLength
            ]
            byteBuffer[byteBufferLength - totalPacketLen :] = 0
            byteBufferLength -= totalPacketLen
            byteBufferLength = max(byteBufferLength, 0)

    return dataOK, frameNumber, detObj


def printFrame(frameNumber, detObj):
    print("\033[H\033[J", end="")  # Cursor oben + clear, kein Flackern
    n = detObj.get("numObj", 0)
    print(f"\n── Frame {frameNumber:>5} │ {n} Objekte ──────────────────────────────")
    if n == 0:
        print("  (keine Objekte erkannt)")
        return
    print(
        f"  {'Nr':>3}  {'X [m]':>7}  {'Y [m]':>7}  {'Z [m]':>7}  {'Range [m]':>10}  {'Doppler':>10}  {'Peak':>6}"
    )

    # for i in range(n):
    #     print(
    #         f"  {i:>3}  {detObj['x'][i]:>7.3f}  {detObj['y'][i]:>7.3f}  {detObj['z'][i]:>7.3f}  "
    #         f"{detObj['range'][i]:>10.3f}  {detObj['doppler'][i]:>10.3f}  {detObj['peakVal'][i]:>6}"
    #     )
    #

    print(
        f"  {0:>3}  {detObj['x'][0]:>7.3f}  {detObj['y'][0]:>7.3f}  {detObj['z'][0]:>7.3f}  "
        f"{detObj['range'][0]:>10.3f}  {detObj['doppler'][0]:>10.3f}  {detObj['peakVal'][0]:>6}"
    )


# ─── Main ─────────────────────────────────────────────────────────────────────
CLIport, Dataport = serialConfig(configFileName)
configParameters = parseConfigFile(configFileName)

print("\nLese Daten... (Ctrl+C zum Beenden)\n")

frameData = {}
currentIndex = 0

while True:
    try:
        dataOk, frameNumber, detObj = readAndParseData16xx(Dataport, configParameters)

        print(dataOk)
        if dataOk:
            printFrame(frameNumber, detObj)
            frameData[currentIndex] = detObj
            currentIndex += 1

        time.sleep(0.03)

    except KeyboardInterrupt:
        print("\nBeendet.")
        CLIport.write(b"sensorStop\n")
        CLIport.close()
        Dataport.close()
        break
