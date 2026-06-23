def parseConfigFile(configFileName) -> dict:
    """
    Liest die wichtigsten Radarparameter aus der cfg-Datei und berechnet daraus
    weitere Messgrößen für die Auswertung.

    Die Funktion wertet insbesondere die Zeilen `profileCfg` und `frameCfg` aus.
    Daraus werden unter anderem berechnet:
    - Anzahl der Range-Bins
    - Anzahl der Doppler-Bins
    - Range-Auflösung in Metern
    - Doppler-Auflösung in m/s
    - maximale Reichweite
    - maximale Geschwindigkeit

    Diese Werte werden später verwendet, um Rohdaten aus dem UART-Datenstrom in
    physikalische Größen umzuwandeln.

    Args:
        configFileName (str): Pfad zur Radar-Konfigurationsdatei.

    Returns:
        dict: Dictionary mit berechneten Radarparametern.
    """
    configParameters = {}

    config = load_config_lines(configFileName)

    numTxAnt = 2
    startFreq: int = 0
    idleTime: int = 0
    rampEndTime: float = 0.0
    freqSlopeConst: float = 0.0
    numAdcSamples: int = 0
    numAdcSamplesRoundTo2: int = 1
    digOutSampleRate: int = 0
    chirpStartIdx: int = 0
    chirpEndIdx: int = 0
    numLoops: int = 0
    numChirpsPerFrame: int = 0

    profileFound = False
    frameFound = False

    for line in config:
        splitWords = line.split(" ")

        if len(splitWords) == 0:
            continue

        # Anzahl der Antennen für die verwendete AWR1642-Konfiguration.
        # numRxAnt = 4

        if splitWords[0] == "profileCfg":
            profileCfg_dict = parse_profile_cfg(splitWords)
            
            # startFreq = int(float(splitWords[2]))
            # idleTime = int(splitWords[3])
            # rampEndTime = float(splitWords[5])
            # freqSlopeConst = float(splitWords[8])
            # numAdcSamples = int(splitWords[10])
            #
            # numAdcSamplesRoundTo2 = 1
            # while numAdcSamples > numAdcSamplesRoundTo2:
            #     numAdcSamplesRoundTo2 *= 2
            #
            # digOutSampleRate = int(splitWords[11])
            profileFound = True

        elif splitWords[0] == "frameCfg":
            frameCfg_dict = parse_frame_cfg(splitWords)

            # chirpStartIdx = int(splitWords[1])
            # chirpEndIdx = int(splitWords[2])
            # numLoops = int(splitWords[3])
            # numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops

            frameFound = True

    if not profileFound or not frameFound:
        raise ValueError(
            f"Config missing profileCfg {profileFound} or frameCfg {frameFound}"
        )

    configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
    configParameters["numRangeBins"] = numAdcSamplesRoundTo2

    configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (
        2 * freqSlopeConst * 1e12 * numAdcSamples
    )

    configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (
        2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"]
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

    configParameters["maxVelocity"] = (3e8) / (
        4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt
    )

    return configParameters


def load_config_lines(filename: str) -> list[str]:
    with open(filename, "r", encoding="utf-8") as file:
        lines = [line.rstrip("\r\n") for line in file]

    if len(lines) == 0:
        raise ValueError("No Data in config file")

    return lines


def next_power_of_two(value: int) -> int:
    result = 1
    while result < value:
        result *= 2
    return result


def parse_profile_cfg(words: list[str]) -> dict:
    cur_dict = {
        "startFreq": int(float(words[2])),
        "idleTime": int(words[3]),
        "rampEndTime": float(words[5]),
        "freqSlopeConst": float(words[8]),
        "numAdcSamples": int(words[10]),

    }
    numAdcSamplesRoundTo2 = 1
    while cur_dict["numAdcSamples"] > numAdcSamplesRoundTo2:
        numAdcSamplesRoundTo2 *= 2

    cur_dict.update({"numAdcSamplesRoundTo2": numAdcSamplesRoundTo2})
    cur_dict.update({"digOutSampleRate": int(words[11])})

    return cur_dict


def parse_frame_cfg(words: list[str]) -> dict:
    chirp_start = int(words[1])
    chirp_end = int(words[2])
    loops = int(words[3])

    return {
        "chirpStartIdx": chirp_start,
        "chirpEndIdx": chirp_end,
        "numLoops": loops,
        "numChirpsPerFrame": (chirp_end - chirp_start + 1) * loops,
    }


