import numpy as np
import math


def getData(filename):
    logfile = open(filename, "r")
    lines = logfile.readlines()
    logfile.close()

    parsedData = []

    parsedPosition = []
    parsedRotation = []

    for i in range(len(lines) // 2):
        positionText = lines[i * 2].strip()
        rotationText = lines[i * 2 + 1].strip()

        position = np.asarray([float(n) for n in positionText[1:-1].split(",")])
        rotation = np.asarray([float(n) for n in rotationText[1:-1].split(",")])

        rotation = euler_from_quaternion(*rotation)
        parsedPosition.append(position)
        parsedRotation.append(rotation)

        # parsedData.append((position, rotation))
        # print(position)
        # print(rotation)

    parsedPosition = np.asarray(parsedPosition)
    parsedRotation = np.asarray(parsedRotation)
    return parsedPosition, parsedRotation


# https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    # return roll_x, pitch_y, yaw_z # in radians
    return math.degrees(roll_x), math.degrees(pitch_y), math.degrees(yaw_z)


# -----


# in the past x frames

# detect horizontal: check x axis change only
# detect raise wand: check y axis change only
# detect rotate-side:  check rotation value

# detect short attack: minimal x-axis changes, distance change from starting to ending yz is signifiiicant
# detect long attack:


def detectHorizontal(positions, rotations, moveThreshold=1.5):
    diffAcrossFrames = positions.max(axis=0) - positions.min(axis=0)
    # print(diffAcrossFrames)
    fitsCriteria = (
        diffAcrossFrames[0] >= moveThreshold
        and diffAcrossFrames[1] < moveThreshold
        and diffAcrossFrames[2] < moveThreshold
    )
    # print("detect hori fit? ", fitsCriteria)

    return fitsCriteria


def detectRaiseWand(positions, rotations, moveThreshold=1.0):
    diffAcrossFrames = positions.max(axis=0) - positions.min(axis=0)

    # if np.sum(diffAcrossFrames) > 0:
    #     print(diffAcrossFrames)
    fitsCriteria = (
        diffAcrossFrames[0] < moveThreshold
        and diffAcrossFrames[1] < moveThreshold
        and diffAcrossFrames[2] >= moveThreshold
    )
    # print("rais wand fit? ", fitsCriteria)
    return fitsCriteria


def detectRotateSide(positions, rotations, moveThreshold=90):

    print(euler_from_quaternion(*rotations[0]))

    # temp = euler_from_quaternion(*rotations[0])
    diffAcrossFrames = rotations.max(axis=0) - rotations.min(axis=0)
    fitsCriteria = (
        diffAcrossFrames[0] < moveThreshold
        and diffAcrossFrames[1] < moveThreshold
        and diffAcrossFrames[2] >= moveThreshold
    )
    return fitsCriteria


def getAction(positionFrames, rotationFrames):

    detectors = {
        "detectRaiseWand": {"fn": detectRaiseWand, "framesToEvaluate": 40},
        "detectHorizontal": {"fn": detectHorizontal, "framesToEvaluate": 40},
        "detectRotateSide": {"fn": detectRotateSide, "framesToEvaluate": 10},
    }

    curLength = len(positionFrames)
    # print("curLength: ", curLength)

    for detectorName, fn in detectors.items():

        fn = detectors[detectorName]["fn"]
        framesToEvaluate = detectors[detectorName]["framesToEvaluate"]
        if curLength < framesToEvaluate:
            continue
        # print(
        #     "detecting from ",
        #     -framesToEvaluate,
        #     " to ",
        #     len(positionFrames),
        #     " for x frames ",
        #     len(positionFrames[-framesToEvaluate:]),
        # )

        fitsCriteria = fn(
            positionFrames[-framesToEvaluate:], rotationFrames[-framesToEvaluate:]
        )
        # fitsCriteria = fn(
        #    positionFrames[:framesToEvaluate], rotationFrames[:framesToEvaluate]
        # )

        if fitsCriteria:
            message = "frame 0" + "~" + str(framesToEvaluate) + " " + detectorName
            print(message)
            return detectorName


def test(filename):
    print(filename)
    positionFrames, rotationFrames = getData(filename)

    # framesToEvaluate = 40
    # detectors = {"detectHorizontal": detectHorizontal, "detectRaiseWand": detectRaiseWand}
    detectors = {
        "detectHorizontal": {"fn": detectHorizontal, "framesToEvaluate": 40},
        "detectRaiseWand": {"fn": detectRaiseWand, "framesToEvaluate": 40},
        "detectRotateSide": {"fn": detectRotateSide, "framesToEvaluate": 10},
    }

    for detectorName, fn in detectors.items():

        fn = detectors[detectorName]["fn"]
        framesToEvaluate = detectors[detectorName]["framesToEvaluate"]

        for i in range(0, len(positionFrames) - framesToEvaluate):
            fitsCriteria = fn(positionFrames[i : i + 80], rotationFrames[i : i + 80])
            if fitsCriteria:
                message = "frame " + str(i) + "~" + str(i + 80) + " " + detectorName
                print(message)
                # print(detectorName)

            continue
            isHorizontal = detectHorizontal(
                positionFrames[i : i + 80], rotationFrames[i : i + 80]
            )
            if isHorizontal:
                print("Is horizontal")


# test('data/rotate-side.txt')


# notes:
# rotate side inaccurate??
