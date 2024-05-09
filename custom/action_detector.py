import numpy as np
import math
import time


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


class ActionDetector:

    def __init__(self, shouldFlip=False):
        self.wandLowered = (False, None)
        self.shouldFlip = shouldFlip

    # in the past x frames

    # detect horizontal: check x axis change only
    # detect raise wand: check y axis change only
    # detect rotate-side:  check rotation value

    # detect short attack: minimal x-axis changes, distance change from starting to ending yz is signifiiicant
    # detect long attack:

    def detectHorizontal(self, positions, rotations, moveThreshold=1.5):
        diffAcrossFrames = positions.max(axis=0) - positions.min(axis=0)
        # print(diffAcrossFrames)

        fitsCriteria = (
            diffAcrossFrames[0] < moveThreshold
            and diffAcrossFrames[1] > moveThreshold
            and diffAcrossFrames[2] < moveThreshold
        )
        # print("detect hori fit? ", fitsCriteria)

        return fitsCriteria

    def detectRaiseWand(self, positions, rotations, moveThreshold=1.0):
        diffAcrossFrames = positions.max(axis=0) - positions.min(axis=0)

        # if np.sum(diffAcrossFrames) > 0:
        #     print(diffAcrossFrames)
        fitsCriteria = (
            diffAcrossFrames[0] < 1.0
            and diffAcrossFrames[1] < 1.0
            and diffAcrossFrames[2] >= 1.5
        )
        # print("raise wand fit? ", fitsCriteria)
        if positions.argmin(axis=0)[2] > positions.argmax(axis=0)[2]:
            fitsCriteria = False
        # print(np.argmax(positions, axis=0))
        # print(np.argmin(positions, axis=0))
        # print("---")
        return fitsCriteria

    def detectRotateSide(self, positions, rotations, moveThreshold=95):

        # print(euler_from_quaternion(*rotations[0]))

        # temp = euler_from_quaternion(*rotations[0])
        diffAcrossFrames = rotations.max(axis=0) - rotations.min(axis=0)
        # print(diffAcrossFrames)
        fitsCriteria = (
            diffAcrossFrames[0] < moveThreshold
            and diffAcrossFrames[1] >= moveThreshold
            and diffAcrossFrames[2] < moveThreshold
        )
        return fitsCriteria

    def detectLowerWand(self, positions, rotations, moveThreshold=1.0):
        diffAcrossFrames = positions.max(axis=0) - positions.min(axis=0)

        fitsCriteria = (
            # diffAcrossFrames[0] < moveThreshold
            diffAcrossFrames[1] < 1.0
            and diffAcrossFrames[2] >= 1.5
        )
        if positions.argmin(axis=0)[2] < positions.argmax(axis=0)[2]:
            fitsCriteria = False
        # smallest value first, should go to raise wand instead
        # if positions.argmin(axis=0) < positions.argmax(axis=0):
        # fitsCriteria = False

        # print("diffacross frames [2] for detect lower wand: ", diffAcrossFrames[2])

        # check if wand raised
        return fitsCriteria

    def detectFastAttack(self, positions, rotations, moveThreshold=1.0):
        diffAcrossFrames = positions.max(axis=1) - positions.min(axis=1)

        fitsCriteria = (
            diffAcrossFrames[0] < moveThreshold
            and diffAcrossFrames[1] < moveThreshold
            and diffAcrossFrames[2] >= moveThreshold
        )

        # print("detecting fast attack")

        # check if wand raised
        return fitsCriteria

    def detectChargedAttack(self, positions, rotations, moveThreshold=1.0):
        # check first 40 frames to see if lowered

        # then check if maintains at a low y value

        wandLoweredFrames = 40
        diffAcrossFrames = positions[:wandLoweredFrames].max(axis=1) - positions[
            :wandLoweredFrames
        ].min(axis=1)

        wandLowered = (
            diffAcrossFrames[0] < moveThreshold
            and diffAcrossFrames[1] < moveThreshold
            and diffAcrossFrames[2] >= moveThreshold
        )

        # check if wand not raised
        return wandLowered

    def getAction(self, positionFrames, rotationFrames):

        detectors = {
            "detectRotateSide": {"fn": self.detectRotateSide, "framesToEvaluate": 40},
            "detectFastAttack": {"fn": self.detectLowerWand, "framesToEvaluate": 40},
            "detectChargedAttack": {
                "fn": self.detectHorizontal,
                "framesToEvaluate": 40,
            },
            "detectRaiseWand": {"fn": self.detectRaiseWand, "framesToEvaluate": 40},
            # "detectFastAttack": {"fn": detectFastAttack, "framesToEvaluate": 40},
            # "detectChargedAttack": {"fn": detectChargedAttack, "framesToEvaluate": 120},
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
                #  print(message)
                return detectorName

                #  if detectorName == "detectLowerWand":
                #      print("Wand lowered, no need to do anything yet")
                #      continue
                #  elif detectorName == "detectRaiseWand":
                #      if self.wandLowered[0]:

                #          waitFastSeconds = 2
                #          waitChargedSeconds = 5
                #          curTime = time.time()

                #          ret = ""
                #          if curTime - self.wandLowered[1] >= waitFastSeconds:
                #              ret = "detectFastAttack"
                #          elif curTime - self.wandLowered[1] >= waitChargedSeconds:
                #              ret = "detectChargedAttack"

                #          self.wandLowered = (False, None)
                #          if ret != "":
                #              return ret

                return detectorName
        return None


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
                # print(message)
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
