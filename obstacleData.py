from utils import *
import matplotlib.path as mplPath
import os, shutil
def obstacleInfo(obstaclePresent, obstacleE, obstacleN, obstacleChi, obstacleWidth, obstacleLength,
                             obstacleSafeWidth, obstacleSafeLength, obstacleSafeRadius):

    class obstacle():
        def __init__(self):

            self.Present = obstaclePresent
            self.E = obstacleE
            self.N = obstacleN
            self.Chi = obstacleChi
            self.w = obstacleWidth
            self.l = obstacleLength
            self.sw = obstacleSafeWidth
            self.sl = obstacleSafeLength
            self.sr = obstacleSafeRadius

            n = len(obstacleE)

            E_corners = np.zeros([n,4])
            N_corners = np.zeros([n,4])

            ERot_array = []
            NRot_array = []

            for k in range(n):

                xBL = obstacleE[k] - obstacleWidth[k] / 2
                xBR = obstacleE[k] + obstacleWidth[k] / 2
                xTR = obstacleE[k] + obstacleWidth[k] / 2
                xTL = obstacleE[k] - obstacleWidth[k] / 2

                yBL = obstacleN[k] - obstacleLength[k] / 2
                yBR = obstacleN[k] - obstacleLength[k] / 2
                yTR = obstacleN[k] + obstacleLength[k] / 2
                yTL = obstacleN[k] + obstacleLength[k] / 2

                E_array = np.array([xBL, xBR, xTR, xTL])
                N_array = np.array([yBL, yBR, yTR, yTL])

                theta = -obstacleChi[k]

                ERot_array, NRot_array = rotateRectangle(obstacleE[k], obstacleN[k], E_array, N_array, theta)

                E_corners[k] = ERot_array
                N_corners[k] = NRot_array

            self.E_corners = E_corners
            self.N_corners = N_corners

            pass

    return obstacle

# def createObstacleData(nE, nN, nU, gridsize, obstacle):
#
#     obstacleOnGrid = np.zeros([nE, nN, nU])
#     n = len(obstacle.E)
#
#     for i in range(n):
#         EGrid = np.floor( obstacle.E[i] / gridsize )
#         NGrid = np.floor( obstacle.N[i] / gridsize )
#         wGrid = np.ceil( obstacle.w[i] / gridsize )
#         lGrid = np.ceil( obstacle.l[i] / gridsize )
#
#         EGrid = np.int(EGrid)
#         NGrid = np.int(NGrid)
#         wGrid = np.int(wGrid)
#         lGrid = np.int(lGrid)
#
#         for j in range(wGrid):
#             for k in range(lGrid):
#                 obstacleOnGrid[EGrid + j, NGrid + k,:] = 1
#
#     # floor is an obstaclce
#     obstacleOnGrid[:,:,0] = 1
#
#     return obstacleOnGrid

def window(x0, detectionWindowParam):

    Chi = x0[3]

    l = detectionWindowParam['L']
    w = detectionWindowParam['W']

    # corners of detection window
    # p1Win = np.array([E - w/2, N])
    # p2Win = np.array([E + w/2, N])
    # p3Win = np.array([E + w/2, N + l])
    # p4Win = np.array([E - w/2, N + l])

    dp1Win = np.array([-w/2,0])
    dp2Win = np.array([w/2, 0])
    dp3Win = np.array([w/2,l])
    dp4Win = np.array([-w/2, l])

    dp1Win = rotate(dp1Win, Chi)
    dp2Win = rotate(dp2Win, Chi)
    dp3Win = rotate(dp3Win, Chi)
    dp4Win = rotate(dp4Win, Chi)

    p1Win = dp1Win + x0[0:2]
    p2Win = dp2Win + x0[0:2]
    p3Win = dp3Win + x0[0:2]
    p4Win = dp4Win + x0[0:2]

    return p1Win, p2Win, p3Win, p4Win

def detectObstacle(x0, detectionWindowParam, obstacle):

    # corners of obstacle
    p1Win, p2Win, p3Win, p4Win = window(x0, detectionWindowParam)

    # window
    bbPath = mplPath.Path(np.array([p1Win, p2Win, p3Win, p4Win]))

    nObs = obstacle.E.size
    detected = np.zeros(nObs, dtype=bool)
    obstacleID = np.array([], dtype=int)

    for k in range(nObs):

        p1Obs = np.array([obstacle.E_corners[k,0], obstacle.N_corners[k,0]])
        p2Obs = np.array([obstacle.E_corners[k,1], obstacle.N_corners[k,1]])
        p3Obs = np.array([obstacle.E_corners[k,2], obstacle.N_corners[k,2]])
        p4Obs = np.array([obstacle.E_corners[k,3], obstacle.N_corners[k,3]])

        # Current algorithm searches for detection of obstacle corners only in
        # detection window. This will be improved later on
        det1 = bbPath.contains_point(p1Obs)
        det2 = bbPath.contains_point(p2Obs)
        det3 = bbPath.contains_point(p3Obs)
        det4 = bbPath.contains_point(p4Obs)
        detected[k] = det1 or det2 or det3 or det4

        if detected[k] == True:
            obstacleID = np.concatenate([obstacleID, np.array([k])])
            #print('Obstacle {0:d} detected'.format(k))

            # if k >= 0:
            #     import matplotlib.pyplot as plt
            #     #
            #     # plt.figure(30)
            #     # plt.plot([p1Win[0], p2Win[0]], [p1Win[1], p2Win[1]], 'c')
            #     # plt.plot([p2Win[0], p3Win[0]], [p2Win[1], p3Win[1]], 'c')
            #     # plt.plot([p3Win[0], p4Win[0]], [p3Win[1], p4Win[1]], 'c')
            #     # plt.plot([p4Win[0], p1Win[0]], [p4Win[1], p1Win[1]], 'c')
            #     #
            #     # plt.plot(p1Obs[0], p1Obs[1], 'ro')
            #     # plt.plot(p2Obs[0], p2Obs[1], 'ro')
            #     # plt.plot(p3Obs[0], p3Obs[1], 'ro')
            #     # plt.plot(p4Obs[0], p4Obs[1], 'ro')
            #     #
            #     # plt.axis('equal')
            #     # plt.grid('on')
            #     # None

    return np.any(detected), obstacleID


def getObstacleData(obstacle, obstacleIdx):

    class obstacleIdxData(object):

        def __init__(self):
            self.Present = obstacle.Present
            self.E = np.array([obstacle.E[obstacleIdx]])
            self.N = np.array([obstacle.N[obstacleIdx]])
            self.w = np.array([obstacle.w[obstacleIdx]])
            self.l = np.array([obstacle.l[obstacleIdx]])
            self.Chi = np.array([obstacle.Chi[obstacleIdx]])
            pass

    return obstacleIdxData


def remainingObstacle(obstacle):

    class obstacleRemainingData(object):
        def __init__(self):

            self.Present = obstacle.Present

            n = obstacle.E.size
            if n > 1:
                self.E = obstacle.E[1:]
                self.N = obstacle.N[1:]
                self.w = obstacle.w[1:]
                self.l = obstacle.l[1:]
                self.Chi = obstacle.Chi[1:]
            else:
                self.E = np.array([obstacle.E[1:]])
                self.N = np.array([obstacle.N[1:]])
                self.w = np.array([obstacle.w[1:]])
                self.l = np.array([obstacle.l[1:]])
                self.Chi = np.array([obstacle.Chi[1:]])
            pass

    return obstacleRemainingData

def getCurrentObstacle(obstacle):

    class obstacleCurrentData(object):
        def __init__(self):

            self.Present = obstacle.Present
            self.E = np.array([obstacle.E[0]])
            self.N = np.array([obstacle.N[0]])
            self.w = np.array([obstacle.w[0]])
            self.l = np.array([obstacle.l[0]])
            self.Chi = np.array([obstacle.Chi[0]])

            pass

    return obstacleCurrentData

def createObstacleData(no, scaleFactorE, scaleFactorN, widthSpace, lengthSpace, horzDistance,
                        rundate, rundir):

    # Obstacle Data

    obstaclePresent = True
    obstacleLengthMargin = 1.0 * scaleFactorN  # ft
    obstacleWidthMargin = 1.0 * scaleFactorE  # ft

    if no == 0:

        obstaclePresent = False
        obstacleE = np.array([]) * scaleFactorE  # ft, left-bottom
        obstacleN = np.array([]) * scaleFactorN  # ft, left-bottom
        obstacleChi = np.array([])  # rad
        obstacleLength = np.array([]) * scaleFactorN  # ft
        obstacleWidth = np.array([]) * scaleFactorE  # ft

        obstacleSafeLength = obstacleLength + 2 * obstacleLengthMargin
        obstacleSafeWidth = obstacleWidth + 2 * obstacleWidthMargin
        obstacleSafeRadius = np.sqrt((obstacleSafeWidth / 2) ** 2 + (obstacleSafeLength / 2) ** 2)

    elif no == 1:

        obstacleE = np.array([7.0 - 0.5]) * scaleFactorE  # ft, center
        #obstacleE = np.array([16]) * scaleFactorE  # ft, center
        obstacleN = np.array([65.0]) * scaleFactorN  # ft, center
        obstacleChi = np.array([0.0])  # rad

        # small obstacle
        obstacleLength = np.array([4.0]) * scaleFactorN # ft
        obstacleWidth = np.array([10.0]) * scaleFactorE # ft

        # large object
        # obstacleLength = np.array([8.0]) * scaleFactorN  # ft
        # obstacleWidth = np.array([20.0]) * scaleFactorE  # ft

        obstacleSafeLength = obstacleLength + 2 * obstacleLengthMargin
        obstacleSafeWidth = obstacleWidth + 2 * obstacleWidthMargin
        obstacleSafeRadius = np.sqrt((obstacleSafeWidth / 2) ** 2 + (obstacleSafeLength / 2) ** 2)

    elif no == 2:
        obstacleE = np.array([-2.0, 12.0]) * scaleFactorE  # ft, left-bottom
        obstacleN = np.array([50.0, 65.0]) * scaleFactorN  # ft, left-bottom
        obstacleChi = np.array([0.0, 0.0])  # rad
        obstacleLength = np.array([15.0, 15.0]) * scaleFactorN  # ft
        obstacleWidth = np.array([15.0, 15.0]) * scaleFactorE  # ft

        obstacleSafeLength = obstacleLength + 2 * obstacleLengthMargin
        obstacleSafeWidth = obstacleWidth + 2 * obstacleWidthMargin
        obstacleSafeRadius = np.sqrt(obstacleSafeLength ** 2 + obstacleSafeWidth ** 2) / 2

    elif no == 4:
        obstacleE = np.array([6.0, 8.0, 18.0, 20.0]) * scaleFactorE  # ft, left-bottom
        obstacleN = np.array([31.0, 46.0, 58.0, 75.0]) * scaleFactorN  # ft, left-bottom
        obstacleChi = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # rad
        obstacleLength = np.array([4.0, 8.0, 4.0, 4.0]) * scaleFactorN  # ft
        obstacleWidth = np.array([4.0, 8.0, 4.0, 4.0]) * scaleFactorE  # ft

        obstacleSafeLength = obstacleLength + 2 * obstacleLengthMargin
        obstacleSafeWidth = obstacleWidth + 2 * obstacleWidthMargin
        obstacleSafeRadius = np.sqrt(obstacleSafeLength ** 2 + obstacleSafeWidth ** 2) / 2


    elif no == 5:
        obstacleE = np.array([6.0, 8.0, 18.0, 20.0, 20.0]) * scaleFactorE  # ft, left-bottom
        obstacleN = np.array([31.0, 46.0, 58.0, 75.0, 100.0]) * scaleFactorN  # ft, left-bottom
        obstacleChi = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # rad
        obstacleLength = np.array([4.0, 8.0, 4.0, 4.0, 4.0]) * scaleFactorN  # ft
        obstacleWidth = np.array([4.0, 8.0, 4.0, 4.0, 4.0]) * scaleFactorE  # ft

        obstacleSafeLength = obstacleLength + 2 * obstacleLengthMargin
        obstacleSafeWidth = obstacleWidth + 2 * obstacleWidthMargin
        obstacleSafeRadius = np.sqrt(obstacleSafeLength ** 2 + obstacleSafeWidth ** 2) / 2

    elif no == 6:
        obstacleE = np.array([8.0, 8.0, 8.0, 1.0, -6.0, -13.0]) * scaleFactorE + 5  # ft, left-bottom
        obstacleN = np.array([54.0, 62.0, 70.0, 70.0, 70.0, 70.0]) * scaleFactorN - 15  # ft, left-bottom
        obstacleChi = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # rad
        obstacleLength = np.array([4.0, 4.0, 4.0, 4.0, 4.0, 4.0]) * scaleFactorN  # ft
        obstacleWidth = np.array([4.0, 4.0, 4.0, 4.0, 4.0, 4.0]) * scaleFactorE  # ft

        obstacleSafeLength = obstacleLength + 2 * obstacleLengthMargin
        obstacleSafeWidth = obstacleWidth + 2 * obstacleWidthMargin
        obstacleSafeRadius = np.sqrt(obstacleSafeLength ** 2 + obstacleSafeWidth ** 2) / 2


    elif no == 7:
        obstacleE = np.array([8.0, 8.0, 8.0, 1.0, -6.0, -13.0, -13.0]) * scaleFactorE + 5  # ft, left-bottom
        obstacleN = np.array([50.0, 60.0, 70.0, 70.0, 70.0, 70.0, 60.0]) * scaleFactorN  # ft, left-bottom
        obstacleChi = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # rad
        obstacleLength = np.array([4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0]) * scaleFactorN  # ft
        obstacleWidth = np.array([4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0]) * scaleFactorE  # ft

        obstacleSafeLength = obstacleLength + 2 * obstacleLengthMargin
        obstacleSafeWidth = obstacleWidth + 2 * obstacleWidthMargin
        obstacleSafeRadius = np.sqrt(obstacleSafeLength ** 2 + obstacleSafeWidth ** 2) / 2


    elif no == 10:  # run 15 iterations
        # obstacleE = np.array(
        #     [8.0, 8.0, 8.0, 8.0, 1.0, -6.0, -13.0, -13.0, -13.0, -13.0]) * scaleFactorE + 5  # ft, left-bottom
        # obstacleN = np.array(
        #     [40.0, 50.0, 60.0, 70.0, 70.0, 70.0, 70.0, 60.0, 50.0, 40.0]) * scaleFactorN  # ft, left-bottom
        # obstacleChi = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # rad
        # obstacleLength = np.array([4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0]) * scaleFactorN  # ft
        # obstacleWidth = np.array([4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0]) * scaleFactorE  # ft

        obstacleE = np.array(
            [25.4, 29.8, 2.8, 14.0, 12.7, 6.8, 27.6, 58.5, 6.2, 45.7])  # ft, left-bottom
        obstacleN = np.array(
            [91.1, 70.9, 52.1, 55.6, 190.0, 92.2, 64.4, 177.6, 161.8, 100.5])  # ft, left-bottom

        # obstacleE = np.array(
        #     [-20, -10, 0, 10, 20, 30, 40, 50, 60, 70]) * scaleFactorE + 5  # ft, left-bottom
        # obstacleN = np.array(
        #     [30, 30, 30, 30, 30, 30, 30, 30, 30, 30]) * scaleFactorN  # ft, left-bottom
        obstacleChi = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # rad
        obstacleLength = np.array([4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0]) * scaleFactorN  # ft
        obstacleWidth = np.array([4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0]) * scaleFactorE  # ft


        obstacleSafeLength = obstacleLength + 2 * obstacleLengthMargin
        obstacleSafeWidth = obstacleWidth + 2 * obstacleWidthMargin
        obstacleSafeRadius = np.sqrt(obstacleSafeLength ** 2 + obstacleSafeWidth ** 2) / 2

    elif no == -1:
        nRand = 10
        obstacleE = np.random.uniform(0, widthSpace, nRand)
        delta_N0 = 50  # use so that obstacles are away from start and end points
        delta_N1 = 75
        obstacleN = np.random.uniform(0 + delta_N0, lengthSpace - delta_N1, nRand)
        obstacleChi = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # rad
        obstacleLength = 4 * np.ones(nRand) * scaleFactorN  # ft
        obstacleWidth = 4 * np.ones(nRand) * scaleFactorE  # ft

        obstacleSafeLength = obstacleLength + 2 * obstacleLengthMargin
        obstacleSafeWidth = obstacleWidth + 2 * obstacleWidthMargin
        obstacleSafeRadius = np.sqrt(obstacleSafeLength ** 2 + obstacleSafeWidth ** 2) / 2

    if len(obstacleSafeRadius) > 0:
        safeDistance = horzDistance + max(obstacleSafeRadius)
    else:
        safeDistance = horzDistance

    # Detection Window
    detectionWindowParam = {'L': safeDistance, 'W': max(obstacleWidth) * 2}

    if obstaclePresent:
        nObstacle = len(obstacleN)
    else:
        nObstacle = 0

    # save random obstacle data
    if no == -1:
        fileName_problemData2 = 'obstaclePositions_' + rundate + '.txt'
        f_problemData2 = open(fileName_problemData2, 'a')
        for k in range(nRand):
            f_problemData2.write("%.1f " % (obstacleE[k]))
        f_problemData2.write("\n")
        for k in range(nRand):
            f_problemData2.write("%.1f " % (obstacleN[k]))

        f_problemData2.close()
        # shutil.move(fileName_problemData2, rundir)
        shutil.move(os.path.join('.', fileName_problemData2),
                    os.path.join(rundir, fileName_problemData2))  # move overwrite

    return obstaclePresent, nObstacle, obstacleE, obstacleN, obstacleChi, obstacleLength, obstacleWidth, \
            obstacleSafeLength, obstacleSafeWidth, obstacleSafeRadius, safeDistance, detectionWindowParam