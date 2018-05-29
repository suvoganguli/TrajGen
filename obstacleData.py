from utils import *
import matplotlib.path as mplPath

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

                xBL = obstacleE[k] - obstacleSafeWidth[k] / 2
                xBR = obstacleE[k] + obstacleSafeWidth[k] / 2
                xTR = obstacleE[k] + obstacleSafeWidth[k] / 2
                xTL = obstacleE[k] - obstacleSafeWidth[k] / 2

                yBL = obstacleN[k] - obstacleSafeLength[k] / 2
                yBR = obstacleN[k] - obstacleSafeLength[k] / 2
                yTR = obstacleN[k] + obstacleSafeLength[k] / 2
                yTL = obstacleN[k] + obstacleSafeLength[k] / 2

                E_array = np.array([xBL, xBR, xTR, xTL])
                N_array = np.array([yBL, yBR, yTR, yTL])

                theta = -obstacleChi[k]

                ERot_array, NRot_array = rotateRectangle(obstacleE[k], obstacleN[k], E_array, N_array, theta)

                E_corners[k] = ERot_array
                N_corners[k] = NRot_array

            # self.xBL = ERot_array[0]
            # self.xBR = ERot_array[1]
            # self.xTR = ERot_array[2]
            # self.xBL = ERot_array[3]
            #
            # self.yBL = NRot_array[0]
            # self.yBR = NRot_array[1]
            # self.yTR = NRot_array[2]
            # self.yBL = NRot_array[3]

            self.E_corners = E_corners
            self.N_corners = N_corners

            pass

    return obstacle

def createObstacleData(nE, nN, nU, gridsize, obstacle):

    obstacleOnGrid = np.zeros([nE, nN, nU])
    n = len(obstacle.E)

    for i in range(n):
        EGrid = np.floor( obstacle.E[i] / gridsize )
        NGrid = np.floor( obstacle.N[i] / gridsize )
        wGrid = np.ceil( obstacle.w[i] / gridsize )
        lGrid = np.ceil( obstacle.l[i] / gridsize )

        EGrid = np.int(EGrid)
        NGrid = np.int(NGrid)
        wGrid = np.int(wGrid)
        lGrid = np.int(lGrid)

        for j in range(wGrid):
            for k in range(lGrid):
                obstacleOnGrid[EGrid + j, NGrid + k,:] = 1

    # floor is an obstaclce
    obstacleOnGrid[:,:,0] = 1

    return obstacleOnGrid

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

    detected = False
    nObs = obstacle.E.size

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
        detected = det1 or det2 or det3 or det4

        if detected is True:
            print('Obstacle {0:d} detected'.format(k))


    return detected


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
