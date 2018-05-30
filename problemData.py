import numpy as np
from utils import *
import datetime
import shutil, distutils.dir_util
from problemMaxIterData import *

# Units
mph2fps = 4.4/3

# ----------------------------------------------------------
# USER INPUTS
# ----------------------------------------------------------

# Grid selection

scaleFactorE = 2
scaleFactorN = 2
scaleFactorh = 1

widthSpace = 32 # ft
lengthSpace = 128  # ft

widthSpace = int(widthSpace * scaleFactorE) # ft
lengthSpace = int(lengthSpace * scaleFactorN)  # ft

# gridSize = 1 # ft/unit
# gridClass = createGrid(gridSize, lengthSpace, widthSpace, heightSpace)
# grid = gridClass()

# Start and End Points
# startPoint = np.array([7 * scaleFactorE, 1 * scaleFactorN])  # E (ft), N (ft)
# endPoint = np.array([(7+0.5) * scaleFactorE, 115 * scaleFactorN])  # E (ft), N (ft)

startPoint = np.array([7 * scaleFactorE, 1 * scaleFactorN])  # E (ft), N (ft)
endPoint   = np.array([7 * scaleFactorE, 115 * scaleFactorN])  # E (ft), N (ft)

# Correction for new path generation with popup obstacle
dNewPathAdjust = 2.0 * np.sqrt(scaleFactorN**2 + scaleFactorN**2)

# expt:
# 'N' - number of MPC time steps
# 'T - time step
# 'ns' - number of states
# 'no' - number of obstacles
# 'V0' - initial speed

sf_T = 1

# default
N = 8
T = 0.5*sf_T
ns = 4
no = 5  # 0, 1, 2, 4, 5, 6, 7
V0 = 10*mph2fps

# mpciterations = problemMaxIterData(N, ns, no, V0, sf_T)
mpciterations = 100

decelType = 'Slow'  # Slow or Fast

lb_reachedNearGoal = 20 #max([V0*N*T, 40]) # ft
lb_reachedGoal = 1 # ft
zeroDistanceChange = 1 # ft

decel = V0**2 / (2 * lb_reachedNearGoal) # fps2


# Number of states
# ns = 2:
#   x0 = E, x1 = N
#   u0 = V, u1 = chi
# ns = 4:
#   x0 = E, x1 = N, x2 = V, x3 = chi
#   u0 = Vdot, u1 = chidot
# ns = 6:
#   x0 = E, x1 = N, x2 = V, x3 = chi, x5 = Vdot, x6 = chidot
#   u0 = Vddot, u1 = chiddot

# Positon Index w.r.t. Path Sections
posIdx0 = {'number': 0}

# ----------------------------------------------------------
# Weighting functions and constraints for MPC problem

if ns == 4:

    # Ipopt settings
    nlpMaxIter = 500

    # Kinematic Constraints
    E0 = startPoint[0]  # ft (North, long)
    N0 = startPoint[1]  # ft (East, lat)
    Chi0 = 0 * np.pi / 180  # rad
    x0 = [E0, N0, V0, Chi0]  # E, N, V, Chi, Vdot, Chidot

    lb_VdotVal = -2  # fps2
    ub_VdotVal = 2 # fps2
    lb_ChidotVal = -30 * np.pi / 180 # rad/s2
    ub_ChidotVal = 30 * np.pi / 180 # rad/s2
    lataccel_maxVal = 0.25 * 32.2  # fps2
    useLatAccelCons = 1

    delta_V = 2 * mph2fps  # fps
    lb_VTerm = V0 - delta_V # not used for ncons_option = 2
    ub_VTerm = V0 + delta_V # not used for ncons_option = 2

    delChi_max_InView = 90 * np.pi / 180
    delChi_max_NotInView = 30 * np.pi / 180
    delChi_max = delChi_max_NotInView

    # 2018-05-24
    # W_P = 0.0
    # W_V = 1.0
    # W_Vdot = 2.0
    # W_Chidot = 0.5
    # W_gDist = 0.01 # 0.01
    # W_gChi = 1  # 1

    W_P = 0.0 # 0.0
    W_V = 2.0 # 1.0
    W_Vdot = 0.5 # 2.0
    W_Chidot = 1e-3 # 1e-3
    W_gDist = 0.01 # 0.01
    W_gChi = 1  # 1

    # Braking parameters

    lb_VdotValSlowDown = -4 # fps2
    lb_VTermSlowDown = 0 # fps

    V_cmd = V0  # fps

    # Terminal constraint
    delta_yRoad = 0.5  # ft # is this used?

    # Path parameters
    pathWidth = 5.0 # ft


elif ns == 6:

    # Ipopt settings
    nlpMaxIter = 500
    #mpciterations = 5

    # Kinematic Constraints

    E0 = startPoint[0]  # ft (North, long)
    N0 = startPoint[1]  # ft (East, lat)
    Chi0 = 0 * np.pi/180 # rad (w.r.t. North)
    x0 = [E0, N0, V0, Chi0, 0, 0]  # E, N, V, Chi, Vdot, Chidot
    lb_VddotVal = -2 # fps3
    ub_VddotVal = 2 # fps3
    lb_ChiddotVal = -20*np.pi/180 # rad/s2
    ub_ChiddotVal = 20*np.pi/180 # rad/s2
    lataccel_maxVal = 0.25*32.2 # fps2
    useLatAccelCons = 1
    lb_V = 0.8*V0 # not used for ncons_option = 2
    ub_V = 1.2*V0 # not used for ncons_option = 2
    delChi_max_InView = 90 * np.pi / 180
    delChi_max_NotInView = 30 * np.pi / 180

    # Tracking Tuning and Data
    W_P = 0.0     #1.0
    W_V = 1.0
    W_Vddot = 1.0   # 20.0
    W_Chiddot = 1e-2 #0.1
    W_gDist = 1.0*0
    W_gChi = 1.0

    V_cmd = V0  # fps

    # Terminal constraint
    delta_yRoad = 0.1 # ft
    delta_V = 1*mph2fps # fps

    # Path parameters
    pathWidth = 5.0 # ft

# ------------------------------------------------------------

# Obstacle Data

obstaclePresent = True
obstacleLengthMargin = 2.5 * scaleFactorN # ft
obstacleWidthMargin = 2.5 * scaleFactorE# ft

if no == 0:

    obstaclePresent = False
    obstacleE = np.array([]) * scaleFactorE # ft, left-bottom
    obstacleN = np.array([]) * scaleFactorN # ft, left-bottom
    obstacleChi = np.array([])  # rad
    obstacleLength = np.array([]) * scaleFactorN # ft
    obstacleWidth = np.array([]) * scaleFactorE # ft

    obstacleSafeLength = obstacleLength + 2*obstacleLengthMargin
    obstacleSafeWidth = obstacleWidth + 2*obstacleWidthMargin
    obstacleSafeRadius = np.sqrt((obstacleSafeWidth/2)**2 + (obstacleSafeLength/2)**2)

elif no == 1:

    obstacleE = np.array([7.0 - 0.5]) * scaleFactorE # ft, center
    obstacleN = np.array([65.0]) * scaleFactorN # ft, center
    obstacleChi = np.array([0.0])  # rad

    # small obstacle
    # obstacleLength = np.array([4.0]) * scaleFactorN # ft
    # obstacleWidth = np.array([6.0]) * scaleFactorE # ft
    # obstacleSafeLength = obstacleLength + 2*obstacleLengthMargin
    # obstacleSafeWidth = obstacleWidth + 2*obstacleWidthMargin

    # large object
    obstacleLength = np.array([8.0]) * scaleFactorN # ft
    obstacleWidth = np.array([20.0]) * scaleFactorE # ft
    obstacleSafeLength = obstacleLength + 2*obstacleLengthMargin
    obstacleSafeWidth = obstacleWidth + 2*obstacleWidthMargin


    obstacleSafeRadius = np.sqrt((obstacleSafeWidth/2)**2 + (obstacleSafeLength/2)**2)

elif no == 2:
    obstacleE = np.array([-2.0, 12.0]) * scaleFactorE # ft, left-bottom
    obstacleN = np.array([50.0, 65.0]) * scaleFactorN # ft, left-bottom
    obstacleChi = np.array([0.0, 0.0])  # rad
    obstacleLength = np.array([15.0, 15.0]) * scaleFactorN # ft
    obstacleWidth = np.array([15.0, 15.0]) * scaleFactorE # ft

    obstacleSafeLength = obstacleLength + 2 * obstacleLengthMargin
    obstacleSafeWidth = obstacleWidth + 2 * obstacleWidthMargin
    obstacleSafeRadius = np.sqrt(obstacleSafeLength ** 2 + obstacleSafeWidth ** 2) / 2

elif no == 4:
    obstacleE = np.array([6.0, 8.0, 18.0, 20.0]) * scaleFactorE # ft, left-bottom
    obstacleN = np.array([31.0, 46.0, 58.0, 75.0]) * scaleFactorN # ft, left-bottom
    obstacleChi = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # rad
    obstacleLength = np.array([4.0, 8.0, 4.0, 4.0]) * scaleFactorN # ft
    obstacleWidth  = np.array([4.0, 8.0, 4.0, 4.0]) * scaleFactorE # ft

    obstacleSafeLength = obstacleLength + 2 * obstacleLengthMargin
    obstacleSafeWidth = obstacleWidth + 2 * obstacleWidthMargin
    obstacleSafeRadius = np.sqrt(obstacleSafeLength ** 2 + obstacleSafeWidth ** 2) / 2



elif no == 5:
    obstacleE = np.array([6.0, 8.0, 18.0, 20.0, 20.0]) * scaleFactorE # ft, left-bottom
    obstacleN = np.array([31.0, 46.0, 58.0, 75.0, 100.0]) * scaleFactorN # ft, left-bottom
    obstacleChi = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # rad
    obstacleLength = np.array([4.0, 8.0, 4.0, 4.0, 4.0]) * scaleFactorN # ft
    obstacleWidth  = np.array([4.0, 8.0, 4.0, 4.0, 4.0]) * scaleFactorE # ft

    obstacleSafeLength = obstacleLength + 2 * obstacleLengthMargin
    obstacleSafeWidth = obstacleWidth + 2 * obstacleWidthMargin
    obstacleSafeRadius = np.sqrt(obstacleSafeLength ** 2 + obstacleSafeWidth ** 2) / 2

elif no == 6:
    obstacleE = np.array([8.0, 8.0, 8.0, 1.0, -6.0, -13.0]) * scaleFactorE + 5 # ft, left-bottom
    obstacleN = np.array([50.0, 60.0, 70.0, 70.0, 70.0, 70.0]) * scaleFactorN - 15 # ft, left-bottom
    obstacleChi = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # rad
    obstacleLength = np.array([4.0, 4.0, 4.0, 4.0, 4.0, 4.0]) * scaleFactorN # ft
    obstacleWidth  = np.array([4.0, 4.0, 4.0, 4.0, 4.0, 4.0]) * scaleFactorE # ft

    obstacleSafeLength = obstacleLength + 2 * obstacleLengthMargin
    obstacleSafeWidth = obstacleWidth + 2 * obstacleWidthMargin
    obstacleSafeRadius = np.sqrt(obstacleSafeLength ** 2 + obstacleSafeWidth ** 2) / 2


elif no == 7:
    obstacleE = np.array([8.0, 8.0, 8.0, 8.0, 1.0, -6.0, -13.0]) * scaleFactorE + 5 # ft, left-bottom
    obstacleN = np.array([40.0, 50.0, 60.0, 70.0, 70.0, 70.0, 70.0]) * scaleFactorN # ft, left-bottom
    obstacleChi = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # rad
    obstacleLength = np.array([4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0]) * scaleFactorN # ft
    obstacleWidth  = np.array([4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0]) * scaleFactorE # ft

    obstacleSafeLength = obstacleLength + 2 * obstacleLengthMargin
    obstacleSafeWidth = obstacleWidth + 2 * obstacleWidthMargin
    obstacleSafeRadius = np.sqrt(obstacleSafeLength ** 2 + obstacleSafeWidth ** 2) / 2

if len(obstacleSafeRadius) > 0:
    safeDistance = V0*N*T + max(obstacleSafeRadius)
else:
    safeDistance = V0*N*T


# Detection Window
detectionWindowParam = {'L': safeDistance, 'W': max(obstacleWidth)*1.10}

# ------------------------------------------------------------

print('Add obstacle rotation')
print('Note: obstacleData.py is currently rotating obstacle')

if ns == 4:
    # problem size
    nx = 4
    nu = 2

    ncons_option = 2

    # if ncons_option == 1:
    #     ncons = 2*N + 4 # (option 1 in nlp.py) running + lataccel + V0 + terminal constraint-y + terminal constraint-V
    #
    # elif ncons_option == 2:
    #     ncons = 2*N + 3 # (option 2 in nlp.py) running + lataccel + terminal constraint-y + terminal constraint-V
    #
    # elif ncons_option == 3:
    #     ncons = 2*N + 2  # (option 3 in nlp.py) running + lataccel + terminal constraint-y

    if ncons_option == 1:
        ncons = 4  # (option 1 in nlp.py) lataccel + V0 + terminal constraint-V + N terminal delChi

    elif ncons_option == 2:
        ncons = 3  # (option 2 in nlp.py) lataccel + terminal constraint-V + terminal delChi

    elif ncons_option == 3:
        ncons = 2  # (option 3 in nlp.py) lataccel + terminal delChi


    t0 = 0
    u0 = np.zeros([N, nu])
    # mpciterations = int(18*N/(6))

    # nlpData
    nlpPrintLevel = 0

    # State and Control indices
    idx_E = 0
    idx_N = 1
    idx_V = 2
    idx_Chi = 3

    idx_Vdot = 0
    idx_Chidot = 1


elif ns == 6:
    # problem size
    nx = 6
    nu = 2

    ncons_option = 2

    # if ncons_option == 1:
    #     ncons = 2*N + 4 # (option 1 in nlp.py) running + lataccel + V0 + terminal constraint-y + terminal constraint-V
    #
    # elif ncons_option == 2:
    #     ncons = 2*N + 3 # (option 2 in nlp.py) running + lataccel + terminal constraint-y + terminal constraint-V
    #
    # elif ncons_option == 3:
    #     ncons = 2*N + 2  # (option 3 in nlp.py) running + lataccel + terminal constraint-y

    if ncons_option == 1:
        ncons = 4 # (option 1 in nlp.py) lataccel + V0 + terminal constraint-V + terminal delChi

    elif ncons_option == 2:
        ncons = 3 # (option 2 in nlp.py) lataccel + terminal constraint-V + terminal delChi

    elif ncons_option == 3:
        ncons = 2  # (option 3 in nlp.py) lataccel + terminal delChi


    t0 = 0
    u0 = np.zeros([N,nu])

    # nlpData
    nlpPrintLevel = 0

    # State and Control indices
    idx_E = 0
    idx_N = 1
    idx_V = 2
    idx_Chi = 3
    idx_Vdot = 4
    idx_Chidot = 5

    idx_Vddot = 0
    idx_Chiddot = 1

else:
    print("Error in ns")

if obstaclePresent:
    nObstacle = len(obstacleN)
else:
    nObstacle = 0

# -------------------------------------------------------
# Save settings in file

fileName_problemData = 'settings.txt'
f_problemData = open(fileName_problemData, 'w')

if ns == 4:

    f_problemData.write("%d %.2f %d %d %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n" % (
        N, T, ns, no,
        lb_VdotVal, ub_VdotVal,
        lb_ChidotVal, ub_ChidotVal,
        delChi_max_InView, lataccel_maxVal,
        lb_VTerm, ub_VTerm, V_cmd
        ))
elif ns == 6:

    f_problemData.write("%d %.2f %d %d %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n" % (
        N, T, ns, no,
        lb_VddotVal, ub_VddotVal,
        lb_ChiddotVal, ub_ChiddotVal,
        delChi_max_InView, lataccel_maxVal,
        lb_VTerm, ub_VTerm, V_cmd
        ))

f_problemData.close()

rundate = datetime.datetime.now().strftime("%Y-%m-%d")

rundir = './run_' + rundate + '/'
distutils.dir_util.mkpath(rundir)

suffix_Popup_NoPopup = '_NoPopup'

if N < 10:
    suffix = '_N0' + str(N) + '_Tp' + str(int(10 * T)) + '_ns' + str(ns) + '_no' + str(no)
else:
    suffix = '_N' + str(N) + '_Tp' + str(int(10 * T)) + '_ns' + str(ns) + '_no' + str(no)

suffix = suffix + suffix_Popup_NoPopup

dst_file = rundir + 'settings' + suffix + '.txt'
shutil.copyfile('settings.txt', dst_file)


