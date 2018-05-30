from path import *
from nmpc import *
from obstacleData import *
import problemData as pdata
import probInfo
import printPlots
import time, datetime
import shutil, distutils.dir_util
import os, os.path
import globalVars

# -------------------------------------------------------------------
# Main.py lets the user run different test cases for Model
# Predictive Control (MPC) based trajectory generation
#
# The user needs to select 3 items:
# 1. Test Case Type: Road shape and orientation
# 2. Experiment Number: Sets various design parameters for MPC
# 3. Number of States: Sets the vehicles states and control states
#
# Edit problemData.py to run different experiments
#
# 01/25/2018
# -------------------------------------------------------------------

print('increase velocity weight to get smoother plots')

# Path data
pathClass = pathInfo('default', startPoint, endPoint)
path = pathClass()
pathType = 'default'

# Obstacle data (static)
obstacleClass = obstacleInfo(obstaclePresent, obstacleE, obstacleN, obstacleChi, obstacleWidth, obstacleLength,
                             obstacleSafeWidth, obstacleSafeLength, obstacleSafeRadius)
obstacle = obstacleClass()

# Storage data
t = np.zeros([mpciterations, 1])
x = np.zeros([mpciterations, nx])
u = np.zeros([mpciterations, nu])

# Iteration data
x0[3] = np.pi/2 - path.pathData.Theta[0]  # align vehicle heading with road heading
tmeasure = t0
xmeasure = x0
u_new = np.zeros([1,nu])
mpciter = 0

# Other parameters
t_slowDown = []
t_slowDown_detected = False
Dist_stop = 0.0
T_stop = 0.0
fVbnd = False

# Print to File
writeToFile = True
if writeToFile == True:
    fileName = 'logFile.txt'
    fHandle = open(fileName, 'w')
else:
    fHandle = -1
    fileName = ''

debug = True
if debug == True:
    globalVars.writeToFileCost = True
    fileNameCost = 'logFileCost.txt'
    if os.path.isfile(fileNameCost) == True:
        os.remove('logFileCost.txt') # remove previous file
    fHandleCost = open(fileNameCost, 'a') # file to append cost (see nlp.py > objective function)
else:
    fHandleCost = -1
    fileNameCost = ''

# Initialize storage arrays
tElapsed = np.zeros(mpciterations)
VTerminal = np.zeros(mpciterations)
latAccel = np.zeros(mpciterations)
delChi = np.zeros(mpciterations)
breakLoop = False

# Speficy initial position index
posIdx = getPosIdx(x0[0], x0[1], path, posIdx0)

# Specify Booleans
saveData = True
plotData = True
trimVals = True

# Create array of paths
pathObj = makePathObj(pdata, path, obstacle)
pathObjArray = [pathObj]


# Main loop
while mpciter < mpciterations:

    # start time keeping
    tStart = time.time()

    #  get new initial value
    t0, x0 = measureInitialValue(tmeasure, xmeasure)

    # search for obstacle
    detected = detectObstacle(x0, detectionWindowParam, obstacle)

    if detected is True:
        delChi_max = delChi_max_InView
    else:
        delChi_max = delChi_max_NotInView

    # solve optimal control problem
    u_new, info = solveOptimalControlProblem(N, t0, x0, u0, T, ncons, nu, path,
                                             obstacle, posIdx, ncons_option, V_cmd,
                                             lb_VTerm, lb_VdotVal, fHandleCost)
    tElapsed[mpciter] = (time.time() - tStart)

    # mpc  future path plot
    latAccel[mpciter], VTerminal[mpciter], delChi[mpciter] = printPlots.nmpcPlotSol(u_new, path, x0,
                                                                                    obstacle, pathType, mpciter)

    # solution information
    printPlots.nmpcPrint(mpciter, info, N, x0, u_new, writeToFile, fHandle, tElapsed[mpciter],
                        latAccel[mpciter], VTerminal[mpciter], delChi[mpciter])

    # store closed loop data
    t[mpciter] = tmeasure
    for k in range(nx):
        x[mpciter, k] = xmeasure[k]
    for j in range(nu):
        u[mpciter, j] = u_new[0,j]

    # change flag (global variable) to write cost breakdown in nlp.py
    if debug is True:
        writeToFileCost = True

    # apply control
    tmeasure, xmeasure = applyControl(T, t0, x0, u_new)

    # prepare restart
    u0 = shiftHorizon(N, u_new)

    posIdx = getPosIdx(xmeasure[0], xmeasure[1], path, posIdx)

    # reset global variable to write cost breakdown in nlp.py
    globalVars.writeToFileCost = True

    x_mpciter = probInfo.computeOpenloopSolution(u0.flatten(1), N, T, t0, x0)
    current_point = x_mpciter[0, 0:2]
    terminal_point = x_mpciter[-1, 0:2]

    # stop vehicle if required
    breakLoop, V_cmd, t_slowDown, t_slowDown_detected, lb_VTerm, lb_VdotVal = \
        vehicleStop(T, x, mpciter, decelType, terminal_point, endPoint,
                lb_reachedGoal, lb_reachedNearGoal, zeroDistanceChange,
                t_slowDown_detected, tmeasure, V_cmd, lb_VTermSlowDown, lb_VdotValSlowDown, decel,
                t_slowDown, lb_VTerm, lb_VdotVal)

    if breakLoop == True:
        break

    if mpciter > 3:
        None

    # next iteration
    mpciter = mpciter + 1


if trimVals is True:
    mpciterations = mpciter
    tElapsed = tElapsed[0:mpciterations]
    VTerminal = VTerminal[0:mpciterations]
    latAccel = latAccel[0:mpciterations]
    delChi = delChi[0:mpciterations]

    t = t[0:mpciterations, :]
    x = x[0:mpciterations, :]
    u = u[0:mpciterations, :]


# close log file
if writeToFile == True:
    fHandle.close()

if debug == True:
    fHandleCost.close()

# start preparing for generating plots
rundate = datetime.datetime.now().strftime("%Y-%m-%d")
rundir = './run_' + rundate + '/'
if N < 10:
    suffix = '_N0' + str(N) + '_Tp' + str(int(10 * T)) + '_ns' + str(ns) + '_no' + str(no)
else:
    suffix = '_N' + str(N) + '_Tp' + str(int(10 * T)) + '_ns' + str(ns) + '_no' + str(no)


suffix = suffix + suffix_Popup_NoPopup # suffix_Popup_NoPopup = '_NoPopup' set in problemData.py

# start preparation for saving plots if required
if saveData == True:

    distutils.dir_util.mkpath(rundir)
    dst_file = rundir + 'logFile' + suffix + '.txt'
    shutil.copyfile('logFile.txt', dst_file)

    # figure 1: path
    dst_fig = rundir + 'path' + suffix + '_Before.png'
    fig = plt.figure(1)
    plt.pause(0.01)
    fig.savefig(dst_fig)

    obstacleDict = obstacleDict_from_ClassInstance(obstacle)
    file_pkl = rundir + 'pathDict_no' + str(no) + suffix_Popup_NoPopup + '.pkl'
    savepkl((pathObjArray, obstacleDict), file_pkl)

    file_pkl2 = rundir + 'pathDict_no' + str(no) + suffix_Popup_NoPopup + '_b.pkl'
    savepkl(obstacle, file_pkl2)

    print('saved data and figure')


# create plots
oldpwd = os.getcwd()
os.chdir(rundir)
settingsFile = 'settings' + suffix + '.txt'
figno = printPlots.nmpcPlot(t, x, u, path, obstacle, tElapsed, VTerminal, latAccel,
                            delChi, settingsFile, pathObjArray, t_slowDown)
os.chdir(oldpwd)

if saveData == True:

    # figure 2: E, N
    dst_fig = rundir + 'E-N' + suffix + '.png'
    fig = plt.figure(2)
    plt.pause(0.01)
    fig.savefig(dst_fig)

    # figure 3: V, Vdot
    dst_fig = rundir + 'V-Vdot' + suffix + '.png'
    fig = plt.figure(3)
    plt.pause(0.01)
    fig.savefig(dst_fig)

    # figure 4: Chi, Chidot
    dst_fig = rundir + 'Chi-Chidot' + suffix + '.png'
    fig = plt.figure(4)
    plt.pause(0.01)
    fig.savefig(dst_fig)

    # figure 5: LatAccel, dy
    dst_fig = rundir + 'LatAccel-dy' + suffix + '.png'
    fig = plt.figure(6)
    plt.pause(0.01)
    fig.savefig(dst_fig)

    if ns == 6:
        # figure 6: V, Vdot
        dst_fig = rundir + 'Vddot-Chiddot' + suffix + '.png'
        fig = plt.figure(5)
        plt.pause(0.01)
        fig.savefig(dst_fig)

    # figure 7: CPU time
    dst_fig = rundir + 'CPUtime' + suffix + '.png'
    fig = plt.figure(7)
    plt.pause(0.01)
    fig.savefig(dst_fig)

    # figure 8: V-terminal
    dst_fig = rundir + 'V-terminal' + suffix + '.png'
    fig = plt.figure(8)
    plt.pause(0.01)
    fig.savefig(dst_fig)

    # figure 9: path
    dst_fig = rundir + 'path' + suffix + '_After.png'
    fig = plt.figure(9)
    plt.pause(0.01)
    fig.savefig(dst_fig)


print('done!')
plt.show()


# Save Data
# answer =  raw_input('Save Figures and Data [y/n]:  ')
# if answer == 'y':
#     dirname = raw_input('Enter Folder Name: ')
#     printPlots.savePlots(dirname, figno)



# -------------------------------------------------------------------

