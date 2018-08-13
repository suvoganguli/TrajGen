import pathMain
import nmpc
import obstacleData
import numpy as np
import matplotlib.pyplot as plt
import problemData as pdata
import probInfo
import printPlots
import time, datetime
import shutil, distutils.dir_util
import os.path
import globalVars
import utils
import numpy as np

def Main(isBatch, showPlot, kRun=None, fBatchRun=None):

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

    # For saving data and figures
    #rundate = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    rundate = datetime.datetime.now().strftime("%Y-%m-%d")
    rundir = './run_' + rundate + '/'
    distutils.dir_util.mkpath(rundir)

    # Path data
    pathClass = pathMain.pathInfo('default', pdata.startPoint, pdata.endPoint)
    path = pathClass()
    pathType = 'default'

    # Obstacle data
    obstaclePresent, nObstacle, obstacleE, obstacleN, obstacleChi, obstacleLength, obstacleWidth, \
    obstacleSafeLength, obstacleSafeWidth, obstacleSafeRadius, safeDistance, detectionWindowParam = \
        obstacleData.createObstacleData(pdata.no, pdata.scaleFactorE, pdata.scaleFactorN,
                                        pdata.widthSpace, pdata.lengthSpace, pdata.horzDistance,
                                        rundate, rundir)

    obstacleClass = obstacleData.obstacleInfo(obstaclePresent, obstacleE, obstacleN, obstacleChi, obstacleWidth, obstacleLength,
                                              obstacleSafeWidth, obstacleSafeLength, obstacleSafeRadius)
    obstacle = obstacleClass()


    # Storage data
    t = np.zeros([pdata.mpciterations, 1])
    x = np.zeros([pdata.mpciterations, pdata.nx])
    u = np.zeros([pdata.mpciterations, pdata.nu])

    # Iteration data
    pdata.x0[3] = np.pi / 2 - path.pathData.Theta[0]  # align vehicle heading with road heading
    tmeasure = pdata.t0
    xmeasure = pdata.x0
    u_new = np.zeros([1, pdata.nu])
    mpciter = 0

    # Other parameters
    t_slowDown = []
    t_slowDown_detected = False
    delChi_maxvec_obstacleInView = np.array([], dtype=float)
    delChi_maxvec_obstacleNotInView = np.array([], dtype=float)
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
        fileNameCost = 'logFileCost.txt'
        if os.path.isfile(fileNameCost) == True:
            os.remove('logFileCost.txt') # remove previous file
        fHandleCost = open(fileNameCost, 'a') # file to append cost (see nlp.py > objective function)
        fileNameCostGrad = 'logFileCostGrad.txt'
        if os.path.isfile(fileNameCostGrad) == True:
            os.remove('logFileCostGrad.txt') # remove previous file
        fHandleCostGrad = open(fileNameCostGrad, 'a') # file to append cost (see nlp.py > objective function)

    else:
        fHandleCost = -1
        fHandleCostGrad = -1
        fileNameCost = ''
        fileNameCostGrad = ''

    # Initialize storage arrays
    tElapsed = np.zeros(pdata.mpciterations)
    VTerminal = np.zeros(pdata.mpciterations)
    latAccel = np.zeros(pdata.mpciterations)
    delChi = np.zeros(pdata.mpciterations)
    breakLoop1 = False

    # Speficy initial position index
    posIdx = obstacleData.getPosIdx(pdata.x0[0], pdata.x0[1], path, pdata.posIdx0)

    # Specify Booleans
    saveData = True
    plotData = True
    trimVals = True

    # Create array of paths
    pathObj = obstacleData.makePathObj(pdata, path, obstacle)
    pathObjArray = [pathObj]

    # Main loop
    while mpciter < pdata.mpciterations:

        # start time keeping
        tStart = time.time()

        #  get new initial value
        t0, x0 = nmpc.measureInitialValue(tmeasure, xmeasure)

        # search for obstacle
        detected, obstacleID = obstacleData.detectObstacle(x0, detectionWindowParam, obstacle)

        if detected == True:
            delChi_max = pdata.delChi_max_InView
            delChi_maxvec_obstacleInView = \
                obstacleData.np.concatenate([delChi_maxvec_obstacleInView, np.array([pdata.delChi_max_InView])])
            delChi_maxvec_obstacleNotInView = \
                obstacleData.np.concatenate([delChi_maxvec_obstacleNotInView, np.array([0])])

            #print('Obstacle(s) detected at mpciter = ' + str(mpciter))
        else:
            delChi_max = pdata.delChi_max_NotInView
            delChi_maxvec_obstacleNotInView = \
                obstacleData.np.concatenate([delChi_maxvec_obstacleNotInView, np.array([pdata.delChi_max_NotInView])])
            delChi_maxvec_obstacleInView = \
                obstacleData.np.concatenate([delChi_maxvec_obstacleInView, np.array([0])])
            #print('No obstacle detected at mpciter = ' + str(mpciter))

        # solve optimal control problem
        u_new, info = nmpc.solveOptimalControlProblem(pdata.N, t0, x0, pdata.u0, pdata.T, pdata.ncons, pdata.nu, path,
                                                      obstacle, posIdx, pdata.ncons_option, pdata.V_cmd,
                                                      pdata.lb_VTerm, pdata.lb_VdotVal, delChi_max, obstacleID, safeDistance,
                                                      debug, fHandleCost, fHandleCostGrad)
        tElapsed[mpciter] = (time.time() - tStart)

        # stop iteration if solution is not "solved" for "acceptable"
        if info['status'] > 1 or info['status'] < -1:
            breakLoop1 = True

            # write the batch run number where solution was not obtained
            if isBatch:
                fBatchRun.write("%d %d\n" %(kBatchRun, info['status']))

        # mpc  future path plot
        latAccel[mpciter], VTerminal[mpciter], delChi[mpciter] = printPlots.nmpcPlotSol(u_new, path, x0,
                                                                                        obstacle, pathType, mpciter, detectionWindowParam)

        # solution information
        printPlots.nmpcPrint(mpciter, info, pdata.N, x0, u_new, writeToFile, fHandle, tElapsed[mpciter],
                             latAccel[mpciter], VTerminal[mpciter], delChi[mpciter])

        # store closed loop data
        t[mpciter] = tmeasure
        for k in range(pdata.nx):
            x[mpciter, k] = xmeasure[k]
        for j in range(pdata.nu):
            u[mpciter, j] = u_new[0,j]

        # apply control
        tmeasure, xmeasure = nmpc.applyControl(pdata.T, t0, x0, u_new)

        # prepare restart
        u0 = nmpc.shiftHorizon(pdata.N, u_new)

        posIdx = obstacleData.getPosIdx(xmeasure[0], xmeasure[1], path, posIdx)

        x_mpciter = probInfo.computeOpenloopSolution(u0.flatten(1), pdata.N, pdata.T, t0, x0)
        current_point = x_mpciter[0, 0:2]
        terminal_point = x_mpciter[-1, 0:2]

        # stop vehicle if required
        breakLoop2, V_cmd, t_slowDown, t_slowDown_detected, lb_VTerm, lb_VdotVal = \
            utils.vehicleStop(pdata.T, x, mpciter, pdata.decelType, terminal_point, pdata.endPoint,
                                     pdata.lb_reachedGoal, pdata.lb_reachedNearGoal, pdata.zeroDistanceChange,
                                     t_slowDown_detected, tmeasure, pdata.V_cmd, pdata.lb_VTermSlowDown, pdata.lb_VdotValSlowDown, pdata.decel,
                                     t_slowDown, pdata.lb_VTerm, pdata.lb_VdotVal)

        # break loop is solution not obtained or vehicle stops
        if (breakLoop1 == True) or (breakLoop2 == True):
            break

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
        fHandleCostGrad.close()

    # start preparing for generating plots
    #rundate = datetime.datetime.now().strftime("%Y-%m-%d")
    #rundir = './run_' + rundate + '/'
    if pdata.N < 10:
        suffix = '_N0' + str(pdata.N) + '_Tp' + str(int(10 * pdata.T)) + '_ns' + str(pdata.ns) + '_no' + str(pdata.no)
    else:
        suffix = '_N' + str(pdata.N) + '_Tp' + str(int(10 * pdata.T)) + '_ns' + str(pdata.ns) + '_no' + str(pdata.no)


    #suffix = suffix + pdata.suffix_Popup_NoPopup # suffix_Popup_NoPopup = '_NoPopup' set in problemData.py

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

        obstacleDict = obstacleData.obstacleDict_from_ClassInstance(obstacle)
        file_pkl = rundir + 'pathDict_no' + str(pdata.no) + '.pkl'
        obstacleData.savepkl((pathObjArray, obstacleDict), file_pkl)

        file_pkl2 = rundir + 'pathDict_no' + str(pdata.no) + '_b.pkl'
        obstacleData.savepkl(obstacle, file_pkl2)

        print('saved data and figure')


    # create plots
    oldpwd = os.getcwd()
    os.chdir(rundir)
    settingsFile = 'settings' + suffix + '.txt'
    figno = printPlots.nmpcPlot(t, x, u, path, obstacle, tElapsed, VTerminal, latAccel,
                                delChi, settingsFile, pathObjArray, t_slowDown,
                                delChi_maxvec_obstacleInView, delChi_maxvec_obstacleNotInView)
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

        if pdata.ns == 6:
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

    # show plots for single run, close plots for batch run
    if showPlot:
        plt.show()
    else:
        plt.pause(2)
        plt.close("all")

# -------------------------------------------------------------------
# Run main file
# -------------------------------------------------------------------

opt = 2

if opt == 1:
    # Batch run for random objects
    # Set 'no = -1' in problemData.py

    batchRunDate = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M")
    batchFileName = 'batchRun' + batchRunDate + '.txt'
    fBatchRun = open(batchFileName ,'w')
    fBatchRun.write('The following run(s) had problems:\n')
    nBatchRun = 2
    isBatch = True
    showPlot = False

    for kBatchRun in range(nBatchRun):
        Main(isBatch, showPlot, kBatchRun, fBatchRun)

    fBatchRun.close()
else:
    # Edit 'no' in problemData.py to >= 0 (specific 'no' available,
    # see function createObstacleData in obstacleData.py)

    isBatch = False
    showPlot = True
    Main(isBatch, showPlot)

