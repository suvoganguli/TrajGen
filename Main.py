def Main():
    import pathMain
    import nmpc
    import obstacleData
    import numpy as np

    import problemData as pdata
    import probInfo
    import printPlots
    import time
    import shutil, distutils.dir_util
    import os.path
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

    # Path data
    pathClass = pathMain.pathInfo('default', pathMain.startPoint, pathMain.endPoint)
    path = pathClass()
    pathType = 'default'

    # Obstacle data (static)
    obstacleClass = obstacleData.obstacleInfo(pathMain.obstaclePresent, pathMain.obstacleE, pathMain.obstacleN, pathMain.obstacleChi, pathMain.obstacleWidth, pathMain.obstacleLength,
                                              pathMain.obstacleSafeWidth, pathMain.obstacleSafeLength, pathMain.obstacleSafeRadius)
    obstacle = obstacleClass()

    # Storage data
    t = np.zeros([pathMain.mpciterations, 1])
    x = np.zeros([pathMain.mpciterations, pathMain.nx])
    u = np.zeros([pathMain.mpciterations, pathMain.nu])

    # Iteration data
    pathMain.x0[3] = np.pi / 2 - path.pathData.Theta[0]  # align vehicle heading with road heading
    tmeasure = pathMain.t0
    xmeasure = pathMain.x0
    u_new = np.zeros([1, pathMain.nu])
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
        globalVars.writeToFileCost = True
        fileNameCost = 'logFileCost.txt'
        if os.path.isfile(fileNameCost) == True:
            os.remove('logFileCost.txt') # remove previous file
        fHandleCost = open(fileNameCost, 'a') # file to append cost (see nlp.py > objective function)
    else:
        fHandleCost = -1
        fileNameCost = ''

    # Initialize storage arrays
    tElapsed = np.zeros(pathMain.mpciterations)
    VTerminal = np.zeros(pathMain.mpciterations)
    latAccel = np.zeros(pathMain.mpciterations)
    delChi = np.zeros(pathMain.mpciterations)
    breakLoop = False

    # Speficy initial position index
    posIdx = obstacleData.getPosIdx(pathMain.x0[0], pathMain.x0[1], path, pathMain.posIdx0)

    # Specify Booleans
    saveData = True
    plotData = True
    trimVals = True

    # Create array of paths
    pathObj = obstacleData.makePathObj(pdata, path, obstacle)
    pathObjArray = [pathObj]


    # Main loop
    while mpciter < pathMain.mpciterations:

        # start time keeping
        tStart = time.time()

        #  get new initial value
        t0, x0 = nmpc.measureInitialValue(tmeasure, xmeasure)

        # search for obstacle
        detected, obstacleID = obstacleData.detectObstacle(x0, pathMain.detectionWindowParam, obstacle)

        if detected == True:
            delChi_max = pathMain.delChi_max_InView
            delChi_maxvec_obstacleInView = \
                obstacleData.np.concatenate([delChi_maxvec_obstacleInView, obstacleData.np.array([pathMain.delChi_max_InView])])
            delChi_maxvec_obstacleNotInView = \
                obstacleData.np.concatenate([delChi_maxvec_obstacleNotInView, obstacleData.np.array([0])])

            print('Obstacle(s) detected at mpciter = ' + str(mpciter))
        else:
            delChi_max = pathMain.delChi_max_NotInView
            delChi_maxvec_obstacleNotInView = \
                obstacleData.np.concatenate([delChi_maxvec_obstacleNotInView, obstacleData.np.array([pathMain.delChi_max_NotInView])])
            delChi_maxvec_obstacleInView = \
                obstacleData.np.concatenate([delChi_maxvec_obstacleInView, obstacleData.np.array([0])])
            print('No obstacle detected at mpciter = ' + str(mpciter))

        # solve optimal control problem
        u_new, info = nmpc.solveOptimalControlProblem(pathMain.N, t0, x0, pathMain.u0, pathMain.T, pathMain.ncons, pathMain.nu, path,
                                                      obstacle, posIdx, pathMain.ncons_option, pathMain.V_cmd,
                                                      pathMain.lb_VTerm, pathMain.lb_VdotVal, delChi_max, obstacleID, fHandleCost)
        tElapsed[mpciter] = (time.time() - tStart)

        # mpc  future path plot
        latAccel[mpciter], VTerminal[mpciter], delChi[mpciter] = printPlots.nmpcPlotSol(u_new, path, x0,
                                                                                        obstacle, pathType, mpciter)

        # solution information
        printPlots.nmpcPrint(mpciter, info, pathMain.N, x0, u_new, writeToFile, fHandle, tElapsed[mpciter],
                             latAccel[mpciter], VTerminal[mpciter], delChi[mpciter])

        # store closed loop data
        t[mpciter] = tmeasure
        for k in range(pathMain.nx):
            x[mpciter, k] = xmeasure[k]
        for j in range(pathMain.nu):
            u[mpciter, j] = u_new[0,j]

        # change flag (global variable) to write cost breakdown in nlp.py
        if debug is True:
            writeToFileCost = True

        # apply control
        tmeasure, xmeasure = nmpc.applyControl(pathMain.T, t0, x0, u_new)

        # prepare restart
        u0 = nmpc.shiftHorizon(pathMain.N, u_new)

        posIdx = obstacleData.getPosIdx(xmeasure[0], xmeasure[1], path, posIdx)

        # reset global variable to write cost breakdown in nlp.py
        globalVars.writeToFileCost = True

        x_mpciter = probInfo.computeOpenloopSolution(u0.flatten(1), pathMain.N, pathMain.T, t0, x0)
        current_point = x_mpciter[0, 0:2]
        terminal_point = x_mpciter[-1, 0:2]

        # stop vehicle if required
        breakLoop, V_cmd, t_slowDown, t_slowDown_detected, lb_VTerm, lb_VdotVal = \
            obstacleData.vehicleStop(pathMain.T, x, mpciter, pathMain.decelType, terminal_point, pathMain.endPoint,
                                     pathMain.lb_reachedGoal, pathMain.lb_reachedNearGoal, pathMain.zeroDistanceChange,
                                     t_slowDown_detected, tmeasure, pathMain.V_cmd, pathMain.lb_VTermSlowDown, pathMain.lb_VdotValSlowDown, pathMain.decel,
                                     t_slowDown, pathMain.lb_VTerm, pathMain.lb_VdotVal)

        if breakLoop == True:
            break

        if mpciter > 30:
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
    #rundate = datetime.datetime.now().strftime("%Y-%m-%d")
    #rundir = './run_' + rundate + '/'
    if pathMain.N < 10:
        suffix = '_N0' + str(pathMain.N) + '_Tp' + str(int(10 * pathMain.T)) + '_ns' + str(pathMain.ns) + '_no' + str(pathMain.no)
    else:
        suffix = '_N' + str(pathMain.N) + '_Tp' + str(int(10 * pathMain.T)) + '_ns' + str(pathMain.ns) + '_no' + str(pathMain.no)


    suffix = suffix + pathMain.suffix_Popup_NoPopup # suffix_Popup_NoPopup = '_NoPopup' set in problemData.py

    # start preparation for saving plots if required
    if saveData == True:

        distutils.dir_util.mkpath(pdata.rundir)
        dst_file = pdata.rundir + 'logFile' + suffix + '.txt'
        shutil.copyfile('logFile.txt', dst_file)

        # figure 1: path
        dst_fig = pdata.rundir + 'path' + suffix + '_Before.png'
        fig = pathMain.plt.figure(1)
        pathMain.plt.pause(0.01)
        fig.savefig(dst_fig)

        obstacleDict = obstacleData.obstacleDict_from_ClassInstance(obstacle)
        file_pkl = pdata.rundir + 'pathDict_no' + str(pathMain.no) + pathMain.suffix_Popup_NoPopup + '.pkl'
        obstacleData.savepkl((pathObjArray, obstacleDict), file_pkl)

        file_pkl2 = pdata.rundir + 'pathDict_no' + str(pathMain.no) + pathMain.suffix_Popup_NoPopup + '_b.pkl'
        obstacleData.savepkl(obstacle, file_pkl2)

        print('saved data and figure')


    # create plots
    oldpwd = os.getcwd()
    os.chdir(pdata.rundir)
    settingsFile = 'settings' + suffix + '.txt'
    figno = printPlots.nmpcPlot(t, x, u, path, obstacle, tElapsed, VTerminal, latAccel,
                                delChi, settingsFile, pathObjArray, t_slowDown,
                                delChi_maxvec_obstacleInView, delChi_maxvec_obstacleNotInView)
    os.chdir(oldpwd)

    if saveData == True:

        # figure 2: E, N
        dst_fig = pdata.rundir + 'E-N' + suffix + '.png'
        fig = pathMain.plt.figure(2)
        pathMain.plt.pause(0.01)
        fig.savefig(dst_fig)

        # figure 3: V, Vdot
        dst_fig = pdata.rundir + 'V-Vdot' + suffix + '.png'
        fig = pathMain.plt.figure(3)
        pathMain.plt.pause(0.01)
        fig.savefig(dst_fig)

        # figure 4: Chi, Chidot
        dst_fig = pdata.rundir + 'Chi-Chidot' + suffix + '.png'
        fig = pathMain.plt.figure(4)
        pathMain.plt.pause(0.01)
        fig.savefig(dst_fig)

        # figure 5: LatAccel, dy
        dst_fig = pdata.rundir + 'LatAccel-dy' + suffix + '.png'
        fig = pathMain.plt.figure(6)
        pathMain.plt.pause(0.01)
        fig.savefig(dst_fig)

        if pathMain.ns == 6:
            # figure 6: V, Vdot
            dst_fig = pdata.rundir + 'Vddot-Chiddot' + suffix + '.png'
            fig = pathMain.plt.figure(5)
            pathMain.plt.pause(0.01)
            fig.savefig(dst_fig)

        # figure 7: CPU time
        dst_fig = pdata.rundir + 'CPUtime' + suffix + '.png'
        fig = pathMain.plt.figure(7)
        pathMain.plt.pause(0.01)
        fig.savefig(dst_fig)

        # figure 8: V-terminal
        dst_fig = pdata.rundir + 'V-terminal' + suffix + '.png'
        fig = pathMain.plt.figure(8)
        pathMain.plt.pause(0.01)
        fig.savefig(dst_fig)

        # figure 9: path
        dst_fig = pdata.rundir + 'path' + suffix + '_After.png'
        fig = pathMain.plt.figure(9)
        pathMain.plt.pause(0.01)
        fig.savefig(dst_fig)


    print('done!')
    pathMain.plt.show()

# -------------------------------------------------------------------

nRun = 3
for _ in range(nRun):
    Main()

