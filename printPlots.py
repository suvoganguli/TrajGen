import numpy as np
from probInfo import *
import matplotlib.pyplot as plt
import matplotlib.figure as fig
import matplotlib.patches as patches
import matplotlib.animation as animation
import matplotlib.patches as patches
import problemData as pdata
import os
from utils import *

# Axis:
# *X, *Y = E [ft], N [ft], theta [rad] (theta is w.r.t +E axis)


def nmpcPlotSol(u_new,path,mpciter,x0,obstacle,case):

    u_mpciter = u_new.flatten(1)
    x_mpciter = computeOpenloopSolution(u_mpciter, N, T, t0, x0)
    East = x_mpciter[:,0]
    North = x_mpciter[:,1]

    # figure 1
    f1 = plt.figure(1,figsize=(5, 7), dpi=100)
    plt.ylabel('N [ft]')
    plt.xlabel('E [ft]')
    #plt.axis('equal')
    None

    if mpciter == 0:

        # Detailed Path
        plt.plot(path.pathData.E, path.pathData.N, linestyle='--', color='c')

        # Laplacian Path
        #plt.plot(path.pathData.pathLaplacian[0,:], path.pathData.pathLaplacian[1,:], linestyle='--', color='k')

        #plt.plot(path.pathData.PathLeftBoundaryE, path.pathData.PathLeftBoundaryN, linestyle='-', color='k')
        #plt.plot(path.pathData.PathRightBoundaryE, path.pathData.PathRightBoundaryN, linestyle='-', color='k')

        plt.plot(path.pathData.PathStartPoint[0], path.pathData.PathStartPoint[1], marker='o', markersize=8, color='r')
        plt.plot(path.pathData.PathEndPoint[0], path.pathData.PathEndPoint[1], marker='o', markersize=8, color='g')

        if True:
            plt.plot(path.pathData.PathRightEndPointsE, path.pathData.PathRightEndPointsN,'m+')
            plt.plot(path.pathData.PathLeftEndPointsE, path.pathData.PathLeftEndPointsN,'m+')

            x1 = path.pathData.PathRightEndPointsE
            x2 = path.pathData.PathLeftEndPointsE
            y1 = path.pathData.PathRightEndPointsN
            y2 = path.pathData.PathLeftEndPointsN
            plt.plot(x1, y1, 'm', x2, y2, 'm')

            x1 = path.pathData.PathCenterEndPointsE - pdata.delta_yRoad*np.sin(path.pathData.Theta_endpoints)
            x2 = path.pathData.PathCenterEndPointsE + pdata.delta_yRoad*np.sin(path.pathData.Theta_endpoints)
            y1 = path.pathData.PathCenterEndPointsN + pdata.delta_yRoad*np.cos(path.pathData.Theta_endpoints)
            y2 = path.pathData.PathCenterEndPointsN - pdata.delta_yRoad*np.cos(path.pathData.Theta_endpoints)
            plt.plot(x1, y1, 'r', x2, y2, 'r')

            #for i in range(len(path.pathData.LaneRightEndPointsX)):
            #    x1 = path.pathData.LaneRightEndPointsX[i]
            #    y1 = path.pathData.LaneRightEndPointsY[i]
            #    x2 = path.pathData.LaneLeftEndPointsX[i]
            #    y2 = path.pathData.LaneLeftEndPointsY[i]
            #    plt.plot([x1, x2], [y1, y2], 'm')

        ax1 = f1.gca()
        ax1.grid(True)

        if True: # obstacle.Present == True:

            nObs = len(obstacle.E)
            if nObs > 0:
                for k in range(nObs):

                    Efc = obstacle.E[k] + pathWidth/2
                    Nfc = obstacle.N[k]
                    W = obstacle.w[k] - pathWidth
                    L = obstacle.l[k]
                    Theta = obstacle.Chi[k]
                    fc = "red"
                    polygon_obstacle = getPatch(Efc, Nfc, W, L, Theta, fc)


                    Efc = obstacle.E[k]
                    Nfc = obstacle.N[k]
                    W = obstacle.w[k]
                    L = obstacle.l[k]
                    Theta = obstacle.Chi[k]
                    fc = "green"
                    polygon_safezone = getPatch(Efc, Nfc, W, L, Theta, fc)

                    ax1.add_patch(polygon_safezone)
                    ax1.add_patch(polygon_obstacle)

    plt.figure(f1.number)
    nEN = len(East)
    plt.plot(East[0:nEN], North[0:nEN], marker='x', markersize=4, color='b')
    plt.plot(East[0], North[0], marker='o', markersize=4, color='r')
    plt.xlim([0, 16])
    plt.ylim([0, 128])
    #ax1.set_xlim([0, 16])
    #ax1.set_ylim([0, 128])

    plt.pause(0.01)
    #if mpciter < mpciterations-1:
    #   ax1 = f1.gca()
    #   del ax1.lines[7:12]

    None


def nmpcPlot(t,x,u,path,obstacle,tElapsed,case):


    if ns == 6:

        figno = np.zeros(7)

        # figure 2
        f, ax = plt.subplots(2)
        figno[0] =  plt.gcf().number
        ax[0].plot(t, x[:,[0]])  # E
        ax[1].plot(t, x[:,[1]])  # N
        ax[0].set_ylabel('E [ft]')
        ax[1].set_ylabel('N [ft]')
        ax[1].set_xlabel('t [sec]')
        for k in range(2):
            ax[k].grid(True)

        # figure 3
        f, ax = plt.subplots(2)
        figno[1] = plt.gcf().number
        ax[0].plot(t, x[:,[2]])  # V
        ax[0].plot(t, lb_V*np.ones(t.shape),linestyle='--', color='g')
        ax[0].plot(t, ub_V*np.ones(t.shape), linestyle='--', color='g')

        ax[1].plot(t, x[:,[4]])  # Vdot
        ax[0].set_ylabel('V [fps]')
        ax[1].set_ylabel('Vdot [fps2]')
        ax[1].set_xlabel('t [sec]')
        for k in range(2):
            ax[k].grid(True)

        # figure 4
        f, ax = plt.subplots(2)
        figno[2] = plt.gcf().number
        ax[0].plot(t, x[:,[3]]*180/np.pi)
        ax[1].plot(t, x[:,[5]]*180/np.pi)
        ax[0].set_ylabel('Chi [deg]')
        ax[1].set_ylabel('Chidot [deg/s]')
        ax[1].set_xlabel('t [sec]')
        for k in range(2):
            ax[k].grid(True)

        # figure 5
        f, ax = plt.subplots(2)
        figno[3] = plt.gcf().number
        ax[0].plot(t, u[:,0])
        ax[0].plot(t, lb_VddotVal*np.ones(t.shape),linestyle='--', color='r')
        ax[0].plot(t, ub_VddotVal*np.ones(t.shape), linestyle='--', color='r')

        ax[1].plot(t, u[:,1]*180/np.pi)
        ax[1].plot(t, lb_ChiddotVal*np.ones(t.shape)*180/np.pi,linestyle='--', color='r')
        ax[1].plot(t, ub_ChiddotVal*np.ones(t.shape)*180/np.pi, linestyle='--', color='r')

        ax[0].set_ylabel('Vddot [fps3]')
        ax[1].set_ylabel('Chiddot [deg/s2]')
        ax[1].set_xlabel('t [sec]')
        for k in range(2):
            ax[k].grid(True)

        # figure 6
        f, ax = plt.subplots(1)
        figno[4] = plt.gcf().number
        plt.plot(t, x[:,[2]] * x[:,[5]] / 32.2)  # V*Chidot
        ax.set_ylabel('Lat Accel [g]')
        ax.set_xlabel('t [sec]')
        ax.grid(True)

        # figure 7
        # figno[5] = plt.gcf().number
        # f, ax = plt.subplots(1, figsize=(5, 7), dpi=100)  #sharex=True
        # lw = 1.0
        # ax.plot(path.E, path.N, linewidth = lw, linestyle='--', color='k')
        # ax.plot(x[:,0],x[:,1], linestyle='-', color='b')
        # ax.set_ylabel('N [ft]')
        # ax.set_xlabel('E [ft]')
        # ax.grid(True)
        # plt.xlim([0, 16])
        # plt.ylim([0, 128])
        # #plt.axis('equal')
        #
        # plt.plot(path.pathData.PathStartPoint[0], path.pathData.PathStartPoint[1], marker='o', markersize=8, color='r')
        # plt.plot(path.pathData.PathEndPoint[0], path.pathData.PathEndPoint[1], marker='o', markersize=8, color='g')
        #
        # nObs = len(obstacle.E)
        # if nObs > 0:
        #     for k in range(nObs):
        #         Efc = obstacle.E[k] + pathWidth / 2
        #         Nfc = obstacle.N[k]
        #         W = obstacle.w[k] - pathWidth
        #         L = obstacle.l[k]
        #         Theta = obstacle.Chi[k]
        #         fc = "red"
        #         polygon_obstacle = getPatch(Efc, Nfc, W, L, Theta, fc)
        #
        #         Efc = obstacle.E[k]
        #         Nfc = obstacle.N[k]
        #         W = obstacle.w[k]
        #         L = obstacle.l[k]
        #         Theta = obstacle.Chi[k]
        #         fc = "green"
        #         polygon_safezone = getPatch(Efc, Nfc, W, L, Theta, fc)
        #
        #         ax.add_patch(polygon_safezone)
        #         ax.add_patch(polygon_obstacle)

    elif ns == 4:

        figno = np.zeros(6)

        # figure 2
        f, ax = plt.subplots(2)
        figno[0] = plt.gcf().number
        ax[0].plot(t, x[:, [0]])  # E
        ax[1].plot(t, x[:, [1]])  # N
        ax[0].set_ylabel('E [ft]')
        ax[1].set_ylabel('N [ft]')
        ax[1].set_xlabel('t [sec]')
        for k in range(2):
            ax[k].grid(True)

        # figure 3
        f, ax = plt.subplots(2)
        figno[1] = plt.gcf().number
        ax[0].plot(t, x[:, [2]])  # V
        ax[0].plot(t, lb_V*np.ones(t.shape),linestyle='--', color='g')
        ax[0].plot(t, ub_V*np.ones(t.shape), linestyle='--', color='g')
        ax[0].set_ylabel('V [fps]')

        ax[1].plot(t, u[:, [0]])  # Vdot
        ax[1].plot(t, lb_VdotVal*np.ones(t.shape),linestyle='--', color='r')
        ax[1].plot(t, ub_VdotVal*np.ones(t.shape), linestyle='--', color='r')

        ax[1].set_ylabel('Vdot [fps2]')
        ax[1].set_xlabel('t [sec]')
        for k in range(2):
            ax[k].grid(True)

        # figure 4
        f, ax = plt.subplots(2)
        figno[2] = plt.gcf().number
        ax[0].plot(t, x[:, [3]] * 180 / np.pi)

        ax[1].plot(t, u[:, [1]] * 180 / np.pi)
        ax[1].plot(t, lb_ChidotVal*np.ones(t.shape)*180/np.pi,linestyle='--', color='r')
        ax[1].plot(t, ub_ChidotVal*np.ones(t.shape)*180/np.pi, linestyle='--', color='r')

        ax[0].set_ylabel('Chi [deg]')
        ax[1].set_ylabel('Chidot [deg/s]')
        ax[1].set_xlabel('t [sec]')
        for k in range(2):
            ax[k].grid(True)


        # figure 5
        f, ax = plt.subplots(1)
        figno[3] = plt.gcf().number
        ax.plot(t, x[:, [2]] * u[:, [1]] / 32.2)  # V*Chidot
        if useLatAccelCons == 1:
            ax.plot(t, lataccel_maxVal*np.ones(t.shape)/32.2,linestyle='--', color='r')
            ax.plot(t, -lataccel_maxVal*np.ones(t.shape)/32.2, linestyle='--', color='r')

        ax.set_ylabel('Lat Accel [g]')
        ax.set_xlabel('t [sec]')
        ax.grid(True)

        # figure 6
        # figno[4] = plt.gcf().number
        # f, ax = plt.subplots(1, figsize=(5, 7), dpi=100)  # sharex=True
        # lw = 1.0
        # ax.plot(path.pathData.E, path.pathData.N, linewidth=lw, linestyle='-', color='k')
        # ax.plot(x[:, 0], x[:, 1], linestyle='-', color='b')
        # ax.set_ylabel('N [ft]')
        # ax.set_xlabel('E [ft]')
        # ax.grid(True)
        # plt.xlim([0, 16])
        # plt.ylim([0, 128])
        # # plt.axis('equal')
        #
        # plt.plot(path.pathData.PathStartPoint[0], path.pathData.PathStartPoint[1], marker='o', markersize=8, color='r')
        # plt.plot(path.pathData.PathEndPoint[0], path.pathData.PathEndPoint[1], marker='o', markersize=8, color='g')
        #
        # nObs = len(obstacle.E)
        # if nObs > 0:
        #     for k in range(nObs):
        #         Efc = obstacle.E[k] + pathWidth / 2
        #         Nfc = obstacle.N[k]
        #         W = obstacle.w[k] - pathWidth
        #         L = obstacle.l[k]
        #         Theta = obstacle.Chi[k]
        #         fc = "red"
        #         polygon_obstacle = getPatch(Efc, Nfc, W, L, Theta, fc)
        #
        #         Efc = obstacle.E[k]
        #         Nfc = obstacle.N[k]
        #         W = obstacle.w[k]
        #         L = obstacle.l[k]
        #         Theta = obstacle.Chi[k]
        #         fc = "green"
        #         polygon_safezone = getPatch(Efc, Nfc, W, L, Theta, fc)
        #
        #         ax.add_patch(polygon_safezone)
        #         ax.add_patch(polygon_obstacle)



    # figure 7
    iterations = np.arange(len(tElapsed))
    f, ax = plt.subplots(1)
    figno[5] = plt.gcf().number
    plt.plot(iterations, tElapsed)
    ax.set_ylabel('CPU Time [sec]')
    ax.set_xlabel('Iteration')
    ax.grid(True)

    plt.show()

    return figno

def nmpcPrint(mpciter, info, N, x, u_new, writeToFile, f, t):

    status = info['status']
    cost = info['obj_val']
    g = info['g']
    idx_lataccel = 2*N
    if ns == 6:
        #idx_trackingerror = 2*N + 2 # (nlp.py, option 1)
        idx_trackingerror = 2*N + 1 # (nlp.py, option 2,3)
    elif ns == 4:
        idx_trackingerror = 2*N + 1
    g1 = g[idx_lataccel]/32.2 # g
    g2 = g[idx_trackingerror] # ft
    text_g1 = "ay [g]"
    text_g2 = "dy [ft]"

    status_msg = info['status_msg']
    u = info['x']
    u0 = u[0]  # Vddot
    u1 = u[N]*180/np.pi  #Chiddot

    if ns == 6:
        text_u0 = "Vddot"
        text_u1 = "Chiddot"
    elif ns == 4:
        text_u0 = "Vdot"
        text_u1 = "Chidot"

    # 0       solved
    # 1       solved to acceptable level
    # 2       infeasible problem detected
    # 3       search direction becomes too small
    # 4       diverging iterates
    # 5       user requested stop
    # -1      maximum number of iterations exceeded
    # -2      restoration phase failed
    # -3      error in step computation
    # -10     not enough degrees of freedom
    # -11     invalid problem definition
    # -12     invalid option
    # -13     invalid number detected
    # -100    unrecoverable exception
    # -101    non-IPOPT exception thrown
    # -102    insufficient memo
    # -199    internal error

    if status == 0:
        status_msg_short = "Solved"
    elif status == 1:
        status_msg_short = "Acceptable"
    elif status == 2:
        status_msg_short = "Infeasible"
    elif status == -1:
        status_msg_short = "Max Iter"
    elif status == 5:
        status_msg_short = "User Stop"
    else:
        status_msg_short = status_msg[0:19]

    if writeToFile == True:
        # if mpciter == 0:
        #     f.write("%*s %*s %*s %*s %*s %*s %*s %*s %*s %*s\n" % (10, "mpciter", 10, "cost",
        #                                        7, text_u0, 7, text_u1,
        #                                        7, "V", 7, "Chi",
        #                                        7, text_g1, 7, text_g2, 15, "status_msg",
        #                                        10, "cpuTime") )
        #
        # f.write("%*d %*.1f %*.1f %*.1f %*.1f %*.1f %*.2f %*.2f %*s %*.1f\n" % (10, mpciter, 10, cost,
        #                                          7, u0, 7, u1,
        #                                          7, x[2], 7, x[3]*180/np.pi,
        #                                          7, g1, 7, g2, 15, status_msg_short,
        #                                          10, t))
        f.write("%d %0.2f\n" % (mpciter, t))

    if mpciter == 0:
        print("%*s %*s %*s %*s %*s %*s %*s %*s %*s %*s\n" % (10, "mpciter", 10, "cost",
                                               7, text_u0, 7, text_u1,
                                               7, "V", 7, "Chi",
                                               7, text_g1, 7, text_g2, 15, "status_msg",
                                              10, "cpuTime") )

    print("%*d %*.1f %*.1f %*.1f %*.1f %*.1f %*.2f %*.2f %*s %*.2f\n" % (10, mpciter, 10, cost,
                                                 7, u0, 7, u1,
                                                 7, x[2], 7, x[3]*180/np.pi,
                                                 7, g1, 7, g2, 15, status_msg_short,
                                                10,t))

    None

def savePlots(dirname,figno):
    try:
        os.makedirs(dirname)
    except OSError:
        pass
    # let exception propagate if we just can't
    # cd into the specified directory

    oldpwd = os.getcwd()
    os.chdir(dirname)

    for k in range(len(figno)):
        plt.savefig(figno[k])

    os.chdir(oldpwd)
