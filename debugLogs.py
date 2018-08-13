def writeLogFileCost(u, N, T, t0, x0, path, obstacle, posIdx, V_cmd, fHandleCost):
    import globalVars
    import probInfo
    import numpy as np

    x = probInfo.computeOpenloopSolution(u, N, T, t0, x0)
    costvec = np.zeros([3 * N + 2, 1])

    if u[6] * 180 / np.pi > 10:
        None

    for k in range(N):
        uk = np.array([u[k], u[k + N]])
        costout = probInfo.runningCosts(uk, x[k], t0 + k * T, path, obstacle, posIdx, V_cmd)
        costvec[k] = costout[0]  # V
        costvec[k + N] = costout[1]  # Vdot or Vddot
        costvec[k + 2 * N] = costout[2]  # Chidot or Chiddot

    cost_goalDist, cost_goalDelChi = probInfo.goalCost(x0, t0)  # goalcost_opt1
    # cost_goalDist, cost_goalDelChi = probInfo.goalCost(x[-1,:], t0) # goalcost_opt2

    costvec[3 * N] = cost_goalDist  # goal dist
    costvec[3 * N + 1] = cost_goalDelChi  # goal delta chi

    if globalVars.writeToFileCost == True:
        for k in range(3 * N):
            fHandleCost.write('%.2f ' % (costvec[k]))
        fHandleCost.write('%.2f ' % (costvec[3 * N]))
        fHandleCost.write('%.2f ' % (costvec[3 * N + 1]))
        fHandleCost.write('\n')
        # globalVars.writeToFileCost = False

