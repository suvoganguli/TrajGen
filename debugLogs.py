import numpy as np

def writeLogFileCost(u, N, T, t0, x0, path, obstacle, posIdx, V_cmd, fHandleCost):

    costvec = getCostVec(u, N, T, t0, x0, path, obstacle, posIdx, V_cmd)

    for k in range(3 * N):
        fHandleCost.write('%.2f ' % (costvec[k]))
    fHandleCost.write('%.2f ' % (costvec[3 * N]))
    fHandleCost.write('%.2f ' % (costvec[3 * N + 1]))
    fHandleCost.write('\n')


def writeLogFileCostGrad(u, N, T, t0, x0, path, obstacle, posIdx, V_cmd, prob, fHandleCostGrad):
    costGrad = prob.gradient(u)
    n = len(costGrad)

    if False:
        costvec = getCostVec(u, N, T, t0, x0, path, obstacle, posIdx, V_cmd)
        eps = 1e-2
        if np.abs(costGrad[0]) > 0.05:

            k = 0
            uplus = np.copy(u)
            uminus = np.copy(u)

            uplus[k] = uplus[k] + eps
            obj_uplus = prob.objective(uplus)

            uminus[k] = uminus[k] - eps
            obj_uminus = prob.objective(uminus)

            obj_grad_u_0 = (obj_uplus - obj_uminus) / (2 * eps)

            costvec_uplus = getCostVec(uplus, N, T, t0, x0, path, obstacle, posIdx, V_cmd)
            costvec_uminus = getCostVec(uminus, N, T, t0, x0, path, obstacle, posIdx, V_cmd)
            costvec_grad = (costvec_uplus - costvec_uminus) / (2 * eps)

            costvec_grad[6]
            costvec_gradb = 0.5*( (u[k] + eps)**2 - (u[k] - eps)**2 ) / (2 * eps)

            None

    for k in range(n):
        fHandleCostGrad.write('%.2f ' % (costGrad[k]))
    fHandleCostGrad.write('\n')


def getCostVec(u, N, T, t0, x0, path, obstacle, posIdx, V_cmd):
    import probInfo
    import numpy as np

    x = probInfo.computeOpenloopSolution(u, N, T, t0, x0)
    costvec = np.zeros([3 * N + 2, 1])

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

    return costvec