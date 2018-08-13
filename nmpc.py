import numpy as np
import probInfo
import debugLogs

def measureInitialValue(tmeasure, xmeasure):
    return tmeasure, xmeasure


def solveOptimalControlProblem(N, t0, x0, u0, T, ncons, nu, path,
                               obstacle, posIdx, ns_option, V_cmd,
                               lb_VTerm, lb_VdotVal, delChi_max, obstacleID,
                               safeDistance, debug, fHandleCost, fHandleCostGrad):

    import nlp

    # OPEN LOOP
    # u_new = np.ones([N,1])

    # CLOSED LOOP
    prob = nlp.nlpProb(N, T, t0, x0, ncons, nu, path,
                       obstacle, posIdx, ns_option, V_cmd,
                       lb_VTerm, lb_VdotVal, delChi_max, obstacleID, safeDistance, fHandleCost)
    probSetup = prob.setup(u0)

    u, info = probSetup.solve(u0.flatten(1))

    nu = len(u)/N
    u_tmp = u.reshape(nu,N)
    u_new = u_tmp.T

    if debug == True:
        debugLogs.writeLogFileCost(u, N, T, t0, x0, path, obstacle, posIdx, V_cmd, fHandleCost)
        debugLogs.writeLogFileCostGrad(u, prob, fHandleCostGrad)

    return u_new, info


def applyControl(T, t0, x0, u):
    xapplied = probInfo.system(u[0,:], x0, T)
    tapplied = t0+T
    return tapplied, xapplied


def shiftHorizon(N, u):
    nu = np.size(u, 1)
    u0 = np.zeros([N, nu])
    for k in range(nu):
        a = u[0:N-1, k]
        b = [u[N-1, k]]
        u0[:,k] = np.concatenate((a,b))
    return u0
