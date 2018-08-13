import ipopt
import probInfo as prob
from problemData import *
import globalVars

class nlpProb(object):

    def __init__(self, N, T, t0, x0, ncons, nu, path, obstacle, posIdx,
                 ns_option, V_cmd, lb_VTerm, lb_VdotVal, delChi_max, obstacleID, safeDistance, fHandleCost = None):
        try:
            self.N = N
            self.T = T
            self.t0 = t0
            self.x0 = x0
            self.ncons = ncons  # number of constraints
            self.ncons_vary = np.copy(ncons)
            self.nu = nu # number of controls
            self.path = path
            self.obstacle = obstacle
            self.posIdx = posIdx
            self.ns_option = ns_option
            self.V_cmd = V_cmd
            self.lb_VTerm = lb_VTerm
            self.lb_VdotVal = lb_VdotVal
            self.fHandleCost = fHandleCost
            self.addObstacleConstraints = False
            self.obstacleNumber = np.array([], dtype=int)
            self.delChi_max = delChi_max
            self.obstacleID = obstacleID
            self.safeDistance = safeDistance

            useOnlyObstaclesInView = True

            if useOnlyObstaclesInView:
                nObstacle = len(obstacle.N)
                if nObstacle > 0:
                    for j in range(nObstacle):
    
                        p1 = x0[0:2]
                        p2 = np.array([obstacle.E[j], obstacle.N[j]])
                        distToObstacle = distance(p1, p2)
    
                        #print('{0:.1f}, {1:.1f}'.format(distToObstacle, safeDistance))
    
                        if distToObstacle < safeDistance:
                            self.addObstacleConstraints = True
                            self.obstacleNumber = np.concatenate([self.obstacleNumber, np.array([j])])
                            self.ncons_vary += N

            else:
                nObstacle = len(obstacleID)
                if nObstacle > 0:
                    for j in range(nObstacle):
    
                        id = obstacleID[j]
                        p1 = x0[0:2]
                        p2 = np.array([obstacle.E[id], obstacle.N[id]])
                        distToObstacle = distance(p1, p2)
    
                        # print('{0:.1f}, {1:.1f}'.format(distToObstacle, safeDistance))
    
                        if distToObstacle < safeDistance:
                            self.addObstacleConstraints = True
                            self.obstacleNumber = np.concatenate([self.obstacleNumber, np.array([id]) ])
                            self.ncons_vary += N

            pass
        except:
            print('Error in init')

    def objective(self, u):
        N = self.N
        T = self.T
        t0 = self.t0
        x0 = self.x0
        path = self.path
        obstacle = self.obstacle
        posIdx = self.posIdx
        V_cmd = self.V_cmd
        fHandleCost = self.fHandleCost

        x = prob.computeOpenloopSolution(u, N, T, t0, x0)
        costvec = np.zeros([3*N+2, 1])

        for k in range(N):
            uk = np.array([u[k],u[k+N]])
            costout = prob.runningCosts(uk, x[k], t0 + k*T, path, obstacle, posIdx, V_cmd)
            costvec[k] = costout[0]     # V
            costvec[k+N] = costout[1]   # Vdot or Vddot
            costvec[k+2*N] = costout[2] # Chidot or Chiddot

        cost_goalDist, cost_goalDelChi = prob.goalCost(x0, t0)  # goalcost_opt1
        #cost_goalDist, cost_goalDelChi = prob.goalCost(x[-1,:], t0) # goalcost_opt2

        costvec[3*N] = cost_goalDist # goal dist
        costvec[3*N+1] = cost_goalDelChi # goal delta chi

        cost = np.sum(costvec)

        # write data once for analysis later using a global variable. other methods can be developed to not use the
        # global variable - but this was the least intrusive way of adding the functionality
        # if globalVars.writeToFileCost == True:
        #     for k in range(3*N):
        #         fHandleCost.write('%.2f ' %(costvec[k]) )
        #     fHandleCost.write('%.2f ' % (costvec[3*N]))
        #     fHandleCost.write('%.2f ' % (costvec[3*N+1]))
        #     fHandleCost.write('\n')
        #     globalVars.writeToFileCost = False

        return cost


    def gradient(self, u):
        N = self.N
        nu = self.nu

        eps = 1e-2
        obj_grad_u = np.zeros(nu*N)
        for k in range(nu*N):
            uplus = np.copy(u)
            uminus = np.copy(u)

            uplus[k] = uplus[k] + eps
            obj_uplus = self.objective(uplus)

            uminus[k] = uminus[k] - eps
            obj_uminus = self.objective(uminus)

            obj_grad_u[k] = (obj_uplus - obj_uminus) / (2 * eps)

        return obj_grad_u


    def constraints(self, u):
        try:
            N = self.N
            T = self.T
            t0 = self.t0
            x0 = self.x0
            path = self.path
            obstacle = self.obstacle
            posIdx = self.posIdx
            ns_option = self.ns_option

            x = prob.computeOpenloopSolution(u, N, T, t0, x0)

            consR1 = np.array([], dtype=float)

            if ns == 6:

                if ns_option == 1: # Additional Current velocity + Terminal velocity constraint

                    consR2 = np.array([x[0, idx_V] * x[0, idx_Chidot] * useLatAccelCons])  # lateral acceleration
                    consR3 = np.array([x[0, idx_V]])  # current velocity

                    constmp = np.concatenate([consR1, consR2])
                    consR = np.concatenate([constmp, consR3])

                    # terminal constraint (dy, V, delChi)
                    consT1, consT2, consT3 = prob.terminalCons(u, x[N - 1], t0, path, obstacle, posIdx)
                    consT = np.concatenate([consT2, consT3])

                elif ns_option == 2:

                    # No terminal velocity constraint
                    consR2 = np.array([x[0, idx_V] * x[0, idx_Chidot] * useLatAccelCons])  # lateral acceleration

                    consR = np.concatenate([consR1, consR2])

                    # terminal constraint (dy, dV, delChi)
                    consT1, consT2, consT3 = prob.terminalCons(u, x[N - 1], t0, path, obstacle, posIdx)
                    consT = np.concatenate([consT2, consT3])

                elif ns_option == 3:

                    # No terminal velocity constraint
                    consR2 = np.array([x[0, idx_V] * x[0, idx_Chidot] * useLatAccelCons])  # lateral acceleration

                    consR = np.concatenate([consR1, consR2])

                    # terminal constraint  (dy, V, delChi)
                    consT1, consT2, consT3 = prob.terminalCons(u, x[N - 1], t0, path, obstacle, posIdx)
                    consT = consT3

            elif ns == 4:

                if ns_option == 1:

                    u_mat = u.reshape(2, -1).T
                    consR2 = np.array([x[0, idx_V] * u_mat[0, idx_Chidot] * useLatAccelCons])  # lateral acceleration
                    consR3 = np.array([x[0, idx_V]])  # current velocity

                    constmp = np.concatenate([consR1, consR2])
                    consR = np.concatenate([constmp, consR3])

                    # terminal constraint  (dy, dV, delChi)
                    consT1, consT2, consT3 = prob.terminalCons(u, x[N - 1], t0, path, obstacle, posIdx)
                    consT = np.concatenate([consT2, consT3])

                elif ns_option == 2:

                    u_mat = u.reshape(2, -1).T
                    consR2 = np.array([x[0, idx_V] * u_mat[0, idx_Chidot] * useLatAccelCons])  # lateral acceleration

                    consR = np.concatenate([consR1, consR2])

                    # terminal constraint (dy, V, delChi)
                    consT1, consT2, consT3 = prob.terminalCons(u, x[N-1], t0, path, obstacle, posIdx)  # ydist, VEnd
                    consT = np.concatenate([consT2, consT3])


                elif ns_option == 3:

                    u_mat = u.reshape(2,-1).T
                    consR2 = np.array([x[0, idx_V] * u_mat[0, idx_Chidot] * useLatAccelCons])  # lateral acceleration

                    consR = np.concatenate([consR1, consR2])

                    # terminal constraint (dy, dV, delChi)
                    consT1, consT2, consT3 = prob.terminalCons(u, x[N-1], t0, path, obstacle, posIdx)  # ydist, VEnd
                    consT = consT3

            # total constraints without obstacles
            cons = np.concatenate([consR,consT])

            # total constraints with obstacles
            if self.addObstacleConstraints == True:

                for j in self.obstacleNumber:
                    for k in range(N):
                        position = x[k][0:2]
                        obstacleDistance = np.sqrt([(obstacle.E[j] - position[0]) ** 2 +
                                            (obstacle.N[j] - position[1]) ** 2])
                        cons = np.concatenate([cons, obstacleDistance])

            return cons
        except:
            print('Error in constraints')

    def jacobian(self, u):
        try:
            N = self.N
            ncons_vary = self.ncons_vary
            nu = self.nu
            jac = np.zeros([ncons_vary,nu*N])
            eps = 1e-2

            for j in range(ncons_vary):

                for k in range(nu*N):
                    uplus = np.copy(u)
                    uminus = np.copy(u)

                    uplus[k] = uplus[k] + eps
                    cons_uplus = self.constraints(uplus)

                    uminus[k] = uminus[k] - eps
                    cons_uminus = self.constraints(uminus)

                    jac[j,k] = (cons_uplus[j] - cons_uminus[j]) / (2 * eps)

            return jac.flatten()
        except:
            print('Error in jacobian')


    def setup(self, u0):
        try:
            N = self.N
            T = self.T
            t0 = self.t0
            x0 = self.x0
            nu = self.nu
            path = self.path
            obstacle = self.obstacle
            posIdx = self.posIdx
            ns_option = self.ns_option
            V_cmd = self.V_cmd
            lb_VTerm = self.lb_VTerm
            lb_VdotVal = self.lb_VdotVal
            fHandleCost = self.fHandleCost
            delChi_max = self.delChi_max
            obstacleID = self.obstacleID
            safeDistance = self.safeDistance

            LARGE_NO = 1e12

            if ns == 6:

                lb_Vddot = np.ones([N,1])*lb_VddotVal
                lb_Chiddot = np.ones([N,1])*lb_ChiddotVal

                ub_Vddot = np.ones([N,1])*ub_VddotVal
                ub_Chiddot = np.ones([N,1])*ub_ChiddotVal

                lb = np.concatenate([lb_Vddot, lb_Chiddot])
                ub = np.concatenate([ub_Vddot,ub_Chiddot])

            elif ns == 4:

                lb_Vdot = np.ones([N, 1]) * lb_VdotVal
                lb_Chidot = np.ones([N, 1]) * lb_ChidotVal

                ub_Vdot = np.ones([N, 1]) * ub_VdotVal
                ub_Chidot = np.ones([N, 1]) * ub_ChidotVal

                lb = np.concatenate([lb_Vdot, lb_Chidot])
                ub = np.concatenate([ub_Vdot, ub_Chidot])


            lataccel_max = lataccel_maxVal

            cl_running = np.array([], dtype=float)
            cu_running = np.array([], dtype=float)

            cl_tmp1 = np.concatenate([cl_running, [-lataccel_max]])
            cu_tmp1 = np.concatenate([cu_running, [+lataccel_max]])

            #u_approx = u0.flatten(1)
            #x = prob.computeOpenloopSolution(u_approx, N, T, t0, x0)

            if ns_option == 1:

                # Speed Constraint
                cl_tmp2 = np.concatenate([cl_tmp1, [lb_V]])
                cu_tmp2 = np.concatenate([cu_tmp1, [ub_V]])

                # Terminal Constraint - V
                tmp = 0
                cl_tmp3 = np.concatenate([cl_tmp2, [tmp]]) # need to modify
                cu_tmp3 = np.concatenate([cu_tmp2, [tmp]])

                # Terminal Constraint - delChi
                cl = np.concatenate([cl_tmp3, [-delChi_max]])
                cu = np.concatenate([cu_tmp3, [+delChi_max]])


            elif ns_option == 2:

                cl_tmp2 = cl_tmp1
                cu_tmp2 = cu_tmp1

                cl_tmp3 = np.concatenate([cl_tmp2, [lb_VTerm]])
                cu_tmp3 = np.concatenate([cu_tmp2, [ub_VTerm]])

                # Terminal Constraint - delChi
                cl = np.concatenate([cl_tmp3, [-delChi_max]])
                cu = np.concatenate([cu_tmp3, [+delChi_max]])

            elif ns_option == 3:

                cl = np.concatenate([cl_tmp1, [-delChi_max]])
                cu = np.concatenate([cu_tmp1, [+delChi_max]])

            # total constraints with obstacles

            if self.addObstacleConstraints == True:

                #print(self.obstacleNumber)
                for j in self.obstacleNumber:
                    for k in range(N):
                        cl = np.concatenate([cl, [obstacle.sr[j]]])
                        cu = np.concatenate([cu, [LARGE_NO]])

            nlp = ipopt.problem(
                n=nu*N,
                m=len(cl),
                problem_obj=nlpProb(N, T, t0, x0, ncons, nu, path,
                                    obstacle, posIdx, ns_option, V_cmd,
                                    lb_VTerm, lb_VdotVal, delChi_max, obstacleID, safeDistance, fHandleCost),
                lb=lb,
                ub=ub,
                cl=cl,
                cu=cu
            )
            #print(len(cl))
            nlp.addOption('print_level', nlpPrintLevel)
            nlp.addOption('max_iter', nlpMaxIter)
            #nlp.addOption('dual_inf_tol',10.0)  # defaut = 1
            nlp.addOption('constr_viol_tol',1e-4)  # default = 1e-4
            nlp.addOption('compl_inf_tol',1e-4) # default = 1e-4
            nlp.addOption('acceptable_tol',1e-6) # default = 1e-6
            nlp.addOption('acceptable_constr_viol_tol',0.01)  # default = 0.01

            return nlp
        except:
            print('Error in setup')