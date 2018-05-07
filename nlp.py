import ipopt
import probInfo as prob
from problemData import *


class nlpProb(object):

    def __init__(self, N, T, t0, x0, ncons, nu, path, obstacle, posIdx, ns_option, V_cmd):
        self.N = N
        self.T = T
        self.t0 = t0
        self.x0 = x0
        self.ncons = ncons  # number of constraints
        self.nu = nu # number of controls
        self.path = path
        self.obstacle = obstacle
        self.posIdx = posIdx
        self.ns_option = ns_option
        self.V_cmd = V_cmd
        pass


    def objective(self, u):
        N = self.N
        T = self.T
        t0 = self.t0
        x0 = self.x0
        path = self.path
        obstacle = self.obstacle
        posIdx = self.posIdx
        V_cmd = self.V_cmd

        x = prob.computeOpenloopSolution(u, N, T, t0, x0)
        cost = 0.0
        costvec = np.zeros([N, 1])

        for k in range(N):
            uk = np.array([u[k],u[k+N]])
            costvec[k] = prob.runningCosts( uk, x[k], t0 + k*T, path, obstacle, posIdx, V_cmd)
            cost = cost + costvec[k]

        cost_goalDist, cost_goalDelChi = prob.goalCost(x0, t0)

        cost = cost + cost_goalDist + cost_goalDelChi

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
        N = self.N
        T = self.T
        t0 = self.t0
        x0 = self.x0
        path = self.path
        obstacle = self.obstacle
        posIdx = self.posIdx
        ns_option = self.ns_option

        x = prob.computeOpenloopSolution(u, N, T, t0, x0)

        # running constraints
        # consR1_R = np.zeros(N)
        # consR1_L = np.zeros(N)
        #
        # for k in range(N):
        #     consR1_R[k], consR1_L[k] = prob.runningCons(u, x[k], t0, path, obstacle, posIdx)
        #
        # consR1 = np.concatenate([consR1_R, consR1_L])

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

                # terminal constraint  (dy, dV, delChi)
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

                # terminal constraint (dy, dV, delChi)
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
        nObstacle = len(obstacle.N)

        if nObstacle > 0:

            for j in range(nObstacle):

                for k in range(N):
                    # vehicle (point mass) location
                    position = x[k][0:2]

                    n_shape = obstacle.E_array.shape # only one object
                    if len(n_shape) == 1:
                        inside = insideBox2(position[0], position[1], obstacle.E_array, obstacle.N_array)
                    else:
                        inside = insideBox2(position[0], position[1], obstacle.E_array[j], obstacle.N_array[j])
                    if inside == False:
                        consObstacle = [1]
                    else:
                        consObstacle = [-1]

                    # consObstacle > 0, if vehicle (point object) is outside rectangle)
                    cons = np.concatenate([cons, consObstacle])

        return cons


    def jacobian(self, u):
        N = self.N
        ncons = self.ncons
        nu = self.nu
        jac = np.zeros([ncons,nu*N])
        eps = 1e-2

        for j in range(ncons):

            for k in range(nu*N):
                uplus = np.copy(u)
                uminus = np.copy(u)

                uplus[k] = uplus[k] + eps
                cons_uplus = self.constraints(uplus)

                uminus[k] = uminus[k] - eps
                cons_uminus = self.constraints(uminus)

                jac[j,k] = (cons_uplus[j] - cons_uminus[j]) / (2 * eps)


        return jac.flatten()


    def setup(self, u0):

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

        # Running Constraints
        #dyRoadL = delta_yRoad
        #dyRoadR = delta_yRoad

        # Running Constraint
        #cl_running = np.concatenate([-1*np.ones(N), 0*np.ones(N)])
        #cu_running = np.concatenate([ 0*np.ones(N), 1*np.ones(N)])
        #cl_running = np.concatenate([-100*np.ones(N), 0*np.ones(N)])
        #cu_running = np.concatenate([ 0*np.ones(N), 100*np.ones(N)])

        cl_running = np.array([], dtype=float)
        cu_running = np.array([], dtype=float)

        cl_tmp1 = np.concatenate([cl_running, [-lataccel_max]])
        cu_tmp1 = np.concatenate([cu_running, [+lataccel_max]])

        u_approx = u0.flatten(1)
        x = prob.computeOpenloopSolution(u_approx, N, T, t0, x0)

        terminal_point = x[-1,0:2]

        if distance(terminal_point, endPoint) < lb_distGoal:
            V_cmd_lo = 0
            V_cmd_hi = V_cmd + delta_V
        else:
            V_cmd_lo = V_cmd - delta_V
            V_cmd_hi = V_cmd + delta_V

        # V_cmd_lo = V_cmd - delta_V
        # V_cmd_hi = V_cmd + delta_V

        if ns_option == 1:

            # Speed Constraint
            cl_tmp2 = np.concatenate([cl_tmp1, [lb_V]])
            cu_tmp2 = np.concatenate([cu_tmp1, [ub_V]])

            # Terminal Constraint - V
            cl_tmp3 = np.concatenate([cl_tmp2, [V_cmd_lo]])
            cu_tmp3 = np.concatenate([cu_tmp2, [V_cmd_hi]])

            # Terminal Constraint - delChi
            cl = np.concatenate([cl_tmp3, [-delChi_max]])
            cu = np.concatenate([cu_tmp3, [+delChi_max]])


        elif ns_option == 2:

            # Terminal Constraint - dy
            #cl_tmp2 = np.concatenate([cl_tmp1, [-dyRoadL]])
            #cu_tmp2 = np.concatenate([cu_tmp1, [dyRoadR]])
            cl_tmp2 = cl_tmp1
            cu_tmp2 = cu_tmp1

            cl_tmp3 = np.concatenate([cl_tmp2, [V_cmd_lo]])
            cu_tmp3 = np.concatenate([cu_tmp2, [V_cmd_hi]])

            # Terminal Constraint - delChi
            cl = np.concatenate([cl_tmp3, [-delChi_max]])
            cu = np.concatenate([cu_tmp3, [+delChi_max]])

        elif ns_option == 3:

            # Terminal Constraint
            #cl = np.concatenate([cl_tmp1, [-dyRoadL]])
            #cu = np.concatenate([cu_tmp1, [dyRoadR]])

            cl = np.concatenate([cl_tmp1, [-delChi_max]])
            cu = np.concatenate([cu_tmp1, [+delChi_max]])


        if obstacle.Present == True:

            nObstacle = len(obstacle.N)
            for k in range(nObstacle):
                for k in range(N):
                    cl = np.concatenate([cl, [0]])
                    cu = np.concatenate([cu, [LARGE_NO]])


        if ncons != len(cl) and ncons != len(cu):
           print('Error: resolve number of constraints')

        nlp = ipopt.problem(
            n=nu*N,
            m=len(cl),
            problem_obj=nlpProb(N, T, t0, x0, ncons, nu, path, obstacle, posIdx, ns_option, V_cmd),
            lb=lb,
            ub=ub,
            cl=cl,
            cu=cu
        )
        nlp.addOption('print_level', nlpPrintLevel)
        nlp.addOption('max_iter', nlpMaxIter)
        #nlp.addOption('dual_inf_tol',10.0)  # defaut = 1
        nlp.addOption('constr_viol_tol',1e-4)  # default = 1e-4
        nlp.addOption('compl_inf_tol',1e-4) # default = 1e-4
        nlp.addOption('acceptable_tol',1e-6) # default = 1e-6
        nlp.addOption('acceptable_constr_viol_tol',0.01)  # default = 0.01

        return nlp
