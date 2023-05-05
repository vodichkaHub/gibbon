#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from mpc_solver.srv import mpc_solveRequest, mpc_solve, mpc_solveResponse

import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver
from gibbon_model import export_gibbon_ode_model
from utils import plot_pendulum

def solve(req: mpc_solveRequest):
    result = mpc_solveResponse()
    Tf = req.tf
    N = req.N_steps

    ocp = AcadosOcp()
    model = export_gibbon_ode_model()
    ocp.model = model

    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ocp.dims.N = N
    
    x0 = np.array([req.x0[0], req.x0[1], req.x0[2], req.x0[3]])
    xf = np.array([req.xf[0], req.xf[1], req.xf[2], req.xf[3]])

    Q_mat = np.diag([1e1, 1e1, 1e-2, 1e-2])
    R_mat = np.diag([1e-2])

    ocp.cost.cost_type = 'EXTERNAL'
    ocp.cost.cost_type_e = 'EXTERNAL'
    ocp.model.cost_expr_ext_cost = model.x.T @ Q_mat @ model.x + model.u.T @ R_mat @ model.u
    ocp.model.cost_expr_ext_cost_e = model.x.T @ Q_mat @ model.x

    lb_no_lim = -11111
    ub_no_lim = 11111
    Fmax = ub_no_lim
    ocp.constraints.lbu = np.array([-Fmax])
    ocp.constraints.ubu = np.array([+Fmax])
    ocp.constraints.idxbu = np.array([0])

    ocp.constraints.x0 = x0
    ocp.constraints.lbx_e = xf
    ocp.constraints.ubx_e = xf
    ocp.constraints.idxbx_e = np.array([1, ub_no_lim, ub_no_lim, ub_no_lim])

    ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES' # FULL_CONDENSING_QPOASES
    # PARTIAL_CONDENSING_HPIPM, FULL_CONDENSING_QPOASES, FULL_CONDENSING_HPIPM,
    # PARTIAL_CONDENSING_QPDUNES, PARTIAL_CONDENSING_OSQP, FULL_CONDENSING_DAQP

    # ocp.solver_options.hessian_approx = 'GAUSS_NEWTON' # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.hessian_approx = 'EXACT'
    ocp.solver_options.exact_hess_constr = 0
    ocp.solver_options.exact_hess_dyn = 0

    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.print_level = 1
    ocp.solver_options.nlp_solver_type = 'SQP' # SQP_RTI, SQP
    ocp.solver_options.tf = Tf

    ocp.solver_options.nlp_solver_max_iter = 5000
    ocp.solver_options.qp_solver_iter_max = 100

    ocp.solver_options.tol = 1e-01
    ocp.solver_options.qp_tol = 1e-01

    # ocp_solver = AcadosOcpSolverCython(ocp)
    # ocp_solver = AcadosOcpSolver(ocp)
    ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp_gibbon.json')
    # ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp.json')

    simX = np.ndarray((N+1, nx))
    simU = np.ndarray((N, nu))

    print(f'MPC task tf: {Tf}\nN steps {N}\nx0 {x0[0]} {x0[1]} {x0[2]} {x0[0]}\nxf {xf[0]} {xf[1]} {xf[2]} {xf[3]}')
    status = ocp_solver.solve()
    # ocp_solver.print_statistics()

    if status != 0:
        return result

    # get solution
    for i in range(N):
        simX[i,:] = ocp_solver.get(i, "x")
        simU[i,:] = ocp_solver.get(i, "u")
    simX[N,:] = ocp_solver.get(N, "x")
    print(f'Sol size {len(simX[:, 0])}')

    print('q1_e', simX[:, 0][-1])
    print('q2_e', simX[:, 1][-1])
    # plot_pendulum(np.linspace(0, Tf, N+1), Fmax, simU, simX, latexify=False)

    r_q0 = Float64MultiArray()
    r_q1 = Float64MultiArray()
    r_q2 = Float64MultiArray()
    r_q3 = Float64MultiArray()
    r_u = Float64MultiArray()

    r_q0.data = simX[:, 0]
    result.trajectory.append(r_q0);
    r_q1.data = simX[:, 1]
    result.trajectory.append(r_q1);
    r_q2.data = simX[:, 2]
    result.trajectory.append(r_q2);
    r_q3.data = simX[:, 3]
    result.trajectory.append(r_q3);
    r_u.data = simU[:, 0]
    result.trajectory.append(r_u);
    
    return result

def mpc_server():
    rospy.init_node('mpc_server')
    s = rospy.Service('mpc_solve', mpc_solve, solve)
    print("MPC solver has started")
    rospy.spin()

if __name__ == "__main__":
    mpc_server()