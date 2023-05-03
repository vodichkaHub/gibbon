#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver
from gibbon_model import export_gibbon_ode_model
from utils import plot_pendulum

def solve():
    ocp = AcadosOcp()
    model = export_gibbon_ode_model()
    ocp.model = model

    Tf = 4.0
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    N = 200
    ocp.dims.N = N

    Q_mat = 2*np.diag([1e3, 1e3, 1e-2, 1e-2])
    R_mat = 2*np.diag([1e-2])

    ocp.cost.cost_type = 'EXTERNAL'
    ocp.cost.cost_type_e = 'EXTERNAL'
    ocp.model.cost_expr_ext_cost = model.x.T @ Q_mat @ model.x + model.u.T @ R_mat @ model.u
    ocp.model.cost_expr_ext_cost_e = model.x.T @ Q_mat @ model.x

    Fmax = 30
    ocp.constraints.lbu = np.array([-Fmax])
    ocp.constraints.ubu = np.array([+Fmax])
    ocp.constraints.idxbu = np.array([0])
    ocp.constraints.x0 = np.array([0.0, np.pi/2, 0.0, 0.0])

    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' # FULL_CONDENSING_QPOASES
    # PARTIAL_CONDENSING_HPIPM, FULL_CONDENSING_QPOASES, FULL_CONDENSING_HPIPM,
    # PARTIAL_CONDENSING_QPDUNES, PARTIAL_CONDENSING_OSQP, FULL_CONDENSING_DAQP
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON' # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = 'IRK'
    # ocp.solver_options.print_level = 1
    ocp.solver_options.nlp_solver_type = 'SQP' # SQP_RTI, SQP
    ocp.solver_options.tf = Tf

    # ocp_solver = AcadosOcpSolverCython(ocp)
    # ocp_solver = AcadosOcpSolver(ocp)
    ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp_gibbon.json')
    # ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp.json')

    simX = np.ndarray((N+1, nx))
    simU = np.ndarray((N, nu))

    status = ocp_solver.solve()
    ocp_solver.print_statistics()

    if status != 0:
        raise Exception(f'acados returned status {status}.')

    # get solution
    for i in range(N):
        simX[i,:] = ocp_solver.get(i, "x")
        simU[i,:] = ocp_solver.get(i, "u")
    simX[N,:] = ocp_solver.get(N, "x")

    print(simX[:, 1])
    plot_pendulum(np.linspace(0, Tf, N+1), Fmax, simU, simX, latexify=False)
    
    return True


if __name__ == '__main__':
    pub = rospy.Publisher('chatter', String, queue_size=10)
    
    rospy.init_node('mpc_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    suc = solve()

    while not rospy.is_shutdown():
        rospy.loginfo(suc)
        pub.publish("hello_str")
        rate.sleep()