from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, solve, inv_minor
import numpy as np

def export_gibbon_ode_model() -> AcadosModel:

    model_name = 'gibbon_ode'

    # constants
    h = 3.03
    g = 9.8
    l1 = 0.8
    lc1 = l1/2
    l2 = 0.8
    lc2 = l2/2
    m1 = 1.1
    m2 = 1.1
    I1 = 1 / 3 * m1 * l1 * l1
    I2 = 1 / 3 * m2 * l2 * l2
    damp = 0.0

    # set up states & controls
    x1 = SX.sym('x1')
    x2 = SX.sym('x2')
    x3 = SX.sym('x3')
    x4 = SX.sym('x4')

    x = vertcat(x1, x2, x3, x4)

    F = SX.sym('F')
    u = vertcat(F)

    # xdot
    dx_1 = SX.sym('dx_1')
    dx_2 = SX.sym('dx_2')
    dx_3 = SX.sym('dx_3')
    dx_4 = SX.sym('dx_4')

    xdot = vertcat(dx_1, dx_2, dx_3, dx_4)

    # dynamics
    M = SX(np.array([[I1 + I2 + m2 * l1 * l1 + 2. * m2 * l1 * lc2 * cos(x2), I2 + m2 * l1 * lc2 * cos(x2)],
                  [I2 + m2 * l1 * lc2 * cos(x2), I2]]))

    C = SX(np.array([[-2. * m2 * l1 * lc2 * sin(x2) * x4, -1. * m2 * l1 * lc2 * sin(x2) * x4],
                  [m2 * l1 * lc2 * sin(x2) * x3, 0]]))

    G = vertcat(m1 * g * (h - lc1) * sin(x1) + m2 * g * ((h - l1) * sin(x1) + (h - lc2) * sin(x1 + x2)),
                  m2 * g * (h - lc2) * sin(x1 + x2))

    M_inv = inv_minor(M)
    # M_inv = solve(M, SX.eye(M.size1()))

    dx3dx4 = M_inv @ (vertcat(0, 1) * u - C @ vertcat(x3, x4) - G - damp * vertcat(x3, x4))
    f_expl = vertcat(x3, x4, dx3dx4[0], dx3dx4[1])

    f_impl = xdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.name = model_name

    return model