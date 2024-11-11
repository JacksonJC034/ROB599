import numpy as np
import matplotlib.pyplot as plt
from assignment_3_helper import LCPSolve, assignment_3_render


# DEFINE GLOBAL PARAMETERS
L = 0.4
MU = 0.3
EP = 0.5
dt = 0.01
m = 0.3
g = np.array([0., -9.81, 0.])
rg = 1./12. * (2 * L * L) #TODO: Rename this to rg_squared since it is $$r_g^2$$ - Do it also in the master
M = np.array([[m, 0, 0], [0, m, 0], [0, 0, m * rg]])
Mi = np.array([[1./m, 0, 0], [0, 1./m, 0], [0, 0, 1./(m * rg)]])
DELTA = 0.001
T = 150


def get_contacts(q):
    """
        Return jacobian of the lowest corner of the square and distance to contact
        :param q: <np.array> current configuration of the object
        :return: <np.array>, <float> jacobian and distance
    """
    # ------------------------------------------------
    # FILL WITH YOUR CODE

    # Extract configuration
    x_t = q[0]
    y_t = q[1]
    theta_t = q[2]

    # Positions of corners in body frame
    corners_body = np.array([
        [ L/2,  L/2],
        [ L/2, -L/2],
        [-L/2, -L/2],
        [-L/2,  L/2]
    ])

    R = np.array([[np.cos(theta_t), -np.sin(theta_t)],
                  [np.sin(theta_t),  np.cos(theta_t)]])

    # Positions of corners in world frame
    corners_world = np.dot(R, corners_body.T).T + np.array([x_t, y_t])

    y_corners = corners_world[:, 1]
    min_y_index = np.argmin(y_corners)
    y_corner = y_corners[min_y_index]
    phi = y_corner

    # Position of the corner in body frame
    r_body = corners_body[min_y_index, :]

    # Compute J_p = ∂p_corner/∂q_t
    # ∂p_corner/∂x_t = [1; 0]
    # ∂p_corner/∂y_t = [0; 1]
    # ∂p_corner/∂θ_t = R'(θ_t) * r_body
    dR_dtheta = np.array([[-np.sin(theta_t), -np.cos(theta_t)],
                          [ np.cos(theta_t), -np.sin(theta_t)]])
    dp_dtheta = np.dot(dR_dtheta, r_body)

    J_p = np.array([[1, 0, dp_dtheta[0]],
                    [0, 1, dp_dtheta[1]]])

    jac = J_p.T
    # ------------------------------------------------
    return jac, phi


def form_lcp(jac, v):
    """
        Return LCP matrix and vector for the contact
        :param jac: <np.array> jacobian of the contact point
        :param v: <np.array> velocity of the center of mass
        :return: <np.array>, <np.array> V and p
    """
    # ------------------------------------------------
    # FILL WITH YOUR CODE
    J_t = jac[:, 0].reshape(-1, 1)
    J_n = jac[:, 1].reshape(-1, 1)
    hat_J = np.hstack([J_n, -J_t, J_t])
    V_3x3 = (hat_J.T @ Mi @ hat_J) * dt

    # Assemble the full V matrix
    V = np.zeros((4, 4))
    V[:3, :3] = V_3x3
    V[3, 0] = MU
    V[3, 1] = -1
    V[3, 2] = -1
    V[1, 3] = 1
    V[2, 3] = 1

    # Assemble the p vector
    f_e = M @ g
    Mi_f_e = Mi @ f_e
    w_n = (1 + EP) * v + dt * Mi_f_e
    w_t = v + dt * Mi_f_e

    p = np.zeros((4, 1))
    p[0] = J_n.T @ w_n
    p[1] = -J_t.T @ w_t
    p[2] = J_t.T @ w_t
    p[3] = 0
    
    p = p.reshape(-1)
    
    # ------------------------------------------------
    return V, p


def step(q, v):
    """
        predict next config and velocity given the current values
        :param q: <np.array> current configuration of the object
        :param v: <np.array> current velocity of the object
        :return: <np.array>, <np.array> q_next and v_next
    """
    # ------------------------------------------------
    # FILL WITH YOUR CODE
    
    # If in contact
    jac, phi = get_contacts(q)
    if phi < DELTA:
        V, p = form_lcp(jac, v)
        fc = lcp_solve(V, p)
        fn, ft_1, ft_2 = fc[0], fc[1], fc[2]
        v_next = v + dt * np.matmul(Mi, m * g + jac[:, 1] * fn - jac[:, 0] * ft_1 + jac[:, 0] * ft_2)
        q_next = q + dt * v_next + np.array([0, DELTA, 0])
    else:
        v_next = v + dt * np.matmul(Mi, m * g)
        q_next = q + dt * v_next
    # ------------------------------------------------
    return q_next, v_next


def simulate(q0, v0):
    """
        predict next config and velocity given the current values
        :param q0: <np.array> initial configuration of the object
        :param v0: <np.array> initial velocity of the object
        :return: <np.array>, <np.array> q and v trajectory of the object
    """
    # ------------------------------------------------
    # FILL WITH YOUR CODE
    # Initialize trajectory arrays & I.C.
    q = np.zeros((3, T))
    v = np.zeros((3, T))
    q[:, 0] = q0
    v[:, 0] = v0

    # Iterate over time steps
    for t in range(1, T):
        q_curr = q[:, t - 1]
        v_curr = v[:, t - 1]
        q_next, v_next = step(q_curr, v_curr)
        q[:, t] = q_next
        v[:, t] = v_next
    # ------------------------------------------------
    return q, v


def lcp_solve(V, p):
    """
        DO NOT CHANGE -- solves the LCP
        :param V: <np.array> matrix of the LCP
        :param p: <np.array> vector of the LCP
        :return: renders the trajectory
    """
    sol = LCPSolve(V, p)
    f_r = sol[1][:3]
    return f_r


def render(q):
    """
        DO NOT CHANGE -- renders the trajectory
        :param q: <np.array> configuration trajectory
        :return: renders the trajectory
    """
    assignment_3_render(q)


if __name__ == "__main__":
    # to test your final code, use the following initial configs
    q0 = np.array([0.0, 1.5, np.pi / 180. * 30.])
    v0 = np.array([0., -0.2, 0.])
    q, v = simulate(q0, v0)

    plt.plot(q[1, :])
    plt.show()

    render(q)




