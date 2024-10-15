import numpy as np
import cvxpy as cp
from scipy.linalg import block_diag
np.set_printoptions(precision=3, suppress=True)


def get_contacts():
    """
        Return contact normals and locations as a matrix
        :return:
            - Contact Matrix R: <np.array> of size (2,3) containing the contact locations [r0 | r1 | r2]
            - Normal Matrix N: <np.array> of size (2,3) containing the contact locations [n0 | n1 | n2]
    """
    # ------------------------------------------------
    # FILL WITH YOUR CODE

    R = np.array([[np.sqrt(2)/4, np.sqrt(2)/4],[-0.5, 0],[0, -0.5]]).T
    N = np.array([[-np.sqrt(2)/2, -np.sqrt(2)/2], [1, 0], [0, 1]]).T
    # ------------------------------------------------
    return R, N


def calculate_grasp(R, N):
    """
        Return the grasp matrix as a function of contact locations and normals
        :param R: <np.array> locations of contact
        :param N: <np.array> contact normals
        :return: <np.array> of size (3,6) Grasp matrix for Fig. 1 containing [ J0 | J1 | J2]
    """
    # ------------------------------------------------
    # FILL WITH YOUR CODE

    r_x, r_y = R[0, :], R[1, :]
    n_x, n_y = N[0, :], N[1, :]
    
    G_row1 = np.array([n_y[0], n_x[0], n_y[1], n_x[1], n_y[2], n_x[2]])
    G_row2 = np.array([-n_x[0], n_y[0], -n_x[1], n_y[1], -n_x[2], n_y[2]])
    G_row3 = np.array([-r_x[0]*n_x[0]-r_y[0]*n_y[0], r_x[0]*n_y[0]-r_y[0]*n_x[0], -r_x[1]*n_x[1]-r_y[1]*n_y[1], r_x[1]*n_y[1]-r_y[1]*n_x[1], -r_x[2]*n_x[2]-r_y[2]*n_y[2], r_x[2]*n_y[2]-r_y[2]*n_x[2]])
    
    G = np.vstack([G_row1, G_row2, G_row3])
    
    # ------------------------------------------------
    return G


def calculate_facet(mu):
    """
        Return friction cone representation in terms of facet normals
        :param mu: <float> coefficient of friction
        :return: <np.array> Facet normal matrix
    """
    # ------------------------------------------------
    # FILL WITH YOUR CODE

    F_i = 1/np.sqrt(1 + mu**2) * np.array([[1, mu], [-1, mu]])
    F = block_diag(F_i, F_i, F_i)
    
    # ------------------------------------------------
    return F


def compute_grasp_rank(G):
    """
        Return boolean of if grasp has rank 3 or not
        :param G: <np.array> grasp matrix as a numpy array
        :return: <bool> boolean flag for if rank is 3 or not
    """
    # ------------------------------------------------
    # FILL WITH YOUR CODE

    rank = np.linalg.matrix_rank(G)
    flag = rank == 3
    
    # ------------------------------------------------
    return flag


def compute_constraints(G, F):
    """
        Return grasp constraints as numpy arrays
        :param G: <np.array> grasp matrix as a numpy array
        :param F: <np.array> friction cone facet matrix as a numpy array
        :return: <np.array>x5 contact constraints
    """
    # ------------------------------------------------
    # FILL WITH YOUR CODE

    A = np.hstack([G, np.zeros((3, 1))])
    b = np.zeros(3)
    
    P_f = np.hstack([-F, np.ones((6, 1))])
    P_d = np.array([[0, 0, 0, 0, 0, 0, -1]])
    e_T = np.array([0, 1, 0, 1, 0, 1])
    P = np.vstack([P_f, P_d, np.hstack([e_T, np.array([0])])])
    q = np.hstack([np.zeros(6), 0, 3])
    
    c = np.array([0, 0, 0, 0, 0, 0, 1]).T
    
    # ------------------------------------------------
    return A, b, P, q, c


def check_force_closure(A, b, P, q, c):
    """
        Solves Linear program given grasp constraints - DO NOT EDIT
        :return: d_star
    """
    # ------------------------------------------------
    # DO NOT EDIT THE CODE IN THIS FUNCTION
    x = cp.Variable(A.shape[1])

    prob = cp.Problem(cp.Maximize(c.T@x),
                      [P @ x <= q, A @ x == b])
    prob.solve()
    d = prob.value
    print('Optimal value of d (d^*): {:3.2f}'.format(d))
    return d
    # ------------------------------------------------


if __name__ == "__main__":
    mu = 0.3
    R, N = get_contacts()

    F = calculate_facet(mu=mu)


