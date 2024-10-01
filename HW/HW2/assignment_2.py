import numpy as np
import cvxpy as cp
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

    R = np.zeros((2, 3))  # TODO: Replace None with your result
    N = np.zeros((2, 3))  # TODO: Replace None with your result
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

    G = np.zeros((3, 6))  # TODO: Replace with your result
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

    F = np.zeros((6, 6))  # TODO: Replace with your result
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

    flag = None  # TODO: Replace None with your result
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

    A = None   # TODO: Replace None with your result
    b = None   # TODO: Replace None with your result
    P = None   # TODO: Replace None with your result
    q = None   # TODO: Replace None with your result
    c = None   # TODO: Replace None with your result
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


