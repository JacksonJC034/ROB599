import numpy as np
import pybullet as p
import pybullet_data
import time
import matplotlib.pyplot as plt
np.set_printoptions(precision=3)

PLANE_H = 0.03
Y_OBS = np.clip(np.random.normal(loc=0., scale=.2, size=2), -0.2, 0.2)
OBS = [[0.45, Y_OBS[0], PLANE_H], [1., Y_OBS[1], PLANE_H]]
ROBOT_MASS = 0.1


class Env:
    def __init__(self, with_obs=False):
        # initialize the simulator and blocks
        self.physicsClient = p.connect(p.GUI)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        plane_id = p.loadURDF('plane.urdf', useFixedBase=True)
        p.changeDynamics(plane_id, -1, lateralFriction=0.99)

        # set camera
        p.resetDebugVisualizerCamera(cameraDistance=2,
                                     cameraYaw=45,  # 45
                                     cameraPitch=-80,  # -89
                                     cameraTargetPosition=[0, 0, 0])

        # set gravity
        p.setGravity(0, 0, 0)

        # add obstacles
        self.with_obs = with_obs
        if with_obs:
            for obs in OBS:
                self.obs = p.loadURDF('cylinder_obs.urdf',
                                      basePosition=obs,
                                      useFixedBase=True)

        # add robot
        if with_obs:
            y0 = 0.
            x0 = 1.75
        else:
            y0 = np.random.randn()
            x0 = np.random.randn()

        self.rob = p.loadURDF('cylinder_robot.urdf',
                              basePosition=[x0, y0, PLANE_H],
                              useFixedBase=False)

    def simulate(self, max_t=300):
        x_distances = []
        y_distances = []
        for sim_step in range(max_t):
            q, v = self.get_state()
            x_distances.append(abs(q[0]))
            y_distances.append(abs(q[1]))

            u = dmp_control(q, v)
            if self.with_obs:
                u = u + collision_avoidance(q, v)
            self.apply_control(u)
            p.stepSimulation()
            # print(self.get_state())
            time.sleep(0.01)
            
        # Plotting x and y distances over time
        plt.figure(figsize=(10, 5))
        plt.plot(x_distances, label='X Distance to Goal')
        plt.plot(y_distances, label='Y Distance to Goal')
        plt.xlabel('Time Step')
        plt.ylabel('Distance to Goal')
        plt.title('Distance to Goal (0, 0) Over Time')
        plt.legend()
        plt.show()

    def get_state(self):
        q, ori = p.getBasePositionAndOrientation(self.rob)
        v, w = p.getBaseVelocity(self.rob)

        return np.asarray(q[:2]), np.asarray(v[:2])

    def apply_control(self, u):
        link_id = -1
        force_loc = np.array([0., 0., 0.])
        u_3 = np.append(u, 0.)
        p.applyExternalForce(self.rob,
                             link_id,
                             u_3,
                             force_loc,
                             p.LINK_FRAME)


def dmp_control(q, v, qd=np.zeros((2, )), vd=np.zeros((2, ))):
    """
        The DMP controller
        :param q: np.array((2, )) -- configuration of the robot
        :param v: np.array((2, )) -- velocity of the robot
        :param qd: np.array((2, )) -- desired configuration of the robot
        :param vd: np.array((2, )) -- desired velocity of the robot
        :return: u: np.array((2, )) -- DMP control
    """
    u = np.zeros(2)
    #########################################

    k = 200.0  # Spring constant
    c = 2.0 * np.sqrt(k)  # Damping constant for critical damping

    pos_error = qd - q
    vel_error = vd - v
    u = ROBOT_MASS * (k * pos_error + c * vel_error)

    #########################################
    return u


def collision_avoidance(q, v, qd=np.array([0., 0.]), OBS=OBS):
    """
        The collision avoidance controller
        :param q: np.array((2, )) -- configuration of the robot
        :param v: np.array((2, )) -- velocity of the robot
        :param qd: np.array((2, )) -- desired configuration of the robot
        :param OBS: (np.array(3,), np.array(3,)) -- position of the obstacles by default is the one specified in this file.
        :return: u: np.array((2, )) -- collision avoidance control
    """
    u = np.zeros(2)
    gamma = 200.
    beta = 5
    #########################################

    for obs in OBS:

        direction_to_obs = obs[:2] - q
        distance_to_obs = np.linalg.norm(direction_to_obs)
        
        if distance_to_obs < 1e-5 or np.linalg.norm(v) < 1e-5:
            continue

        cos_phi = np.dot(direction_to_obs, v) / (distance_to_obs * np.linalg.norm(v))
        phi = np.arccos(np.clip(cos_phi, -1.0, 1.0))
        
        rotation_matrix = np.array([[0, -1], [1, 0]])
        Rv = rotation_matrix @ v
        
        u_coll_avoid = gamma * Rv * phi * np.exp(-beta * abs(phi))
        
        u += u_coll_avoid

    #########################################
    return u


def main(with_obs=False):
    env = Env(with_obs=True)
    env.simulate()


if __name__ == "__main__":
    main()

