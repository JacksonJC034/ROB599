import numpy as np
import random
from assignment_7_helper import World

np.set_printoptions(precision=3, suppress=True)


def stack():
    """
    function to stack objects
    :return: average height of objects
    """
    # DO NOT CHANGE: initialize world
    env = World()

    # ============================================
    # YOUR CODE HERE:
    # example get object states
    obj_state = env.get_obj_state()
    print(obj_state)

    # example send command to robot
    robot_command = [np.array([0., 0., 0., 0. * np.pi / 180., 0.2])]
    env.robot_command(robot_command)

    # example get robot state
    rob_state = env.get_robot_state()
    print(rob_state)


    # ============================================
    # DO NOT CHANGE: getting average height of objects:
    obj_state = env.get_obj_state()
    avg_height = np.mean(obj_state[:, 2])
    print("Average Object Height: {:4.3f}".format(avg_height))
    return env, avg_height


if __name__ == "__main__":
    np.random.seed(42)
    random.seed(42)
    stack()
