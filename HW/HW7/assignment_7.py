import numpy as np
import random
import time
from assignment_7_helper import World

np.set_printoptions(precision=3, suppress=True)

FIXTURE_POS = np.array([0.0, -0.15, 0.05])
BLOCK_HEIGHT = 0.05
CYAN_HEIGHT = 0.05

def move_gripper(env, x, y, z, theta, g_open):
    env.robot_command([np.array([x, y, z, theta, g_open])])

def push_object(env, obj_pos, push_dir, distance, theta=1.732):
    x, y, z = obj_pos
    z_approach = z
    g_open = 0
    approach_dist = 0.1

    start_x = x - push_dir[0]*approach_dist
    start_y = y - push_dir[1]*approach_dist

    move_gripper(env, start_x, start_y, z_approach, theta, g_open)

    z_push = z + 0.01
    move_gripper(env, start_x, start_y, z_push, theta, g_open)

    end_x = x + push_dir[0]*distance
    end_y = y + push_dir[1]*distance
    move_gripper(env, end_x, end_y, z_push, theta, g_open)

    move_gripper(env, end_x, end_y, z_approach, theta, g_open)

def pick_object(env, obj_pos, theta=0.0):
    x, y, z = obj_pos
    g_open = 0.2
    z_approach = z + 0.08
    z_grasp = z - 0.015

    move_gripper(env, x, y, z_approach, theta, g_open)

    move_gripper(env, x, y, z_grasp, theta, g_open)

    move_gripper(env, x, y, z_grasp, theta, 0.0)

    move_gripper(env, x, y, z_approach, theta, 0.0)

def place_object(env, place_pos, top_z, theta=0.0, g_open_after=0.2):
    x, y = place_pos
    z_approach = top_z + 0.08
    z_place = top_z + 0.005
    move_gripper(env, x, y, z_approach, theta, 0.0)
    time.sleep(0.1)
    move_gripper(env, x, y, z_place, 1.732/3, 0.0)
    time.sleep(0.1)
    move_gripper(env, x, y, z_place, 1.732*2/3, 0.0)
    time.sleep(0.1)
    move_gripper(env, x, y, z_place, 1.732, 0.0)
    time.sleep(0.5)
    move_gripper(env, x, y, z_place, 1.732, 0.1)
    time.sleep(0.1)
    move_gripper(env, x, y, z_approach + 0.5, 1.732, g_open_after)
    time.sleep(0.1)
    
def place_object_shift(env, place_pos, top_z, theta=0.0, g_open_after=0.2):
    x, y = place_pos
    z_approach = top_z + 0.1
    z_place = top_z + 0.005
    move_gripper(env, x, y, z_approach, theta, 0.0)
    time.sleep(0.1)
    move_gripper(env, x, y, z_place, 0, 0.0)
    time.sleep(0.1)
    move_gripper(env, x, y, z_place, 0, 0.0)
    time.sleep(0.1)
    move_gripper(env, x, y, z_place, 0, 0.0)
    time.sleep(0.5)
    move_gripper(env, x, y, z_place, 0, 0.1)
    time.sleep(0.1)
    move_gripper(env, x, y, z_approach + 0.5, 0, g_open_after)
    time.sleep(0.1)

def stack():
    env = World()
    obj_state = env.get_obj_state()
    red_pos = obj_state[0, :3]
    blue_pos = obj_state[1, :3]
    green_pos = obj_state[2, :3]
    cyan_pos = obj_state[3, :3]

    # print("Initial Object states:", obj_state)

    # Step 1: Push red and green to create space
    env.home_arm()
    push_object(env, red_pos, push_dir=[0,1], distance=0.1)
    env.home_arm()
    obj_state = env.get_obj_state()
    green_pos = obj_state[2, :3]
    push_object(env, green_pos, push_dir=[0,1], distance=0.1)

    obj_state = env.get_obj_state()
    red_pos = obj_state[0, :3]
    blue_pos = obj_state[1, :3]
    green_pos = obj_state[2, :3]
    cyan_pos = obj_state[3, :3]

    # Step 2: Pick red and place on fixture
    env.home_arm()
    red_pos = red_pos + np.array([-0.002, 0, 0])
    pick_object(env, red_pos)
    fixture_top_z = 0.05 + (0.1/2.0)
    red_drop = FIXTURE_POS[:2] + np.array([0.003, 0.003])
    place_object(env, red_drop, fixture_top_z + 0.01)
    current_stack_height = fixture_top_z + BLOCK_HEIGHT

    # Step 3: Pick blue and place on top of red
    env.home_arm()
    obj_state = env.get_obj_state()
    red_pos = obj_state[0, :3]
    blue_pos = obj_state[1, :3]
    blue_pos = blue_pos + np.array([-0.002, 0, 0])
    pick_object(env, blue_pos)
    blue_drop = FIXTURE_POS[:2] + np.array([0, 0])
    place_object_shift(env, blue_drop, current_stack_height + 0.02)
    current_stack_height += BLOCK_HEIGHT

    # Step 4: Pick green and place on top of blue
    env.home_arm()
    obj_state = env.get_obj_state()
    green_pos = obj_state[2, :3]
    green_pos = green_pos + np.array([0, 0, 0])
    pick_object(env, green_pos)
    green_drop = FIXTURE_POS[:2] + np.array([0, -0.002])
    place_object_shift(env, green_drop, current_stack_height + 0.02)
    current_stack_height += BLOCK_HEIGHT

    # Step 5: Pick cyan and place on top of green
    env.home_arm()
    obj_state = env.get_obj_state()
    cyan_pos = obj_state[3, :3]
    pick_object(env, cyan_pos)
    cyan_drop = FIXTURE_POS[:2] + np.array([0, -0.02])
    place_object_shift(env, cyan_drop, current_stack_height + 0.035)
    current_stack_height += CYAN_HEIGHT

    # Compute final average height
    obj_state = env.get_obj_state()
    avg_height = np.mean(obj_state[:, 2])
    print("Final Average Object Height: {:4.3f}".format(avg_height))
    return env, avg_height


if __name__ == "__main__":
    np.random.seed(42)
    random.seed(42)
    stack()
