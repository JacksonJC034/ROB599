import numpy as np
import pybullet as p
import open3d as o3d
import assignment_6_helper as helper


def get_antipodal(pcd):
    """
    function to compute antipodal grasp given point cloud pcd
    :param pcd: point cloud in open3d format (converted to numpy below)
    :return: gripper pose (4, ) numpy array of gripper pose (x, y, z, theta)
    """
    # convert pcd to numpy arrays of points and normals
    pc_points = np.asarray(pcd.points)
    pc_normals = np.asarray(pcd.normals)

    # ------------------------------------------------
    # FILL WITH YOUR CODE

    # gripper orientation - replace 0. with your calculations
    theta = 0.
    # gripper pose: (x, y, z, theta) - replace 0. with your calculations
    gripper_pose = np.array([0., 0., 0., theta])
    # ------------------------------------------------

    return gripper_pose


def main(n_tries=5):
    # Initialize the world
    world = helper.World()

    # start grasping loop
    # number of tries for grasping
    for i in range(n_tries):
        # get point cloud from cameras in the world
        pcd = world.get_point_cloud()
        # check point cloud to see if there are still objects to remove
        finish_flag = helper.check_pc(pcd)
        if finish_flag:  # if no more objects -- done!
            print('===============')
            print('Scene cleared')
            print('===============')
            break
        # visualize the point cloud from the scene
        helper.draw_pc(pcd)
        # compute antipodal grasp
        gripper_pose = get_antipodal(pcd)
        # send command to robot to execute
        robot_command = world.grasp(gripper_pose)
        # robot drops object to the side
        world.drop_in_bin(robot_command)
        # robot goes to initial configuration and prepares for next grasp
        world.home_arm()
        # go back to the top!

    # terminate simulation environment once you're done!
    p.disconnect()
    return finish_flag


if __name__ == "__main__":
    flag = main()
