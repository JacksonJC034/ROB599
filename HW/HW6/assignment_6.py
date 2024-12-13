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
    w_max = 0.15
    
    best_pair = None
    best_score = -np.inf  # Keep track of how "good" the pair is (dot product closeness to -1)
    
    N = pc_points.shape[0]
    for i in range(N):
        p1 = pc_points[i]
        n1 = pc_normals[i]
        
        # Look for opposite normals
        # Condition: n1 dot n2 ≈ -1
        opp_normal = -n1
        
        # Compute dot products with all normals to find candidates
        dot_products = np.einsum('ij,j->i', pc_normals, opp_normal) # dot each normal with opp_normal
        
        # Filter candidates based on dot product threshold
        # A threshold like dot_products > 0.95 means cos(theta) ~ 0.95, angle ~ >160 degrees.
        candidates = np.where(dot_products > 0.95)[0]
        
        for c in candidates:
            if c == i:
                continue
            p2 = pc_points[c]
            n2 = pc_normals[c]
            
            # Check distance constraint
            dist = np.linalg.norm(p2 - p1)
            if dist <= w_max:
                # Evaluate how good this pair is
                # One metric: just use the dot product to pick the best opposite normals
                score = dot_products[c]
                if score > best_score:
                    best_score = score
                    best_pair = (p1, p2, n1, n2)
    
    # If no pair found, default to something
    if best_pair is None:
        # No antipodal grasp found – fallback strategy
        # Just place gripper at origin
        return np.array([0., 0., 0.1, 0.])  # some safe default pose
    
    p1, p2, n1, n2 = best_pair
    grasp_center = 0.5 * (p1 + p2)
    
    # Compute theta
    d = p2 - p1
    theta = np.arctan2(d[1], d[0])
    
    # Set gripper z a bit above the contact points (offest is not needed (tested))
    z_gripper = max(p1[2], p2[2])
    
    gripper_pose = np.array([grasp_center[0], grasp_center[1], z_gripper, theta])
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
