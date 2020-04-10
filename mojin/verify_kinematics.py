#!/usr/bin/env python

import six
import math
import random
import types
import numpy as np

import rospy
import tf
from tf import transformations
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from moveit_msgs.msg import Constraints, MoveItErrorCodes, RobotState, DisplayRobotState
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse

from moveit_commander import MoveGroupCommander, RobotCommander



# kinematics helper
_compute_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
_compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

def compute_fk(frame_id, fk_link_names, robot_state):
    fk_request = GetPositionFKRequest(
        fk_link_names=fk_link_names,
        robot_state=robot_state)
    fk_request.header.frame_id = frame_id

    return _compute_fk(fk_request)

def compute_ik(group_name, robot_state, pose_stamped, ik_link_name='', constraints=Constraints(), avoid_collisions=False, timeout=rospy.Duration(0), attempts=0):
    ik_request = GetPositionIKRequest(ik_request=PositionIKRequest(
                                                    group_name=group_name,
                                                    robot_state=robot_state,
                                                    pose_stamped=pose_stamped,
                                                    ik_link_name=ik_link_name,
                                                    constraints=constraints,
                                                    avoid_collisions=avoid_collisions,
                                                    timeout=timeout,
                                                    attempts=attempts))
    return _compute_ik(ik_request)

# diff helper
X, Y, Z, W = 0, 1, 2, 3
AXES = [X, Y, Z]
axis_lbl = ['x', 'y', 'z']

def orientation_diff(a, b, epsilon=0.0001):
    a_euler = transformations.euler_from_quaternion([a.x, a.y, a.z, a.w])
    b_euler = transformations.euler_from_quaternion([b.x, b.y, b.z, b.w])
    close = np.allclose(a=a_euler, b=b_euler, atol=epsilon)
    if not close:
        for axis in AXES:
            if not np.allclose(a_euler[axis], b_euler[axis], epsilon):
                print('rotation around {}: {:.4f}'.format(axis_lbl[axis], b_euler[axis] - a_euler[axis]))

    return not close


def position_diff(a, b, epsilon=0.0001):
    a_pos = [a.x, a.y, a.z]
    b_pos = [b.x, b.y, b.z]
    close = np.allclose(a_pos, b_pos, epsilon)
    if not close:
        for axis in AXES:
            if not np.allclose(a_pos[axis], b_pos[axis], epsilon):
                print('position.{}: {:.4f}'.format(axis_lbl[axis], b_pos[axis] - a_pos[axis]))
    return not close


def pose_diff(a, b, epsilon=0.0001):
    p_diff = position_diff(a.position, b.position, epsilon=epsilon)
    o_diff = orientation_diff(a.orientation, b.orientation, epsilon=epsilon)
    return p_diff or o_diff


# utils helper
def gen_pose(frame_id="base_link", pos=(0,0,0), euler=(0,0,0)):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pos
    pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(*euler))
    return pose

def format_posestamped(ps):
    # type: (PoseStamped) -> str
    return "{{(x: {}, y: {}, z: {}), (qx: {}, qy: {}, qz: {}, qw: {}) @ {}}}"\
        .format(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z,
                ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w,
                ps.header.frame_id)

def class_enum_to_string(enum_num,_class):
    try:
        return next((k for k,v in _class.__dict__.items() if type(v) is int and v==enum_num))
    except:
        return "UNKNOWN VALUE"






if __name__ == '__main__':
    rospy.init_node('verify_kinematics')
    while rospy.get_time() == 0.0: pass

    rc = RobotCommander(robot_description="robot_description", ns="")
    mgc = MoveGroupCommander("arm")
    goal_pub = rospy.Publisher('/goal_pose', PoseStamped, queue_size=1)
    robot_state_pub = rospy.Publisher('/pick_place/robot_state', DisplayRobotState, queue_size=1)
    rospy.sleep(1.0)

    # Forward Kinematics
    frame_id = mgc.get_planning_frame()
    fk_link_names = [mgc.get_end_effector_link()]
    robot_state = rc.get_current_state()
    rospy.logdebug('FKRequest: \n\
                    frame_id: {}\n\
                    fk_link_names: {}\n\
                    robot_state: {}'.format(frame_id, fk_link_names, robot_state))
    fk_response = compute_fk(frame_id, fk_link_names, robot_state)
    rospy.logdebug('FKResponse: \n\
                    fk_response: {}'.format(fk_response))
    rospy.logwarn('FK ErrorCode: {}'.format(class_enum_to_string(fk_response.error_code.val, MoveItErrorCodes)))

    no_ik_count = 0
    fk_diff_count = 0

    #while not rospy.is_shutdown():
    total = 100
    for i in range(total):
        # Random Target
        roll = rospy.get_param('goal/roll', random.uniform(-math.pi, math.pi))
        pitch = rospy.get_param('goal/pitch', random.uniform(-math.pi, math.pi))
        yaw = rospy.get_param('goal/yaw', random.uniform(-math.pi, math.pi))
        goal_pose = gen_pose(rospy.get_param('goal/frame_id', mgc.get_planning_frame().strip("/")),
                             pos=[rospy.get_param('goal/x', random.uniform(-1.0, 1.0)),
                                  rospy.get_param('goal/y', random.uniform(-1.0, 1.0)),
                                  rospy.get_param('goal/z', random.uniform( 0.2, 2.0))],
                             euler=[roll,
                                    pitch,
                                    yaw])
        rospy.loginfo("goal_pose: {}".format(format_posestamped(goal_pose)))
        goal_pub.publish(goal_pose)

        # Inverse Kinematics

        #six.moves.input("Press ENTER to compute ik")
        group_name = mgc.get_name()
        robot_state = rc.get_current_state() # seed
        pose_stamped = goal_pose
        rospy.logdebug('IKRequest: \n\
                       group_name: {}\n\
                       robot_state: {}\n\
                       pose_stamped: {}'.format(group_name, robot_state, pose_stamped))
        ik_response = compute_ik(group_name, robot_state, pose_stamped, ik_link_name='', constraints=Constraints(), avoid_collisions=False, timeout=rospy.Duration(0), attempts=0)
        rospy.logdebug('IKResponse: \n\
                       ik_response: {}'.format(ik_response))
        rospy.logwarn('IK ErrorCode: {}'.format(class_enum_to_string(ik_response.error_code.val, MoveItErrorCodes)))

        if ik_response.error_code.val == MoveItErrorCodes.SUCCESS:
            #display ik solution
            display_robot_state = DisplayRobotState()
            display_robot_state.state = ik_response.solution
            robot_state_pub.publish(display_robot_state)

            #fk of ik solution
            frame_id = mgc.get_planning_frame()
            fk_link_names = [mgc.get_end_effector_link()]
            robot_state = ik_response.solution
            rospy.logdebug('FKRequest: \n\
                            frame_id: {}\n\
                            fk_link_names: {}\n\
                            robot_state: {}'.format(frame_id, fk_link_names, robot_state))
            fk_response = compute_fk(frame_id, fk_link_names, robot_state)
            rospy.logdebug('FKResponse: \n\
                            fk_response: {}'.format(fk_response))
            rospy.logwarn('FK ErrorCode: {}'.format(class_enum_to_string(fk_response.error_code.val, MoveItErrorCodes)))

            #pose_diff
            if pose_diff(fk_response.pose_stamped[0].pose, goal_pose.pose, epsilon=0.005): # kinematics_solver_search_resolution
                rospy.logerr('pose_diff between goal_pose and fk of ik_solution')
                fk_diff_count += 1
        else:
            no_ik_count += 1
        
        #six.moves.input("Press ENTER for next Pose")

    rospy.loginfo('total: {}'.format(total))
    rospy.loginfo('no_ik_count: {}'.format(no_ik_count))
    rospy.loginfo('fk_diff_count: {}'.format(fk_diff_count))
