#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from ros_arm.srv import GoalPosition, GoalPositionResponse


def goal_error(pos_joint, goal_joint):
    '''
    Estimate if the goal is reached
    - pos_joint: current joint position
    - param goal_joint: goal joint position
    '''

    tolerance = .05  # error tolerance
    result = abs(pos_joint - goal_joint) <= abs(tolerance)
    return result


def polynomial_interpolation(start, end, t):
    """
    Third-order polynomial interpolation function.
    - start: Initial value
    - end: Final value
    - t: Interpolation parameter (0 to 1)
    """
    return start + (end - start) * (3 * t**2 - 2 * t**3)


def move_arm(pos_joints):
    '''
    Move the arm to the goal position
    - pos_joints: goal joint position
    '''

    time_elapsed = rospy.Time.now()

    for i in range(len(pos_joints)):
        globals()['pub_j'+str(i+1)].publish(pos_joints[i])
    
    joint_state = rospy.wait_for_message('/arm/joint_states', JointState)
    initial_positions = [joint_state.position[i]
                         for i in range(len(pos_joints))]

    while True:
        current_time = rospy.Time.now()
        elapsed_time = current_time - time_elapsed
        if elapsed_time.to_sec() > 1:
            rospy.logwarn("Timeout occurred!")
            break

        joint_state = rospy.wait_for_message('/arm/joint_states', JointState)
        result = True

        # Interpolate joint positions using the polynomial
        interpolation_factor = elapsed_time.to_sec() / 1
        interpolated_positions = [polynomial_interpolation(initial, target, interpolation_factor)
                                  for initial, target in zip(initial_positions, pos_joints)]

        for i in range(len(pos_joints)):
            result = goal_error(
                joint_state.position[i], interpolated_positions[i]) and result

        if result:
            time_elapsed = joint_state.header.stamp - time_elapsed
            break

    return time_elapsed


# limit joint angle
def clamp_at_boundaries(requested_joint):
    min_joint = rospy.get_param('~min_joint_angle', -2*math.pi)
    max_joint = rospy.get_param('~max_joint_angle', 2*math.pi)

    clamped_joint = requested_joint
    if not min_joint <= requested_joint <= max_joint:
        clamped_joint = min(max(requested_joint, min_joint), max_joint)
        rospy.logwarn('j1 is out of bounds, valid range (%s,%s), clamping to: %s',
                      min_joint, max_joint, clamped_joint)

    return clamped_joint


# handle the move request
def handle_move_request(req):
    joint1 = req.joint1
    joint2 = req.joint2
    joint3 = req.joint3
    joint4 = req.joint4
    joint5 = req.joint5
    joint6 = req.joint6
    finger_joint1 = req.finger_joint1
    finger_joint2 = req.finger_joint2
    joints = [joint1, joint2, joint3, joint4,
              joint5, joint6, finger_joint1, finger_joint2]
    clamp_joint = [0, 0, 0, 0, 0, 0, 0, 0]

    for i in range(len(joints)):
        rospy.loginfo('GoToPositionRequest Received - j%s:%s,', i, joints[i])
        clamp_joint[i] = clamp_at_boundaries(joints[i])
    time_elapsed = move_arm(clamp_joint)

    return GoalPositionResponse(time_elapsed)

# define mover service


def mover_service():
    rospy.init_node('arm_mover')
    service = rospy.Service('~arm_mover', GoalPosition, handle_move_request)
    rospy.spin()


if __name__ == '__main__':
    pub_j1 = rospy.Publisher(
        '/arm/joint1_position_controller/command', Float64, queue_size=10)
    pub_j2 = rospy.Publisher(
        '/arm/joint2_position_controller/command', Float64, queue_size=10)
    pub_j3 = rospy.Publisher(
        '/arm/joint3_position_controller/command', Float64, queue_size=10)
    pub_j4 = rospy.Publisher(
        '/arm/joint4_position_controller/command', Float64, queue_size=10)
    pub_j5 = rospy.Publisher(
        '/arm/joint5_position_controller/command', Float64, queue_size=10)
    pub_j6 = rospy.Publisher(
        '/arm/joint6_position_controller/command', Float64, queue_size=10)
    pub_j7 = rospy.Publisher(
        '/arm/finger_joint1_position_controller/command', Float64, queue_size=10)
    pub_j8 = rospy.Publisher(
        '/arm/finger_joint2_position_controller/command', Float64, queue_size=10)
    try:
        mover_service()
    except rospy.ROSInterruptException:
        pass
