#!/usr/bin/env python

import sys
import copy
import rospy
import time
import math
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Bool
from moveit_commander.conversions import pose_to_list
from franka_control.msg import ErrorRecoveryAction, ErrorRecoveryActionGoal
import pickle

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


_EPS = np.finfo(float).eps * 4.0

def quaternion_matrix(quaternion):
    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [                0.0,                 0.0,                 0.0, 1.0]])


def quaternion_from_matrix(matrix, isprecise=False):
    M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
    if isprecise:
        q = np.empty((4, ))
        t = np.trace(M)
        if t > M[3, 3]:
            q[0] = t
            q[3] = M[1, 0] - M[0, 1]
            q[2] = M[0, 2] - M[2, 0]
            q[1] = M[2, 1] - M[1, 2]
        else:
            i, j, k = 0, 1, 2
            if M[1, 1] > M[0, 0]:
                i, j, k = 1, 2, 0
            if M[2, 2] > M[i, i]:
                i, j, k = 2, 0, 1
            t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
            q[i] = t
            q[j] = M[i, j] + M[j, i]
            q[k] = M[k, i] + M[i, k]
            q[3] = M[k, j] - M[j, k]
            q = q[[3, 0, 1, 2]]
        q *= 0.5 / math.sqrt(t * M[3, 3])
    else:
        m00 = M[0, 0]
        m01 = M[0, 1]
        m02 = M[0, 2]
        m10 = M[1, 0]
        m11 = M[1, 1]
        m12 = M[1, 2]
        m20 = M[2, 0]
        m21 = M[2, 1]
        m22 = M[2, 2]
        # symmetric matrix K
        K = np.array([[m00-m11-m22, 0.0,         0.0,         0.0],
                         [m01+m10,     m11-m00-m22, 0.0,         0.0],
                         [m02+m20,     m12+m21,     m22-m00-m11, 0.0],
                         [m21-m12,     m02-m20,     m10-m01,     m00+m11+m22]])
        K /= 3.0
        # quaternion is eigenvector of K that corresponds to largest eigenvalue
        w, V = np.linalg.eigh(K)
        q = V[[3, 0, 1, 2], np.argmax(w)]
    if q[0] < 0.0:
        np.negative(q, q)
    return q

def euler_to_quaternion(yaw, pitch, roll):

    roll = roll * pi / 180.0
    pitch = pitch * pi / 180.0
    yaw = yaw * pi / 180.0

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    q = geometry_msgs.msg.Quaternion()
    q.x = qx
    q.y = qy
    q.z = qz
    q.w = qw

    return q
    # self.quaternion_publisher.publish(q)

    # return [qx, qy, qz, qw]


def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    e = geometry_msgs.msg.Vector3()
    e.x = X
    e.y = Y
    e.z = Z
    # self.euler_publisher.publish(e)

    return X, Y, Z


class dataCollection(object):

  def __init__(self):
    super(dataCollection, self).__init__()

    ## initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('data_collection',
                    anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "panda_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    self.error_recovery_pub = rospy.Publisher('/franka_control/error_recovery/goal', 
                                                   ErrorRecoveryActionGoal, 
                                                  queue_size=1)

    self.start_recording_pub = rospy.Publisher("/record_start", Bool, queue_size=1)
    self.stop_recording_pub = rospy.Publisher("/record_stop", Bool, queue_size=1)

    self.touch_id_pub = rospy.Publisher("/touch_id", String, queue_size=1)

    self.touch_id = 0

    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    # Misc variables
    self.robot = robot
    self.scene = scene
    self.group = group
    self.planning_frame = planning_frame

  def go_to_pose(self, p, q):
    group = self.group

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation = q
    pose_goal.position = p
    group.set_pose_target(pose_goal)

    plan = group.go(wait=True)
    group.stop()

    group.clear_pose_targets()

    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def execute_plan(self, plan):
    group = self.group
    return group.execute(plan, wait=True)

  def plan_to_pose(self, p, q):

    group = self.group

    wpose = group.get_current_pose().pose
    wpose.position.x = p.x
    wpose.position.y = p.y
    wpose.position.z = p.z

    wpose.orientation.x = q.x
    wpose.orientation.y = q.y
    wpose.orientation.z = q.z
    wpose.orientation.w = q.w
    # waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path(
                                       [wpose],   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    #print(fraction)
    time.sleep(0.1)
    if fraction > 0.:
        return self.execute_plan(plan)
    else:
        
        return False


  def loops(self, yaws, z_levels, z_step = 0.001, reps = 0):

    err_rec_msg = ErrorRecoveryActionGoal()
    msg_true = Bool()
    msg_true.data = True

    no_error = True

    roll = -180.0
    pitch = 0.0
    yaw = -135.0
    yaw_initial = -135.0

    pos_up = self.group.get_current_pose().pose.position
    pos_up.z = pos_up.z + 0.03
    q = self.group.get_current_pose().pose.orientation


    try:

        pos_down = copy.deepcopy(pos_up)
        for dyaw in yaws:
            yaw_ = yaw + dyaw
            print([yaw_, pitch, roll])
            q = euler_to_quaternion(yaw_, pitch, roll)
            print(q)

            success_u = False
            success_u = self.plan_to_pose(pos_up, q)


            print(success_u)
            while (success_u == False):
                self.error_recovery_pub.publish(err_rec_msg)
                time.sleep(0.2)
                success_u = self.plan_to_pose(pos_up, q)
                time.sleep(0.4)
                fail = False
                msg_touchid = String()
                msg_touchid.data = str(-1)
            
            print("=========")
            print(yaw_)

            for iz in xrange(z_levels):
                pos_down.x = pos_up.x
                pos_down.y = pos_up.y
                delta_z = - (z_step * iz)
                pos_down.z = pos_up.z + delta_z - 0.03
                
                # print(pos_down.z)

                rep = 0
                fail = False
                success_d = False
                success_u = False
                while (rep < reps) and (fail == False):
                    print('rep:')
                    print(rep)
                    self.start_recording_pub.publish(msg_true)

                    success_d = self.plan_to_pose(pos_down, q)

                    
                    time.sleep(0.3)  #kad dode dole 

                    success_u = self.plan_to_pose(pos_up, q)
                    time.sleep(0.1) # kad dode gore
                    
                    fail = not success_u or not success_d

                    if (fail == True):
                        self.error_recovery_pub.publish(err_rec_msg)
                        time.sleep(0.2)
                        self.go_to_pose(pos_up, q)
                        time.sleep(0.4)
                        fail = False
                        msg_touchid = String()
                        msg_touchid.data = str(-1)
                    else:
                        rep += 1
                        self.touch_id += 1
                        msg_touchid = String()
                        msg_touchid.data = str(self.touch_id)
                    self.touch_id_pub.publish(msg_touchid)
                    time.sleep(0.1)
                    self.stop_recording_pub.publish(msg_true)
                    
                    outFolder = "/home/franka/ivona_dipl/labels/"

                    #print('ovo je yaw')
                    #print(yaw)
                    #print('ovo je yaw_')
                    #print(yaw_)
                    yaw_spremi= yaw_-yaw
                    print('yaw_ koji se sprema') 
                    print(yaw_spremi) 
                    # da se ponisti 
                    yaw_depth_arr = [yaw_spremi, delta_z]
                    pathOut = outFolder + "yaw_depth_" + str(self.touch_id) + ".txt"
                    with open(pathOut, 'wb') as fp:
                        pickle.dump(yaw_depth_arr, fp)

    except KeyboardInterrupt:
        return


def main():

  try:
    
    experiment = dataCollection()

    experiment.group.set_max_velocity_scaling_factor(0.1) #ivona : da ide sporo

    roll = -180.0
    pitch = 0.0
    yaw = -135.0
    q = euler_to_quaternion(yaw, pitch, roll)

    pose_start = experiment.group.get_current_pose().pose
    pose_start.orientation.x = q.x
    pose_start.orientation.y = q.y
    pose_start.orientation.z = q.z
    pose_start.orientation.w = q.w

    plan, fraction = experiment.group.compute_cartesian_path([pose_start],   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)
    print("fraction:")
    print(fraction)
    execute = raw_input("execute?")
    if (execute == 'y') or (execute == 'Y'):
        experiment.execute_plan(plan)


    print("Manually bring tactip above probe - pos-low")
    print("")
    execute = raw_input("ready?")
    if (execute == 'y') or (execute == 'Y'):
        pose_start = experiment.group.get_current_pose().pose
        pose_start.orientation.x = q.x
        pose_start.orientation.y = q.y
        pose_start.orientation.z = q.z
        pose_start.orientation.w = q.w

        plan, fraction = experiment.group.compute_cartesian_path([pose_start],   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)
        experiment.execute_plan(plan) 


    #yaws = [-40, -30, -20, -10, 0, 10, 20, 30, 40]  #ivona : ovo promijeni
    #yaws = [-25, -20, -15, -10, 0, 10, 15, 20, 25]
    
    yaws = []
    for i in range(0,180,5):
       yaws.append(i)
    yaws = [0]

    #yaws =  [0,90]
    z_levels = 2 # kod mene 5mm/0.5mm = 10
    reps = 1   #kolko istih slucajeva kuta i dubine zelim

    z_step = 0.5e-3  #metri #za tolko se spusti dole, kod mene 0.5mm = 0.5e-3m

    try:
        res = experiment.loops(yaws, z_levels, z_step, reps)
        time.sleep(1)

    except ValueError as e:
        return
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

  except rospy.ROSInterruptException:
      return
  except KeyboardInterrupt:
      return
  print("============ Python experiment demo complete!")

  return


if __name__ == '__main__':
    main()

