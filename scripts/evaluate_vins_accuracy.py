#!/usr/bin/env python

import tf.transformations as tf
import numpy as np
import scipy.optimize as minimize


def interpolatePoses(ts_out, ts_in, pose_in):
    """
    Map pose inputs from ts_in to ts_out
    Parameters:
        ts_out - Output time stamps
        ts_in - Input time stamps
        pose_in - Array of ts_in size containing poses-[position, quaterion]
                  size (7, N), Quat is stored as (x,y,z,w)
    Return: interpolated poses at ts_out
    """
    position_out = np.interp(ts_out, ts_in, pose_in[:3, :])
    quat_out_list = []
    quat_in = pose_in[3:, :].T
    # Find fraction for interpolating quaternion
    N = ts_in.size
    out = np.interp(ts_out, ts_in, np.arange(N))
    for out_i in out:
        id0 = np.floor(out_i)
        id1 = np.min(N-1, id0+1)
        frac = out_i - id0
        quat_out = tf.quaternion_slerp(quat_in[id0], quat_in[id1], frac)
        quat_out_list.append(quat_out)
    quat_out_arr = np.vstack(quat_out_list)
    return np.hstack((position_out, quat_out_arr))

def getPoseMatrix(pose):
    """
    Generate pose matrix from quaternion and position
    Parameters
        pose - [position, quaternion] quaternion in xyzw format
    Return:
        Matrix [4x4]
    """
    mat = tf.quaternion_matrix(pose[3:])
    mat[:3, -1] = pose[:3]
    return mat

def leftMultiplyPose(pose0, pose1):
    """
    Multiply pose0 with pose1 assuming pose1 is 7xn matrix. pose0 is 7
    vector.
    """
    if len(pose1.shape) == 1:
        pose1 = np.expand_dims(pose1, axis=1)
    p0 = pose0[:3]
    q0 = pose0[3:]
    q0_c = tf.quaternion_conjugate(q0)
    p1 = pose1[:3, :]
    q1 = pose1[3:, :]
    N = p1.shape[0]
    p1_exp = np.vstack((p1, np.zeros(N)))
    p1_rot = tf.quaternion_multiply(q0_c, tf.quaternion_multiply(p1_exp, q0))
    q_out = tf.quaternion_multiply(q0, q1)
    p_out = p0 + p1_rot[:3, :]
    return np.vstack((p_out, q_out))

def rightMultiplyPose(pose0, pose1):
    """
    Multiply pose0 with pose1 assuming pose0 is 7xn matrix.
    """
    if len(pose0.shape) == 1:
        pose0 = np.expand_dims(pose0, axis=1)
    p0 = pose0[:3, :]
    q0 = pose0[3:, :]
    q0_c = tf.quaternion_conjugate(q0)
    p1 = pose1[:3]
    q1 = pose1[3:]
    p1_exp = np.hstack((p1, 0))
    p1_rot = tf.quaternion_multiply(q0_c, tf.quaternion_multiply(p1_exp, q0))
    q_out = tf.quaternion_multiply(q0, q1)
    p_out = p0 + p1_rot[:3, :]
    return np.vstack((p_out, q_out))

    
def cost(x0, vins_poses, mocap_poses):
    g0 = x0[:7] # Mocap_O to VINS_O
    g1 = x0[7:] # IMU to Marker
    mocap_vins = leftMultiplyPose(g0, vins_poses)
    mocap_marker = rightMultiplyPose(mocap_vins, g1)
    pdiff = mocap_marker[:3, :] - mocap_poses[:3, :]
    qdot= np.sum(mocap_marker[3:, :]*mocap_poses[3:, :], axis=0)
    q_dist = np.sum(1.0 - np.square(qdot))
    p_dist = np.sum(np.square(pdiff))
    return (p_dist + q_dist)

def quat_constraint1(x0):
    q0 = x0[3:7]
    return np.sum(q0*q0) - 1

def quat_constraint2(x0):
    q1 = x0[10:]
    return np.sum(q1*q1) - 1
    

# TODO Add tests and documentation
# Add rosbag support and plotting 

def estimate(vins_poses, mocap_poses):
    cons = ({'type': 'eq', 'fun':quat_constraint1},
            {'type': 'eq', 'fun':quat_constraint2})
    res = minimize(cost, np.zeros(14), args=(vins_poses, mocap_poses),
                   constraints= cons)
    euler_rpy = tf.euler_from_quaternion(res.x[10:], 'rzyx')
    print "Cost: ", res.fun
    print "IMU_to_Marker origin: ", res.x[7:10]
    print "IMU_to_Marker rpy: ", euler_rpy*(180.0/np.pi)