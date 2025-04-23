import numpy as np
import pinocchio as pino
import yaml
import os
from ament_index_python.packages import get_package_share_directory


def skewSym(x):
    return np.array([
                        [0., -x[2], x[1]],
                        [x[2], 0., -x[0]],
                        [-x[1], x[0], 0.]
                    ])

def spatialForceTransformInverse(E, r):
    # E: the rotation matrix from child link to parent link, express in the child frame;
    # r: the translation vector from child link to parent link, express in the child frame;
    # XForcecInv: transformation matrix for force from child to parent
    SpatialRotation = np.block([
                                    [E.T, np.zeros_like(E)],
                                    [np.zeros_like(E), E.T]
                               ])
    SpatialTranslation = np.block([
                                    [np.identity(len(E)), np.zeros_like(E)],
                                    [skewSym(r), np.identity(len(E))]
                                  ])
    XForceInv = np.dot(SpatialTranslation, SpatialRotation)
    return XForceInv

def spatialForceTransform(E, r):
    # E: the rotation matrix from child link to parent link, express in the child frame;
    # r: the translation vector from child link to parent link, express in the child frame;
    # Note that the implementation assumes spatial force as [force, torque]^{T}
    # instead of the classic way of [force, torque]^{T}
    # XForce: transformation matrix for force from parent to child, e.g. X^b_w @ F_w = F_b,
    # b stands for body and w stands for world 
    SpatialRotation = np.block([
                                    [E, np.zeros_like(E)],
                                    [np.zeros_like(E), E]
                               ])
    SpatialTranslation = np.block([
                                    [np.identity(len(E)), np.zeros_like(E)],
                                    [-skewSym(r), np.identity(len(E))]
                                  ])
    XForce = np.dot(SpatialRotation, SpatialTranslation)
    return XForce

def get_tran_world(pino_model, pino_data, frame_name):
    # note that forward kinematics and updateFramePlacements should be called before this function
    # otherwise, the data in pino_data is not updated
    frame_id = pino_model.getFrameId(frame_name)
    # print(frame_id, len(pino_data.oMf), frame_name, "=========================================")
    return pino_data.oMf[frame_id].translation, pino_data.oMf[frame_id].rotation

def get_frame_jacobian(pino_model, pino_data, frame_name):
    # note that forward kinematics and updateFramePlacements should be called before this function
    # otherwise, the data in pino_data is not updated
    frame_id = pino_model.getFrameId(frame_name)
    return pino.getFrameJacobian(pino_model, pino_data, frame_id, pino.ReferenceFrame.LOCAL_WORLD_ALIGNED)

def trans_foot_world_aligned_to_trunk(pino_model, pino_data, foot_name):
    trunk_pos_world_to_local, trunk_rot_local_to_world = get_tran_world(pino_model, pino_data, "body")
    X_world_to_trunk = spatialForceTransform(trunk_rot_local_to_world.T, trunk_pos_world_to_local)
    
    # convert the jacobian to the world frame
    foot_pos_world, foot_rot_world = get_tran_world(pino_model, pino_data, foot_name)
    R_world_to_foot_world_aligned = np.eye(3)
    pos_world_to_foot_world_aligned = foot_pos_world
    
    X_foot_world_aligned_to_world = spatialForceTransformInverse(R_world_to_foot_world_aligned, pos_world_to_foot_world_aligned)
    return X_world_to_trunk @ X_foot_world_aligned_to_world

def trans_trunk_to_foot_world_aligned(pino_model, pino_data, foot_name):
    trunk_pos_world_to_local, trunk_rot_local_to_world = get_tran_world(pino_model, pino_data, "body")
    X_trunk_to_world = spatialForceTransformInverse(trunk_rot_local_to_world.T, trunk_pos_world_to_local)
    
    # convert the jacobian to the world frame
    foot_pos_world, foot_rot_world = get_tran_world(pino_model, pino_data, foot_name)
    R_world_to_foot_world_aligned = np.eye(3)
    pos_world_to_foot_world_aligned = foot_pos_world
    
    X_world_to_foot_world_aligned = spatialForceTransform(R_world_to_foot_world_aligned, pos_world_to_foot_world_aligned)
    return X_world_to_foot_world_aligned @ X_trunk_to_world

def trans_foot_world_aligned_to_trunk_extended(pino_model, pino_data, foot_name):
    X_foot_world_aligned_to_trunk = trans_foot_world_aligned_to_trunk(pino_model, pino_data, foot_name)
    identity = np.eye(13)
    X_foot_world_aligned_to_trunk_extended = np.block([
                                                        [X_foot_world_aligned_to_trunk, np.zeros((6, 13))],
                                                        [np.zeros((13, 6)), identity]
                                                      ])                          
    return X_foot_world_aligned_to_trunk_extended

def compute_jacobian_feet_combined_T(pino_model, pino_data, q):
    pino.forwardKinematics(pino_model, pino_data, q)
    pino.centerOfMass(pino_model, pino_data, q)
    pino.updateFramePlacements(pino_model, pino_data)
    Js = []
    
    # compute the lever arm from the CoM to the contact point
    com2cons = {}
    foot_names = ["fl_foot", "fr_foot", "rl_foot", "rr_foot"]
    for i in range(len(foot_names)):
        foot_name = foot_names[i]
        foot_pos_world, foot_rot_world = get_tran_world(pino_model, pino_data, foot_name)
        com_pos_world = pino_data.com[0]
        com2con = foot_pos_world - com_pos_world
        com2cons[foot_name] = com2con
    
    for i in range(len(foot_names)):
        foot_name = foot_names[i]
        J_foot_world_aligned = get_frame_jacobian(pino_model, pino_data, foot_name)
        # convert the jacobian to the CoM frame
        X_star_foot_world_aligned_to_trunk_extended = trans_foot_world_aligned_to_trunk_extended(pino_model, pino_data, foot_name) # X_star indicates the transformation for the spatial force
        X_star_trunk_to_foot_world_aligned = trans_trunk_to_foot_world_aligned(pino_model, pino_data, foot_name)
        J_trunk = X_star_foot_world_aligned_to_trunk_extended @ J_foot_world_aligned.T @ X_star_trunk_to_foot_world_aligned

        J_trunk_lin = J_trunk[:, :3]
        J_trunk_rot = J_trunk[:, 3:]
        
        # compute the jacobian in the local coordinate
        com2con = com2cons[foot_name]
        J_trunk_combined_T = J_trunk_lin + J_trunk_rot @ skewSym(com2con)
        Js.append(J_trunk_combined_T)
    Js = np.hstack(Js)
    return Js

def soft_sign(x, k=100):
    kx = k*x
    return np.tanh(kx)

def sign(x, use_soft_sign=True):
    if use_soft_sign:
        return soft_sign(x)
    else:
        return np.sign(x)
    
def alpha_func(err, alpha, use_soft_sign=True):
    return sign(err, use_soft_sign) * (np.abs(err) ** alpha)
    
def q_func(err, k_hg, k_sliding, use_soft_sign=True):
    return k_sliding * alpha_func(err, 1/2, use_soft_sign) + k_hg * err

def err_mapping_func(err, alg="hg", k_hg=1, k_sliding=1, use_soft_sign=True):
    if alg == "hg":
        return k_hg*err, k_hg*err
    elif alg == "sliding":
        return alpha_func(k_sliding*err, 1/2, use_soft_sign), sign(err, use_soft_sign)
    elif alg == "mixing":
        return q_func(err, k_hg, k_sliding, use_soft_sign), sign(err, use_soft_sign) + q_func(err, k_hg, k_sliding, use_soft_sign)
    else:
        raise NotImplementedError

def compute_feet_positions(pino_model, pino_data, q):
    # compute the feet positions of the foot frames
    foot_names = ["fl_foot", "fr_foot", "rl_foot", "rr_foot"]
    foot_pos = {}
    for i in range(len(foot_names)):
        foot_name = foot_names[i]
        pino.forwardKinematics(pino_model, pino_data, q)
        pino.updateFramePlacements(pino_model, pino_data)
        foot_frame = pino_model.getFrameId(foot_name)
        foot_tran = pino_data.oMf[foot_frame].translation.copy()
        foot_pos[foot_name] = np.array(foot_tran)
    return foot_pos

def dis_ob_tau_zaxis(tau, gm_measured, gm, est_f_z, q, v, pino_model, pino_data, gain_param, dt, alg="hg"):
    """
        For the observer only estimate the z-axis force
        est_f: the estimated force in the trunk frame, 3*4, [f_x, f_y, f_z]^{T}, the first two entries are ground truth, only estimate
        the z-axis force
    """
    k_hg, k_sliding, L1, L2 = gain_param
    
    pino.computeCoriolisMatrix(pino_model, pino_data, q, v)
    C = pino_data.C
    pino.computeGeneralizedGravity(pino_model, pino_data, q)
    g = pino_data.g
    
    Js_T = compute_jacobian_feet_combined_T(pino_model, pino_data, q)
    
    # split the jacobian into the z-axis part and the rest for each leg
    idx = [2, 5, 8, 11] # the index of the z-axis force in the ground truth force
    Js_z_T = Js_T[:, idx]
    Js_xy_T = np.delete(Js_T, idx, axis=1)
    
    # split the estimated force into the estimated z-axis force and the ground truth xy-axis force
    # f_xy_gt = np.delete(f_gt, idx)
    # f_xy_gt = np.zeros_like(f_xy_gt)
    f_xy_gt = np.zeros(8)
    
    est_F_z = Js_z_T @ est_f_z
    Fxy_gt = Js_xy_T @ f_xy_gt
    
    tau = np.hstack([np.zeros(6), tau])
    
    err = gm_measured - gm
    err1, err2 = err_mapping_func(err, alg=alg, k_hg=k_hg, k_sliding=k_sliding)
    dgm = (tau + Fxy_gt) + est_F_z + C.T @ v - g + L1 @ err1
    dest_Fz = L2 @ err2
    
    gm += dgm * dt
    
    # map the estimated external torque to the force
    delta_F_z = dest_Fz * dt
    delta_f_z = np.linalg.pinv(Js_z_T) @ delta_F_z
    
    # clipped the delta force to 0 if the estimated force is close to 0
    est_f_z_new = est_f_z + delta_f_z
    est_f_z_new = np.clip(est_f_z_new, [0, 0, 0, 0], [100, 100, 100, 100])
    
    # compute the delta force
    delta_f_z = est_f_z_new - est_f_z
    delta_F_z = Js_z_T @ delta_f_z
    est_F_z += delta_F_z
        
    est_f_z = np.linalg.pinv(Js_z_T) @ est_F_z
    
    est_f = np.zeros(12)
    est_f[idx] = est_f_z
    est_f[np.delete(np.arange(12), idx)] = f_xy_gt
    return gm, est_f   

from functools import reduce
class ContactDetector(object):
    def __init__(self, bandwidth=30, nv=19, freq=200, alg="mixing", pino_model=None, pino_data=None):
        yaml_path = os.path.join(get_package_share_directory('quadstack_contact'), 'resource', 'contact_param.yaml')
        # yaml_path = Path(__file__).resolve().parent / "config" / "contact_param.yaml"
        # load the yaml file for hyperparameters
        with open(f"{yaml_path}", "r") as f:
            config = yaml.safe_load(f)
            bandwidth = config["bandwidth"]
            alpha = config["alpha"]
            threshold = config["threshold"]
            foot_pos_z_threshold = config["foot_pos_z_threshold"]
            foot_pos_z_alpha = config["foot_pos_z_alpha"]
        alpha = reduce(lambda a, b: a | b, alpha) # merge the list of dicts into one dict
        threshold = reduce(lambda a, b: a | b, threshold)
        foot_pos_z_threshold = reduce(lambda a, b: a | b, foot_pos_z_threshold)
        foot_pos_z_alpha = reduce(lambda a, b: a | b, foot_pos_z_alpha)
        
        bandwidth = [bandwidth] * nv
        bandwidth = np.array(bandwidth)
        L1 = np.diag(2 * bandwidth)
        L2 = np.diag(bandwidth ** 2)
        k_hg = 1
        k_sliding = 1
        self.gain_param = [k_hg, k_sliding, L1, L2]
        self.dt = 1 / freq
        self.alg = alg
        self.gm_est = np.zeros(nv)
        self.est_f_z = np.zeros(4)
        self.est_f = np.zeros(12)
        self.est_f_filtered = np.zeros(12)
        self.zaxis_index = np.array([2, 5, 8, 11])
        self.pino_model = pino_model
        self.pino_data = pino_data
        
        # setting for filtering
        self.est_f_filtered_prev = np.zeros(12)
        self.foot_names = ["fl_foot", "fr_foot", "rl_foot", "rr_foot"]
        self.alpha = alpha
        self.threshold = threshold
        self.foot_pos_z_threshold = foot_pos_z_threshold
        self.foot_pos_z_filtered = {"fl_foot": 0, "fr_foot": 0, "rl_foot": 0, "rr_foot": 0}
        self.foot_pos_z_alpha = foot_pos_z_alpha

    def apply_contact_force_estimation(self, q, v, tau):
        M = pino.crba(self.pino_model, self.pino_data, q)
        gm_gt = M @ v
        gm_est, est_f = dis_ob_tau_zaxis(tau, gm_gt, self.gm_est, self.est_f_z, q, v, self.pino_model, self.pino_data, self.gain_param, self.dt, alg=self.alg)
        return gm_est, est_f
    
    def apply_contact_detection(self, q, v, tau):
        gm_est, est_f = self.apply_contact_force_estimation(q, v, tau)
        self.est_f_z = est_f[self.zaxis_index] # save the estimated z-axis force
        self.gm_est = gm_est # save the estimated generalized momentum
        
        est_f_filtered = np.zeros(12)
        contact_states = {}
        for i in range(len(self.foot_names)):
            idx = self.zaxis_index[i]
            foot_name = self.foot_names[i]
            alpha_foot = self.alpha[foot_name]
            est_f_filtered[idx] = (1- alpha_foot) * self.est_f_filtered_prev[idx] + alpha_foot * est_f[idx] 
            self.est_f_filtered_prev[idx] = est_f_filtered[idx]
            contact_state = est_f_filtered[idx] > self.threshold[foot_name]
            contact_states[foot_name] = contact_state
        return est_f, est_f_filtered, contact_states
    
    def apply_contact_detection_gt(self, q):
        feet_pos = compute_feet_positions(self.pino_model, self.pino_data, q)
        contact_states = {}
        feet_pos_filtered = {"fl_foot": np.zeros(3), "fr_foot": np.zeros(3), "rl_foot": np.zeros(3), "rr_foot": np.zeros(3)}
        for i in range(len(self.foot_names)):
            foot_name = self.foot_names[i]
            foot_pos_filtered = np.zeros(3)
            foot_pos = feet_pos[foot_name]
            foot_pos_z = foot_pos[2]
            alpha = self.foot_pos_z_alpha[foot_name]
            foot_pos_z_filtered_prev = self.foot_pos_z_filtered[foot_name]
            foot_pos_z_filtered =  (1 - alpha) * foot_pos_z_filtered_prev + alpha * foot_pos_z
            self.foot_pos_z_filtered[foot_name] = foot_pos_z_filtered
        
            thre = self.foot_pos_z_threshold[foot_name]
            in_contact = foot_pos_z_filtered < thre
            contact_states[foot_name] = in_contact

            # update the foot position
            foot_pos_filtered[2] = foot_pos_z_filtered
            feet_pos_filtered[foot_name] = foot_pos_filtered
            
        return feet_pos, feet_pos_filtered, contact_states