--[[
--  hb4.0 lua model for use with the RBDL library
--]]

local baseInertia = {
    { 0.28, 0.,     0. },
    { 0.,     0.34, 0. },
    { 0.,     0.,     0.54 }
}
local l0Inertia = {
    { 0.0067, 0.,     0. },
    { 0.,     0.0285, 0. },
    { 0.,     0.,     0.0274 }
}
local l1Inertia = {
    { 0.0335, 0.,     0. },
    { 0.,     0.0158, 0. },
    { 0.,     0.,     0.0393 }
}
local l2Inertia = {
    { 0.0213, 0.,     0. },
    { 0.,     0.0002, 0. },
    { 0.,     0.,     0.0212 }
}

--leg links
local base = { mass = 6.2, com = { 0., 0., 0. }, inertia = baseInertia }
local l0frrl = { mass = 0.630, com = { 0.055, 0.016, 0. }, inertia = l0Inertia }
local l0flrr = { mass = 0.630, com = { 0.055, -0.016, 0. }, inertia = l0Inertia }
local l1r = { mass = 0.840, com = { 0.031, -0.0164, -0.003 }, inertia = l1Inertia }
local l1l = { mass = 0.840, com = { 0.031, 0.0164, -0.003 }, inertia = l1Inertia }
local l2r = { mass = 0.137, com = { 0.001, -0.104, -0.003 }, inertia = l2Inertia }
local l2l = { mass = 0.137, com = { 0.001, 0.104, -0.003 }, inertia = l2Inertia }

local bodies = {
    base = base,

    fr_l0 = l0frrl,
    fr_l1 = l1r,
    fr_l2 = l2r,

    fl_l0 = l0flrr,
    fl_l1 = l1l,
    fl_l2 = l2l,

    rl_l0 = l0frrl,
    rl_l1 = l1l,
    rl_l2 = l2l,

    rr_l0 = l0flrr,
    rr_l1 = l1r,
    rr_l2 = l2r
}
local joints = {
    -- Freeflyer joint created following the DoF order convention: xrot, yrot, zrot, xpos, ypos, zpos
    freeflyer = {
        { 1., 0., 0., 0., 0., 0. },
        { 0., 1., 0., 0., 0., 0. },
        { 0., 0., 1., 0., 0., 0. },
        { 0., 0., 0., 1., 0., 0. },
        { 0., 0., 0., 0., 1., 0. },
        { 0., 0., 0., 0., 0., 1. }
    },
    rot_x = {
        { 1., 0., 0., 0., 0., 0. }
    },
    fixed = {},
}
local j2Rot = {
    { 1., 0.,      0. },
    { 0., -0.9731, 0.2305 },
    { 0., -0.2305, -0.9731 }
}

local j1mirrorRot = {
    { 0.,  1., 0. },
    { -1., 0., 0. },
    { 0.,  0., 1. }
}
local j1mirrorTranslation = { 0.0555, 0.0125, 0. }
local j2mirrorTranslation = { 0.0584, -0.2, 0. }

local footRRTranslation = { 0.0054, -0.2, 0. }

local j1t = { 0.0555, -0.0125, 0. }
local j1r = {
    { 0., -1., 0. },
    { 1., 0.,  0. },
    { 0., 0.,  1. }
}
local j2t = { 0.0584, -0.2, 0. }
local j2r = {
    { 1., 0.,      0. },
    { 0., -0.9731, 0.2305 },
    { 0., -0.2305,  -0.9731 }
}
local ffrt = { 0.0054, -0.2, 0. }

-- LEFT
local j1tm = { 0.0555, 0.0125, 0. }
local j1rm = {
    { 0., 1., 0. },
    { -1., 0.,  0. },
    { 0., 0.,  1. }
}
local j2tm = { 0.0584, 0.2, 0. }
local j2rm = {
    { 1., 0.,      0. },
    { 0., -0.9731, -0.2305 },
    { 0., 0.2305, -0.9731 }
}
local fflt = { 0.0054, 0.2, 0. }

local model = {
    gravity = { 0., 0., -9.81 },
    configuration = {
        axis_front = { 1., 0., 0.},
        axis_right = { 0., 1., 0.},
        axis_up    = { 0., 0., 1.}
    },

    frames = {
        {
            name = "base",
            parent = "ROOT",
            body = bodies.base,
            joint = joints.freeflyer
        },
        --leg 0 - RF
        {
            name = "fr_j0",
            parent = "base",
            body = bodies.fr_l0,
            joint = joints.rot_x,
            joint_frame = {
                r = { 0.1375, -0.064, 0. }
            }
        },
        {
            name = "fr_j1",
            parent = "fr_j0",
            body = bodies.fr_l1,
            joint = joints.rot_x,
            joint_frame = {
                r = j1t,
                E = j1r
            }
        },
        {
            name = "fr_j2",
            parent = "fr_j1",
            joint = joints.rot_x,
            body = bodies.fr_l2,
            joint_frame = {
                r = j2t,
                E = j2r
            }
        },
        {
            name = "fr_foot",
            parent = "fr_j2",
            joint = joints.fixed,
            joint_frame = {
                r = ffrt
            }
        },
        --leg 1 - FL
        {
            name = "fl_j0",
            parent = "base",
            body = bodies.fl_l0,
            joint = joints.rot_x,
            joint_frame = {
                r = { 0.1375, 0.064, 0. }
            }
        },
        {
            name = "fl_j1",
            parent = "fl_j0",
            body = bodies.fl_l1,
            joint = joints.rot_x,
            joint_frame = {
                r = j1tm,
                E = j1rm
            }
        },
        {
            name = "fl_j2",
            parent = "fl_j1",
            joint = joints.rot_x,
            body = bodies.fl_l2,
            joint_frame = {
                r = j2tm,
                E = j2rm
            }
        },
        {
            name = "fl_foot",
            parent = "fl_j2",
            joint = joints.fixed,
            joint_frame = {
                r = fflt
            }
        },
        --leg 2 - RL
        {
            name = "rl_j0",
            parent = "base",
            body = bodies.rl_l0,
            joint = joints.rot_x,
            joint_frame = {
                r = { -0.1375, 0.064, 0. },
                E = {
                    { -1., 0.,  0. },
                    { 0.,  -1., 0. },
                    { 0.,  0.,  1. }
                }
            }
        },
        {
            name = "rl_j1",
            parent = "rl_j0",
            body = bodies.rl_l1,
            joint = joints.rot_x,
            joint_frame = {
                r = j1t,
                E = j1r
            }
        },
        {
            name = "rl_j2",
            parent = "rl_j1",
            joint = joints.rot_x,
            body = bodies.rl_l2,
            joint_frame = {
                r = j2tm,
                E = j2rm
            }
        },
        {
            name = "rl_foot",
            parent = "rl_j2",
            joint = joints.fixed,
            joint_frame = {
                r = fflt
            }
        },
        --leg 3 - RR
        {
            name = "rr_j0",
            parent = "base",
            body = bodies.rr_l0,
            joint = joints.rot_x,
            joint_frame = {
                r = { -0.1375, -0.064, 0. },
                E = {
                    { -1., 0.,  0. },
                    { 0.,  -1., 0. },
                    { 0.,  0.,  1. }
                }
            }
        },
        {
            name = "rr_j1",
            parent = "rr_j0",
            body = bodies.rr_l1,
            joint = joints.rot_x,
            joint_frame = {
                r = j1mirrorTranslation,
                E = j1mirrorRot
            }
        },
        {
            name = "rr_j2",
            parent = "rr_j1",
            joint = joints.rot_x,
            body = bodies.rr_l2,
            joint_frame = {
                r = j2mirrorTranslation,
                E = j2Rot
            }
        },
        {
            name = "rr_foot",
            parent = "rr_j2",
            joint = joints.fixed,
            joint_frame = {
                r = footRRTranslation
            }
        }
    }
};
return model;
