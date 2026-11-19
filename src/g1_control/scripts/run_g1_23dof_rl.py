import os, glob, socket, struct, torch
import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.usd, omni.timeline, omni.physx
from pxr import UsdPhysics, PhysxSchema
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.prims import Articulation

enable_extension("isaacsim.ros2.bridge")

candidates = [
    os.path.expanduser("~/g1_ws/src/g1_description/usd/g1_23dof_grounded.usd"),
    os.path.expanduser("~/g1_ws/src/g1_description/usd/g1_23dof_grounded.usda"),
] + glob.glob(os.path.expanduser("~/**/g1_23dof*.usd*"), recursive=True)

usd_path = next((p for p in candidates if os.path.exists(p)), None)
if not usd_path:
    raise FileNotFoundError("Scène g1_23dof*.usd introuvable !")

print(f"[INFO] Ouverture de : {usd_path}")
omni.usd.get_context().open_stage(usd_path)
for _ in range(5):
    simulation_app.update()

stage = omni.usd.get_context().get_stage()
for prim in stage.Traverse():
    if prim.IsA(UsdPhysics.Scene):
        PhysxSchema.PhysxSceneAPI.Apply(prim).CreateTimeStepsPerSecondAttr(200)

# Recherche robuste de l'ArticulationRootAPI dans le stage
art_paths = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
print(f"[INFO] Articulations détectées dans la scène : {art_paths}")

if not art_paths:
    # Si l'ArticulationRootAPI est sur le prim parent /g1_23dof ou /World/g1_23dof
    for candidate_path in ["/g1_23dof", "/World/g1_23dof", "/g1_23dof/pelvis", "/World/g1_23dof/pelvis"]:
        prim = stage.GetPrimAtPath(candidate_path)
        if prim.IsValid():
            UsdPhysics.ArticulationRootAPI.Apply(prim)
            art_paths = [candidate_path]
            print(f"[INFO] ArticulationRootAPI appliqué sur : {candidate_path}")
            break

if not art_paths:
    raise RuntimeError("Impossible de trouver le robot G1 dans le stage USD.")

robot_prim_path = art_paths[0]

timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(10):
    simulation_app.update()

robot = Articulation(prim_paths_expr=robot_prim_path, name="g1_23dof_ctrl")
robot.initialize()

leg_names = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
]
dof_names = list(robot.dof_names)
print(f"[INFO] {robot.num_dof} DOFs trouvés : {dof_names}")
leg_idx = [dof_names.index(n) for n in leg_names]

default_pos = np.zeros(robot.num_dof, dtype=np.float32)
for i, a in zip(leg_idx, [-0.1, 0.0, 0.0, 0.3, -0.2, 0.0, -0.1, 0.0, 0.0, 0.3, -0.2, 0.0]):
    default_pos[i] = a

kps = np.full(robot.num_dof, 100.0, dtype=np.float32)
kds = np.full(robot.num_dof, 2.0, dtype=np.float32)
for i in [leg_idx[3], leg_idx[9]]: kps[i], kds[i] = 150.0, 4.0
for i in [leg_idx[4], leg_idx[5], leg_idx[10], leg_idx[11]]: kps[i], kds[i] = 40.0, 2.0
robot.set_gains(kps=kps, kds=kds)

device = "cuda" if torch.cuda.is_available() else "cpu"
policy = torch.jit.load(os.path.expanduser("~/unitree_rl_gym/deploy/pre_train/g1/motion.pt"), map_location=device).eval()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("127.0.0.1", 9870))
sock.setblocking(False)

cmd_vel = np.zeros(3, dtype=np.float32)
action = np.zeros(12, dtype=np.float32)
target_pos = default_pos.copy()
step = 0

def quat_rotate_inverse(q, v):
    qw, qx, qy, qz = q
    q_vec = np.array([qx, qy, qz], dtype=np.float32)
    return v * (2.0 * qw * qw - 1.0) - np.cross(q_vec, v) * qw * 2.0 + q_vec * np.dot(q_vec, v) * 2.0

def on_step(dt):
    global action, step
    try:
        while True:
            vx, vy, vyaw = struct.unpack("fff", sock.recvfrom(1024)[0])
            cmd_vel[:] = [np.clip(vx, -1.0, 1.0), np.clip(vy, -0.5, 0.5), np.clip(vyaw, -1.0, 1.0)]
    except BlockingIOError:
        pass

    if step % 4 == 0:
        q = robot.get_joint_positions()[0]
        dq = robot.get_joint_velocities()[0]
        quat = robot.get_world_poses()[1][0]
        omega_world = robot.get_angular_velocities()[0]

        omega_local = quat_rotate_inverse(quat, omega_world) * 0.25
        grav = quat_rotate_inverse(quat, np.array([0.0, 0.0, -1.0], dtype=np.float32))
        qj = np.array([q[i] - default_pos[i] for i in leg_idx], dtype=np.float32)
        dqj = np.array([dq[i] * 0.05 for i in leg_idx], dtype=np.float32)
        phase = ((step * 0.005) % 0.8) / 0.8

        obs = np.concatenate([
            omega_local, grav, cmd_vel * [2.0, 2.0, 0.25],
            qj, dqj, action, [np.sin(2*np.pi*phase), np.cos(2*np.pi*phase)]
        ]).astype(np.float32)

        with torch.no_grad():
            action = policy(torch.from_numpy(obs).unsqueeze(0).to(device)).cpu().numpy().squeeze()

        for i, idx in enumerate(leg_idx):
            target_pos[idx] = action[i] * 0.25 + default_pos[idx]

    robot.set_joint_position_targets(target_pos)
    step += 1

sub = omni.physx.get_physx_interface().subscribe_physics_step_events(on_step)
print("[SUCCÈS] Contrôleur RL G1 23 DOF actif à 200 Hz !")

while simulation_app.is_running():
    simulation_app.update()

sock.close()
simulation_app.close()
