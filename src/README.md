# Simulation

## Lancer la téléopération

1. Lancer IsaacSim

``` bash
cd isaacsim
./isaac-sim.selector.sh

#Appuyer sur start
```
Puis ouvrir le fichier .usd

2. Dans IsaacSim, sous window > script editor, il faut coller:

``` bash
import os, socket, struct, torch, asyncio
import numpy as np
import omni.usd, omni.kit.app, omni.timeline, omni.physx
from pxr import UsdPhysics, Sdf
from isaacsim.core.prims import Articulation

async def start_g1_controller():
    stage = omni.usd.get_context().get_stage()
    robot_prim_path = next((str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)), None)
    if not robot_prim_path:
        print("[ERREUR] Aucune articulation trouvée !")
        return

    # 1. Régler le pas de temps PhysX à 200 Hz (dt = 0.005s) sur UsdPhysics.Scene
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            prim.CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.UInt).Set(200)
            print(f"[INFO] PhysicsScene réglée à 200 Hz sur {prim.GetPath()}")

    # 2. Nettoyer tout ancien callback et ancien socket UDP
    if hasattr(omni, "_g1_physx_sub") and omni._g1_physx_sub is not None:
        omni._g1_physx_sub.unsubscribe()
        omni._g1_physx_sub = None
    if hasattr(omni, "_g1_sock") and omni._g1_sock:
        omni._g1_sock.close()

    # 3. Démarrer la simulation et attendre la création de la vue physique PhysX
    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        timeline.play()

    app = omni.kit.app.get_app()
    for _ in range(15):
        await app.next_update_async()

    # Instancier Articulation SANS appeler .initialize() manuellement (géré automatiquement par PhysX)
    robot = None
    for _ in range(20):
        try:
            cand = Articulation(prim_paths_expr=robot_prim_path, name="g1_23dof_ctrl")
            if cand.is_physics_handle_valid():
                robot = cand
                break
        except Exception:
            pass
        await app.next_update_async()

    if robot is None:
        print(f"[ERREUR] Impossible d'attacher la vue physique sur {robot_prim_path}. Vérifiez que le robot n'a pas 'Fix Base Link' activé.")
        return

    leg_names = [
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    ]
    dof_names = list(robot.dof_names)
    leg_idx = [dof_names.index(n) for n in leg_names]

    default_pos = np.zeros(robot.num_dof, dtype=np.float32)
    for i, a in zip(leg_idx, [-0.1, 0.0, 0.0, 0.3, -0.2, 0.0, -0.1, 0.0, 0.0, 0.3, -0.2, 0.0]):
        default_pos[i] = a

    # Gains officiels Unitree G1 (stables à 200 Hz)
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
    omni._g1_sock = sock

    cmd_vel = np.zeros(3, dtype=np.float32)
    action = np.zeros(12, dtype=np.float32)
    target_pos = default_pos.copy()
    step = 0

    def quat_rotate_inverse(q, v):
        qw, qx, qy, qz = q
        q_vec = np.array([qx, qy, qz], dtype=np.float32)
        a = v * (2.0 * qw * qw - 1.0)
        b = np.cross(q_vec, v) * qw * 2.0
        c = q_vec * np.dot(q_vec, v) * 2.0
        return a - b + c

    def on_physics_step(dt):
        nonlocal action, step
        if not timeline.is_playing() or not robot.is_physics_handle_valid():
            return
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
            grav_local = quat_rotate_inverse(quat, np.array([0.0, 0.0, -1.0], dtype=np.float32))

            qj = np.array([q[i] - default_pos[i] for i in leg_idx], dtype=np.float32)
            dqj = np.array([dq[i] * 0.05 for i in leg_idx], dtype=np.float32)
            phase = ((step * 0.005) % 0.8) / 0.8

            obs = np.concatenate([
                omega_local,
                grav_local,
                cmd_vel * np.array([2.0, 2.0, 0.25], dtype=np.float32),
                qj,
                dqj,
                action,
                np.array([np.sin(2 * np.pi * phase), np.cos(2 * np.pi * phase)], dtype=np.float32)
            ]).astype(np.float32)

            with torch.no_grad():
                action = policy(torch.from_numpy(obs).unsqueeze(0).to(device)).cpu().numpy().squeeze()
            for i, idx in enumerate(leg_idx):
                target_pos[idx] = action[i] * 0.25 + default_pos[idx]

        robot.set_joint_position_targets(target_pos)
        step += 1

    omni._g1_physx_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(on_physics_step)
    print(f"[SUCCÈS] Contrôleur RL G1 23 DOF actif à 200 Hz sur {robot_prim_path} !")

asyncio.ensure_future(start_g1_controller())
```

Puis appuyer sur run.

3. Dans deux terminales différentes :

Terminale 1: ROS 2 Bridge

Activer le pont vers ROS 2

``` bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 ~/g1_ws/src/g1_control/scripts/ros2_cmd_bridge.py
```

Terminale 2: Teleop_key

Controle du robot avec le clavier

``` bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Intégration des capteurs

Pour verifier les topics

``` bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

Pour afficher les données une fois

``` bash
ros2 topic echo topic_name --once
```

Pour visualiser les données

``` bash
rviz2

# Aller dans Add > Topic puis choisir le nom du topic
# Il faut changer le FrameID
```

1. Camera D435i

Ajouter la camera:
Create > Camera
Puis l'inclure dans torso_link > d435_link d435

Ajouter la brique ros2:
Cliquez sur la camera, puis Tools > Robotics > ROS 2 OmniGraphs

2. IMU

Ajouter l'IMU:
Create > Sensors > Imu Sensor
Puis l'inclure dans torso_link > imu_in_torso

Ajouter la brique ros2:
Clic droit sur le capteur, puis Create > Visual Scripting > Action Graph

3. Lidar

Ajouter le lidar pour 2D:
Create > Sensors > PhysX Lidar > Rotating
Puis l'inclure dans torso_link > mid360_link

Ajouter le lidar pour 3D:
Create > Sensors > RTX Lidar > NVIDIA > Example Rotary
Puis l'inclure dans torso_link > mid360_link

Ajouter la brique ROS2:
Clic droit sur le capteur, puis Create > Visual Scripting > Action Graph


