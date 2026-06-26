# sim_demo.py
# Franka pick-and-place demo based on NVIDIA's official franka_pick_up.py example.
# Uses SingleManipulator + ParallelGripper — the exact pattern PickPlaceController needs.

import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.examples.franka.controllers.pick_place_controller import PickPlaceController
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.viewports import set_camera_view

print("=" * 60)
print("  Articulated Robot Manipulation — Isaac Sim 5.0 Demo")
print("  Official Pick-and-Place | Franka Panda + Lula IK")
print("=" * 60)

# ── Asset path ────────────────────────────────────────────────────────────────
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    raise SystemExit

# ── World ─────────────────────────────────────────────────────────────────────
my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

# ── Load Franka USD directly from NVIDIA asset server ────────────────────────
asset_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
robot_prim = add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka")

# ── Set up the ParallelGripper exactly as NVIDIA's example does ───────────────
gripper = ParallelGripper(
    end_effector_prim_path="/World/Franka/panda_rightfinger",
    joint_prim_names=["panda_finger_joint1", "panda_finger_joint2"],
    joint_opened_positions=np.array([0.05, 0.05]),
    joint_closed_positions=np.array([0.02, 0.02]),
    action_deltas=np.array([0.01, 0.01]),
)

# ── Wrap in SingleManipulator ─────────────────────────────────────────────────
my_franka = my_world.scene.add(
    SingleManipulator(
        prim_path="/World/Franka",
        name="my_franka",
        end_effector_prim_path="/World/Franka/panda_rightfinger",
        gripper=gripper,
    )
)

# ── Red cube (blue in NVIDIA example — we make it red) ───────────────────────
cube = my_world.scene.add(
    DynamicCuboid(
        name="cube",
        position=np.array([0.3, 0.3, 0.3]),
        prim_path="/World/Cube",
        scale=np.array([0.0515, 0.0515, 0.0515]),
        size=1.0,
        color=np.array([1.0, 0.15, 0.0]),
    )
)

# ── Open gripper at start, then reset ────────────────────────────────────────
my_franka.gripper.set_default_state(my_franka.gripper.joint_opened_positions)
my_world.reset()

# ── Controller ────────────────────────────────────────────────────────────────
my_controller = PickPlaceController(
    name="pick_place_controller",
    gripper=my_franka.gripper,
    robot_articulation=my_franka,
)
articulation_controller = my_franka.get_articulation_controller()

# ── Cinematic camera ──────────────────────────────────────────────────────────
set_camera_view(eye=np.array([1.3, -1.0, 0.9]), target=np.array([0.25, 0.2, 0.3]))

# ── Main loop ─────────────────────────────────────────────────────────────────
reset_needed   = False
task_completed = False
print("[RUNNING] Franka Lula IK solving pick-and-place trajectory...")

while simulation_app.is_running():
    my_world.step(render=True)

    if my_world.is_stopped() and not reset_needed:
        reset_needed   = True
        task_completed = False

    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            my_controller.reset()
            reset_needed   = False
            task_completed = False

        actions = my_controller.forward(
            picking_position=cube.get_local_pose()[0],
            placing_position=np.array([-0.3, -0.3, 0.0515 / 2.0]),
            current_joint_positions=my_franka.get_joint_positions(),
            end_effector_offset=np.array([0, 0.005, 0]),
        )

        if my_controller.is_done() and not task_completed:
            print("[DONE] Pick-and-place complete! Cube transported to drop zone.")
            task_completed = True

        articulation_controller.apply_action(actions)

simulation_app.close()
