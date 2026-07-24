from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers import PickPlaceController
import numpy as np

world = World()
world.scene.add_default_ground_plane()
franka = world.scene.add(Franka(prim_path="/World/franka", name="franka"))
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/cube",
        name="cube",
        position=np.array([0.3, 0.3, 0.3]),
        scale=np.array([0.0515, 0.0515, 0.0515]),
        color=np.array([0, 0, 1.0]),
    )
)

world.reset()

franka_controller = PickPlaceController(
    name="pick_place_controller",
    gripper=franka.gripper,
    robot_articulation=franka,
)
franka.gripper.set_joint_positions(franka.gripper.joint_opened_positions)
goal_position = np.array([-0.3, -0.3, 0.0515 / 2.0])

while simulation_app.is_running():
    world.step(render=True)
    cube_postion, _ = cube.get_world_pose()
    current_joint_positions = franka.get_joint_positions()
    actions = franka_controller.forward(
        picking_position=cube_postion,
        placing_position=goal_position,
        current_joint_positions=current_joint_positions,
    )
    franka.apply_action(actions)
    if franka_controller.is_done():
        print("Pick and place task completed!")
        break

simulation_app.close()
