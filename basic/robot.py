from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api.robots import Robot
from isaacsim.core.api.controllers import BaseController
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.wheeled_robots.controllers.wheel_base_pose_controller import WheelBasePoseController
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController

import numpy as np

class CoolController(BaseController):
    def __init__(self):
        super().__init__(name="my_cool_controller")
        self._wheel_radius = 0.03
        self._wheel_base = 0.1125
        return

    def forward(self, command):
        joint_velocities = [0.0, 0.0]
        joint_velocities[0] = ((2 * command[0]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[1] = ((2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        # A controller has to return an ArticulationAction
        return ArticulationAction(joint_velocities=joint_velocities)

world = World()
world.scene.add_default_ground_plane()

assets_root_path = get_assets_root_path()
asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
# add_reference_to_stage(usd_path=asset_path, prim_path="/World/jetbot")

# jetbot_robot = world.scene.add(Robot(prim_path="/World/jetbot", name="jetbot"))
jetbot_robot = world.scene.add(
    WheeledRobot(
        prim_path="/World/jetbot",
        name="jetbot",
        wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
        create_robot=True,
        usd_path=asset_path
    )
)
turn_duration = 5.0
turn_step = int(turn_duration / world.get_physics_dt())
# my_controller = CoolController()
my_controller = WheelBasePoseController(
    name="cool_controller",
    open_loop_wheel_controller=DifferentialController(
        name="simple_control",
        wheel_radius=0.03,
        wheel_base=0.1125
    ),
    is_holonomic=False
)

world.reset()
for i in range(5000):
    print("Num of degrees of freedom: ", jetbot_robot.num_dof)
    print("Joint positions: ", jetbot_robot.get_joint_positions())
    position, orientation = jetbot_robot.get_world_pose()
    # if i < turn_step:
    #     wheel_velocities = 5 * np.random.rand(2,)
    # else:
    #     wheel_velocities = np.zeros(2,)
    # jetbot_robot.get_articulation_controller().apply_action(
    #     ArticulationAction(
    #         joint_positions=None,
    #         joint_efforts=None,
    #         joint_velocities=wheel_velocities
    #     )
    # )
    # jetbot_robot.apply_wheel_actions(
    #     my_controller.forward(command=[0.2, np.pi / 4])
    # )
    jetbot_robot.apply_action(
        my_controller.forward(
            start_position=position,
            start_orientation=orientation,
            goal_position=np.array([0.8,2.4])
        )
    )
    world.step(render=True)

simulation_app.close()
