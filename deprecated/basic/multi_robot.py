from asyncio import tasks

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from isaacsim.robot.wheeled_robots.controllers.wheel_base_pose_controller import WheelBasePoseController
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from isaacsim.robot.manipulators.examples.franka.controllers import PickPlaceController
from isaacsim.robot.manipulators.examples.franka.tasks import PickPlace
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.api.objects.cuboid import VisualCuboid
from isaacsim.core.api import World
import numpy as np

class RobotPlaying(BaseTask):
    def __init__(self, name, offset=None):
        super().__init__(name=name, offset=offset)
        self._jetbot_goal_position = np.array([np.random.uniform(1.2, 1.6), 0.3, 0.0]) + self._offset
        self._task_event = 0
        self._pick_place_task = PickPlace(
            cube_initial_position=np.array([0.1, 0.3, 0.05]),
            target_position=np.array([0.7, -0.3, 0.0515 / 2.0]),
            offset=offset
        )

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._pick_place_task.set_up_scene(scene)
        jetbot_name = find_unique_string_name(
            "jetbot", is_unique_fn=lambda x: not self.scene.object_exists(x)
        )
        jetbot_prim_path = find_unique_string_name(
            initial_name = "/World/jetbot",
            is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        assets_root_path = get_assets_root_path()
        jetbot_asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
        self._jetbot = scene.add(
            WheeledRobot(
                prim_path=jetbot_prim_path,
                name=jetbot_name,
                wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                create_robot=True,
                usd_path=jetbot_asset_path,
                position=np.array([0.0, 0.3, 0.0])
            )
        )
        self._task_objects[self._jetbot.name] = self._jetbot
        pick_place_params = self._pick_place_task.get_params()
        self._franka = scene.get_object(pick_place_params["robot_name"]["value"])
        current_position, _ = self._franka.get_world_pose()
        self._franka.set_world_pose(position=current_position + np.array([1.0, 0.0, 0.0]))
        self._franka.set_default_state(position=current_position + np.array([1.0, 0.0, 0.0]))
        self._move_task_objects_to_their_frame()
        return

    def get_observations(self):
        current_jetbot_position, current_jetbot_orientation = self._jetbot.get_world_pose()
        observations = {
            self.name + "_envent": self._task_event,
            self._jetbot.name: {
                "position": current_jetbot_position,
                "orientation": current_jetbot_orientation,
                "goal_position": self._jetbot_goal_position
            }
        }
        observations.update(self._pick_place_task.get_observations())
        return observations

    def get_params(self):
        pick_place_params = self._pick_place_task.get_params()
        params_representation = pick_place_params
        params_representation["jetbot_name"] = {
            "value": self._jetbot.name,
            "modifiable": False,
        }
        params_representation["franka_name"] = pick_place_params["robot_name"]
        return params_representation

    def pre_step(self, control_index, simulation_time):
        if self._task_event == 0:
            current_jetbot_position, _ = self._jetbot.get_world_pose()
            if np.mean(np.abs(current_jetbot_position[:2] - self._jetbot_goal_position[:2])) < 0.04:
                self._task_event += 1
                self._cube_arrive_setup_index = control_index
        elif self._task_event == 1:
            if control_index - self._cube_arrive_setup_index == 200:
                self._task_event += 1
        return

    def post_reset(self):
        self._franka.gripper.set_joint_positions(self._franka.gripper.joint_opened_positions)
        self._task_event = 0

world = World()
num_of_tasks = 3
tasks = []
franka_controllers = []
jetbot_controllers = []
frankas = []
jetbots = []
cube_names = []
for i in range(num_of_tasks):
    world.add_task(RobotPlaying(name=f"robot_playing_task_{i}", offset=np.array([0, (i * 2) - 3, 0])))
    tasks.append(world.get_task(f"robot_playing_task_{i}"))

world.reset()

for i in range(num_of_tasks):
    task_params = tasks[i].get_params()
    frankas.append(world.scene.get_object(task_params["franka_name"]["value"]))
    jetbots.append(world.scene.get_object(task_params["jetbot_name"]["value"]))
    cube_names.append(task_params["cube_name"]["value"])
    franka_controllers.append(PickPlaceController(
        name=f"pick_place_controller_{i}",
        gripper=frankas[i].gripper,
        robot_articulation=frankas[i],
        events_dt=[0.008, 0.002, 0.5, 0.1, 0.05, 0.05, 0.0025, 1, 0.008, 0.08]
    ))
    jetbot_controllers.append(WheelBasePoseController(
        name=f"jetbot_controller_{i}",
        open_loop_wheel_controller=DifferentialController(
            name=f"simple_control_{i}",
            wheel_radius=0.03,
            wheel_base=0.1125
        ),
    ))

while simulation_app.is_running():
    world.step(render=True)
    current_observations = world.get_observations()
    for i in range(num_of_tasks):
        if current_observations[tasks[i].name + "_envent"] == 0:
            jetbots[i].apply_wheel_actions(
                jetbot_controllers[i].forward(
                    start_position=current_observations[jetbots[i].name]["position"],
                    start_orientation=current_observations[jetbots[i].name]["orientation"],
                    goal_position=current_observations[jetbots[i].name]["goal_position"]
                )
            )
        elif current_observations[tasks[i].name + "_envent"] == 1:
            jetbots[i].apply_wheel_actions(ArticulationAction(joint_velocities=[-8, -8]))
        elif current_observations[tasks[i].name + "_envent"] == 2:
            jetbots[i].apply_wheel_actions(ArticulationAction(joint_velocities=[0, 0]))
            actions = franka_controllers[i].forward(
                picking_position=current_observations[cube_names[i]]["position"],
                placing_position=current_observations[cube_names[i]]["target_position"],
                current_joint_positions=current_observations[frankas[i].name]["joint_positions"]
            )
            frankas[i].apply_action(actions)
        if franka_controllers[i].is_done():
            print(f"Pick and place task {i} completed!")
    if all(controller.is_done() for controller in franka_controllers):
        print("All tasks completed!")
        break

simulation_app.close()
