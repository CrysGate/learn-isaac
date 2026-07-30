from pydoc import ispackage

from cv2 import ROTATE_90_COUNTERCLOCKWISE

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False })

from isaacsim.core.api import World
from isaacsim.sensors.camera import Camera
from isaacsim.core.api.objects import DynamicCuboid
import isaacsim.core.utils.numpy.rotations as rot_utils
import numpy as np
import matplotlib.pyplot as plt

world = World(stage_units_in_meters=1.0)
cube_1 = world.scene.add(
    DynamicCuboid(
        prim_path="/Cube_1",
        name="Cube_1",
        position=np.array([5.0, 3, 1.0]),
        scale=np.array([0.6, 0.5, 0.2]),
        size=1.0,
        color=np.array([255, 0, 0])
    )
)
cube_2 = world.scene.add(
    DynamicCuboid(
        prim_path="/Cube_2",
        name="Cube_2",
        position=np.array([-5.0, 1, 3.0]),
        scale=np.array([0.1, 0.1, 0.1]),
        size=1.0,
        color=np.array([0, 0, 255]),
        linear_velocity=np.array([0.0, 0.0, 0.4]),
    )
)
camera = Camera(
    prim_path="/World/Camera",
    position=np.array([0.0, 0.0, 25.0]),
    frequency=20,
    resolution=(256, 256),
    orientation=rot_utils.euler_angles_to_quats(
        np.array([0, 90, 0]),
        degrees=True
    )
)


world.scene.add_default_ground_plane()
world.reset()
camera.initialize()
camera.add_motion_vectors_to_frame()

i = 0
while simulation_app.is_running():
    world.step(render=True)
    print(camera.get_current_frame())
    if i == 100:
        points_2d = camera.get_image_coords_from_world_points(
            np.array([
                cube_2.get_world_pose()[0],
                cube_1.get_world_pose()[0]
            ])
        )
        points_3d = camera.get_world_points_from_image_coords(
            points_2d,
            np.array([24.94, 24.94])
        )
        print("2D points: ", points_2d)
        print("3D points: ", points_3d)
        imgplot = plt.imshow(camera.get_rgb()[:, :, :3])
        plt.show()
        print("motion vectors: ", camera.get_current_frame()["motion_vectors"])
    if world.is_playing():
        if world.current_time_step_index == 0:
            world.reset()
    i += 1

simulation_app.close()
