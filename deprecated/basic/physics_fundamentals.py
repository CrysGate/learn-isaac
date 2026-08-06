"""
Isaac Sim 5.1.0 Physics Fundamentals Demo

这个示例展示：

1. 只有视觉，没有物理
2. 只有 Rigid Body，没有 Collider
3. 同时具有 Rigid Body 和 Collider
4. 只有 Collider 的静态平台
5. Physics Material：摩擦力和弹性
6. Physics Scene：重力、物理频率、CCD
7. Revolute Joint：旋转关节
8. Joint Drive：位置控制
9. Articulation Root：机器人关节树
10. Physics Step 和 Render Update 的区别
"""

# ============================================================
# 1. 必须首先启动 Isaac Sim
# ============================================================

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# ============================================================
# 2. Isaac Sim 启动后，才能导入这些模块
# ============================================================

import numpy as np
import omni.physx
import omni.timeline
import omni.usd

from isaacsim.core.utils.viewports import set_camera_view
from omni.physx.scripts import physicsUtils
from pxr import (
    Gf,
    PhysxSchema,
    PhysicsSchemaTools,
    Sdf,
    Usd,
    UsdGeom,
    UsdLux,
    UsdPhysics,
    UsdShade,
)

# ============================================================
# 3. 创建新的 USD Stage
# ============================================================

usd_context = omni.usd.get_context()
usd_context.new_stage()

stage = usd_context.get_stage()

# 使用 Z 轴作为竖直方向
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

# 一个 USD 长度单位等于一米
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

# 创建 /World 根节点
UsdGeom.Xform.Define(stage, "/World")

# ============================================================
# 4. 创建 Physics Scene
# ============================================================

physics_scene = UsdPhysics.Scene.Define(
    stage,
    "/World/PhysicsScene",
)

# 重力方向：Z 轴负方向
physics_scene.CreateGravityDirectionAttr().Set(
    Gf.Vec3f(0.0, 0.0, -1.0)
)

# 重力大小：9.81 m/s²
physics_scene.CreateGravityMagnitudeAttr().Set(9.81)

# 给 Physics Scene 添加 PhysX 专有配置
physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(
    physics_scene.GetPrim()
)

# 每秒进行 120 次物理计算
PHYSICS_FPS = 120
physx_scene_api.CreateTimeStepsPerSecondAttr().Set(PHYSICS_FPS)

# 在整个 Physics Scene 中允许使用 CCD
physx_scene_api.CreateEnableCCDAttr().Set(True)

print("\n========== Physics Scene ==========")
print("Physics FPS:", PHYSICS_FPS)
print("Physics dt:", 1.0 / PHYSICS_FPS)
print("Gravity:", physics_scene.GetGravityMagnitudeAttr().Get())


# ============================================================
# 5. 创建地面
# ============================================================

PhysicsSchemaTools.addGroundPlane(
    stage,
    "/World/GroundPlane",
    "Z",                         # 地面法线方向
    20.0,                        # 地面大小
    Gf.Vec3f(0.0, 0.0, 0.0),    # 地面位置
    Gf.Vec3f(0.5, 0.5, 0.5),    # 显示颜色
)


# ============================================================
# 6. 创建简单光源
# ============================================================

light = UsdLux.DistantLight.Define(
    stage,
    "/World/Light",
)

light.CreateIntensityAttr().Set(3000.0)

UsdGeom.Xformable(light.GetPrim()).AddRotateXYZOp().Set(
    Gf.Vec3f(315.0, 0.0, 0.0)
)


# ============================================================
# 7. 一些简单的辅助函数
# ============================================================

def create_cube(
    path: str,
    position: tuple,
    scale: tuple,
    color: tuple,
):
    """
    只创建一个视觉 Cube。

    注意：
    此时它没有 RigidBodyAPI，也没有 CollisionAPI。
    """

    cube = UsdGeom.Cube.Define(stage, path)

    # Cube 原始边长设为 1 米
    cube.CreateSizeAttr().Set(1.0)

    xformable = UsdGeom.Xformable(cube.GetPrim())

    xformable.AddTranslateOp().Set(
        Gf.Vec3d(*position)
    )

    xformable.AddScaleOp().Set(
        Gf.Vec3f(*scale)
    )

    cube.CreateDisplayColorAttr().Set(
        [Gf.Vec3f(*color)]
    )

    return cube


def create_sphere(
    path: str,
    position: tuple,
    radius: float,
    color: tuple,
):
    """只创建一个视觉 Sphere。"""

    sphere = UsdGeom.Sphere.Define(stage, path)

    sphere.CreateRadiusAttr().Set(radius)

    xformable = UsdGeom.Xformable(sphere.GetPrim())

    xformable.AddTranslateOp().Set(
        Gf.Vec3d(*position)
    )

    sphere.CreateDisplayColorAttr().Set(
        [Gf.Vec3f(*color)]
    )

    return sphere


def add_rigid_body(
    prim,
    mass: float = 1.0,
    add_collider: bool = True,
):
    """
    给 Prim 添加刚体。

    add_collider=False:
        物体会受到重力，但不会发生碰撞。

    add_collider=True:
        物体既能运动，也能碰撞。
    """

    # 给 Prim 应用 RigidBodyAPI
    rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(prim)

    rigid_body_api.CreateRigidBodyEnabledAttr().Set(True)

    # 创建初始速度属性
    rigid_body_api.CreateVelocityAttr().Set(
        Gf.Vec3f(0.0, 0.0, 0.0)
    )

    # 设置质量
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr().Set(mass)

    if add_collider:
        collision_api = UsdPhysics.CollisionAPI.Apply(prim)
        collision_api.CreateCollisionEnabledAttr().Set(True)

    return rigid_body_api


def add_static_collider(prim):
    """
    只添加 Collider，不添加 Rigid Body。

    这种物体不会受到重力，通常作为：
    地面、墙、桌子、固定平台。
    """

    collision_api = UsdPhysics.CollisionAPI.Apply(prim)
    collision_api.CreateCollisionEnabledAttr().Set(True)

    return collision_api


def get_world_position(path: str):
    """读取一个 Prim 当前的世界坐标。"""

    prim = stage.GetPrimAtPath(path)

    transform = UsdGeom.Xformable(
        prim
    ).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )

    position = transform.ExtractTranslation()

    return tuple(round(float(value), 2) for value in position)


# ============================================================
# 8. 实验一：只有视觉，没有物理
# ============================================================

visual_only = create_cube(
    path="/World/VisualOnly",
    position=(-4.0, 0.0, 3.0),
    scale=(0.7, 0.7, 0.7),
    color=(1.0, 1.0, 0.0),
)

# 不添加 RigidBodyAPI
# 不添加 CollisionAPI
#
# 结果：
# 它会一直停在空中。


# ============================================================
# 9. 实验二：只有 Rigid Body，没有 Collider
# ============================================================

rigid_only = create_cube(
    path="/World/RigidOnly",
    position=(-2.0, 0.0, 3.0),
    scale=(0.7, 0.7, 0.7),
    color=(1.0, 0.1, 0.1),
)

add_rigid_body(
    prim=rigid_only.GetPrim(),
    mass=1.0,
    add_collider=False,
)

# 结果：
# 它会受到重力并下落。
#
# 但是，因为没有 CollisionAPI，
# 它会直接穿过地面。


# ============================================================
# 10. 实验三：Rigid Body + Collider
# ============================================================

normal_dynamic_cube = create_cube(
    path="/World/RigidWithCollider",
    position=(0.0, 0.0, 3.0),
    scale=(0.7, 0.7, 0.7),
    color=(0.1, 1.0, 0.1),
)

normal_cube_rigid_api = add_rigid_body(
    prim=normal_dynamic_cube.GetPrim(),
    mass=1.0,
    add_collider=True,
)

# 给刚体添加 PhysX 专有 API
normal_cube_physx_api = PhysxSchema.PhysxRigidBodyAPI.Apply(
    normal_dynamic_cube.GetPrim()
)

# 对这个刚体启用连续碰撞检测
normal_cube_physx_api.CreateEnableCCDAttr().Set(True)

# 结果：
# 它受到重力。
# 它有碰撞形状。
# 因此最终停在地面上。


# ============================================================
# 11. 实验四：只有 Collider 的静态平台
# ============================================================

platform = create_cube(
    path="/World/StaticPlatform",
    position=(2.0, 0.0, 1.0),
    scale=(1.5, 1.5, 0.2),
    color=(0.4, 0.4, 0.4),
)

add_static_collider(platform.GetPrim())

# 平台上方再放一个动态方块
platform_cube = create_cube(
    path="/World/PlatformCube",
    position=(2.0, 0.0, 4.0),
    scale=(0.6, 0.6, 0.6),
    color=(0.1, 0.5, 1.0),
)

add_rigid_body(
    prim=platform_cube.GetPrim(),
    mass=1.0,
    add_collider=True,
)

# 结果：
# StaticPlatform 不会下落，因为它没有 RigidBodyAPI。
# PlatformCube 会下落，并停在平台上。


# ============================================================
# 12. 实验五：Physics Material
# ============================================================

# 创建一个 Physics Material
bouncy_material = UsdShade.Material.Define(
    stage,
    "/World/Materials/BouncyMaterial",
)

material_api = UsdPhysics.MaterialAPI.Apply(
    bouncy_material.GetPrim()
)

# 静摩擦
material_api.CreateStaticFrictionAttr().Set(0.2)

# 动摩擦
material_api.CreateDynamicFrictionAttr().Set(0.1)

# 恢复系数，越大越容易反弹
material_api.CreateRestitutionAttr().Set(0.9)

bouncy_sphere = create_sphere(
    path="/World/BouncySphere",
    position=(4.0, 0.0, 4.0),
    radius=0.45,
    color=(0.7, 0.2, 1.0),
)

add_rigid_body(
    prim=bouncy_sphere.GetPrim(),
    mass=1.0,
    add_collider=True,
)

# 把物理材质绑定到 Sphere 的 Collider
physicsUtils.add_physics_material_to_prim(
    stage,
    bouncy_sphere.GetPrim(),
    "/World/Materials/BouncyMaterial",
)

# 结果：
# 紫色球落地后会明显反弹。


# ============================================================
# 13. 实验六：创建一个最简单的机器人 Articulation
# ============================================================

robot_path = "/World/SimpleRobot"

robot_root = UsdGeom.Xform.Define(
    stage,
    robot_path,
)

# ArticulationRootAPI 表示：
# 这个节点下面是一棵机器人关节树。
UsdPhysics.ArticulationRootAPI.Apply(
    robot_root.GetPrim()
)


# ------------------------------------------------------------
# 13.1 机器人 Base Link
# ------------------------------------------------------------

base_path = f"{robot_path}/BaseLink"

base_link = create_cube(
    path=base_path,
    position=(0.0, -4.0, 0.5),
    scale=(1.0, 1.0, 1.0),
    color=(0.2, 0.2, 0.2),
)

add_rigid_body(
    prim=base_link.GetPrim(),
    mass=10.0,
    add_collider=True,
)


# ------------------------------------------------------------
# 13.2 使用 Fixed Joint 把 Base 固定到世界
# ------------------------------------------------------------

fixed_joint = UsdPhysics.FixedJoint.Define(
    stage,
    f"{robot_path}/FixedBaseJoint",
)

# Body 0 不设置，表示连接到世界。
fixed_joint.CreateBody1Rel().SetTargets(
    [Sdf.Path(base_path)]
)

# 世界坐标中的固定点
fixed_joint.CreateLocalPos0Attr().Set(
    Gf.Vec3f(0.0, -4.0, 0.5)
)

# 相对于 BaseLink 中心的位置
fixed_joint.CreateLocalPos1Attr().Set(
    Gf.Vec3f(0.0, 0.0, 0.0)
)


# ------------------------------------------------------------
# 13.3 机器人 Arm Link
# ------------------------------------------------------------

arm_path = f"{robot_path}/ArmLink"

arm_link = create_cube(
    path=arm_path,
    position=(0.0, -4.0, 2.0),
    scale=(0.35, 0.35, 2.0),
    color=(1.0, 0.5, 0.1),
)

add_rigid_body(
    prim=arm_link.GetPrim(),
    mass=1.0,
    add_collider=True,
)


# ------------------------------------------------------------
# 13.4 创建旋转关节
# ------------------------------------------------------------

joint_path = f"{robot_path}/ArmJoint"

arm_joint = UsdPhysics.RevoluteJoint.Define(
    stage,
    joint_path,
)

# Body 0 是父 Link
arm_joint.CreateBody0Rel().SetTargets(
    [Sdf.Path(base_path)]
)

# Body 1 是子 Link
arm_joint.CreateBody1Rel().SetTargets(
    [Sdf.Path(arm_path)]
)

# 关节在 BaseLink 顶部
arm_joint.CreateLocalPos0Attr().Set(
    Gf.Vec3f(0.0, 0.0, 0.5)
)

# 关节在 ArmLink 底部
arm_joint.CreateLocalPos1Attr().Set(
    Gf.Vec3f(0.0, 0.0, -1.0)
)

# 绕关节局部 Y 轴旋转
arm_joint.CreateAxisAttr().Set("Y")

# USD 旋转关节限制使用度
arm_joint.CreateLowerLimitAttr().Set(-60.0)
arm_joint.CreateUpperLimitAttr().Set(60.0)


# ------------------------------------------------------------
# 13.5 给关节添加 Drive
# ------------------------------------------------------------

drive_api = UsdPhysics.DriveAPI.Apply(
    arm_joint.GetPrim(),
    "angular",
)

# 类似位置控制中的 Kp
drive_api.CreateStiffnessAttr().Set(200.0)

# 类似位置控制中的 Kd
drive_api.CreateDampingAttr().Set(20.0)

# 最大输出力矩
drive_api.CreateMaxForceAttr().Set(1000.0)

# 初始目标角度
target_position_attr = drive_api.CreateTargetPositionAttr()
target_position_attr.Set(45.0)


# ============================================================
# 14. 检查 Schema 是否真的应用到了 Prim
# ============================================================

print("\n========== Schema Check ==========")

print(
    "VisualOnly has RigidBodyAPI:",
    bool(UsdPhysics.RigidBodyAPI(visual_only.GetPrim())),
)

print(
    "VisualOnly has CollisionAPI:",
    bool(UsdPhysics.CollisionAPI(visual_only.GetPrim())),
)

print(
    "RigidOnly has RigidBodyAPI:",
    bool(UsdPhysics.RigidBodyAPI(rigid_only.GetPrim())),
)

print(
    "RigidOnly has CollisionAPI:",
    bool(UsdPhysics.CollisionAPI(rigid_only.GetPrim())),
)

print(
    "RigidWithCollider has RigidBodyAPI:",
    bool(UsdPhysics.RigidBodyAPI(normal_dynamic_cube.GetPrim())),
)

print(
    "RigidWithCollider has CollisionAPI:",
    bool(UsdPhysics.CollisionAPI(normal_dynamic_cube.GetPrim())),
)

print(
    "SimpleRobot has ArticulationRootAPI:",
    bool(UsdPhysics.ArticulationRootAPI(robot_root.GetPrim())),
)

print(
    "ArmJoint target position:",
    target_position_attr.Get(),
)


# ============================================================
# 15. 设置观察相机
# ============================================================

simulation_app.update()

set_camera_view(
    eye=np.array([11.0, 11.0, 8.0]),
    target=np.array([0.0, -1.0, 1.5]),
)


# ============================================================
# 16. Physics Step 回调
# ============================================================

state = {
    "physics_steps": 0,
    "render_updates": 0,
    "simulation_time": 0.0,
    "last_target": None,
}


def on_physics_step(dt: float):
    """
    每个 Physics Step 调用一次。

    物理频率是 120 Hz，因此理论上每秒调用 120 次。
    """

    state["physics_steps"] += 1
    state["simulation_time"] += dt

    simulation_time = state["simulation_time"]

    # 每三秒切换一次关节目标
    phase = int(simulation_time // 3.0) % 2

    if phase == 0:
        target = 45.0
    else:
        target = -45.0

    if target != state["last_target"]:
        target_position_attr.Set(target)
        state["last_target"] = target

        print(
            f"\nJoint target changed to {target} degrees"
        )

    # 每 120 个物理步打印一次，约等于一仿真秒
    if state["physics_steps"] % PHYSICS_FPS == 0:
        print("\n========== Simulation State ==========")

        print(
            "Simulation time:",
            round(simulation_time, 2),
            "seconds",
        )

        print(
            "Physics steps:",
            state["physics_steps"],
        )

        print(
            "Render updates:",
            state["render_updates"],
        )

        print(
            "VisualOnly position:",
            get_world_position("/World/VisualOnly"),
        )

        print(
            "RigidOnly position:",
            get_world_position("/World/RigidOnly"),
        )

        print(
            "RigidWithCollider position:",
            get_world_position("/World/RigidWithCollider"),
        )

        print(
            "PlatformCube position:",
            get_world_position("/World/PlatformCube"),
        )

        print(
            "BouncySphere position:",
            get_world_position("/World/BouncySphere"),
        )


# 订阅 Physics Step 事件
physics_subscription = (
    omni.physx.get_physx_interface()
    .subscribe_physics_step_events(on_physics_step)
)


# ============================================================
# 17. 启动 Timeline
# ============================================================

RENDER_FPS = 60

timeline = omni.timeline.get_timeline_interface()

# USD 时间轴每秒 60 个时间码
timeline.set_time_codes_per_second(RENDER_FPS)

# 每个应用更新推进一个时间码
timeline.set_ticks_per_frame(1)

timeline.play()

print("\n========== Demo Started ==========")
print("Yellow cube: visual only")
print("Red cube: rigid body without collider")
print("Green cube: rigid body with collider")
print("Blue cube: falling onto static platform")
print("Purple sphere: bouncy physics material")
print("Orange arm: revolute joint and position drive")
print("Close the Isaac Sim window or press Ctrl+C to exit.\n")


# ============================================================
# 18. 主循环
# ============================================================

try:
    while simulation_app.is_running():
        state["render_updates"] += 1

        # 推进应用、渲染和 Timeline
        simulation_app.update()

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    timeline.stop()

    # 释放回调引用
    physics_subscription = None

    simulation_app.close()
