import time

import numpy as np
import sapien.core as sapien
from sapien.utils.viewer import Viewer


def demo(fix_root_link, balance_passive_force):
    engine = sapien.Engine()
    renderer = sapien.SapienRenderer()
    engine.set_renderer(renderer)

    scene_config = sapien.SceneConfig()
    scene = engine.create_scene(scene_config)
    scene.set_timestep(1 / 500.0)
    scene.add_ground(0)

    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])

    viewer = Viewer(renderer)
    viewer.set_scene(scene)
    viewer.set_camera_xyz(x=-2, y=0, z=1)
    viewer.set_camera_rpy(r=0, p=-0.3, y=0)

    # Load URDF
    loader: sapien.URDFLoader = scene.create_urdf_loader()
    loader.fix_root_link = fix_root_link
    loader.load_multiple_collisions_from_file = True

    robot: sapien.Articulation = loader.load(
        "ManiSkill2_real2sim/mani_skill2_real2sim/assets/descriptions/ada_description/robots_urdf/ada.urdf"
    )
    print(robot.get_links())
    robot.set_root_pose(sapien.Pose([0, 0, 0.2], [1, 0, 0, 0]))
    print([x.name for x in robot.get_active_joints()])
    print(robot.get_qlimits())

    # Set initial joint positions
    # qpos = np.array([-1.5, 3.22, 1.23, -2.19, 1.8, 1.2, 0.0, 0.0])
    qpos = np.array([-1.5, 3.22, 1.23, -2.19, 1.8, 1.2, 1.0, 1.0])
    robot.set_qpos(qpos)
    for joint in robot.get_active_joints():
        joint.set_drive_property(stiffness=1e5, damping=1e3)

    while not viewer.closed:
        print(robot.get_qpos())
        for _ in range(4):  # render every 4 steps
            if balance_passive_force:
                qf = robot.compute_passive_force(
                    gravity=True,
                    coriolis_and_centrifugal=True,
                )
                robot.set_qf(qf)
            print("target qpos", qpos)
            print("current qpos", robot.get_qpos())
            robot.set_drive_target(qpos)
            scene.step()
        scene.update_render()
        viewer.render()


def main():
    demo(fix_root_link=True, balance_passive_force=True)


if __name__ == "__main__":
    main()

    """
    Links: [Actor(name="world", id="2"), Actor(name="j2n6s200_link_base", id="3"), Actor(name="j2n6s200_link_1", id="4"), Actor(name="j2n6s200_link_2", id="5"), Actor(name="j2n6s200_link_3", id="6"), Actor(name="j2n6s200_link_4", id="7"), Actor(name="j2n6s200_link_5", id="8"), Actor(name="j2n6s200_link_6", id="9"), Actor(name="j2n6s200_end_effector", id="15"), Actor(name="j2n6s200_hand_tip", id="16"), Actor(name="j2n6s200_hand_base", id="10"), Actor(name="j2n6s200_link_finger_1", id="13"), Actor(name="j2n6s200_link_finger_tip_1", id="14"), Actor(name="j2n6s200_link_finger_2", id="11"), Actor(name="j2n6s200_link_finger_tip_2", id="12")]
    Active joints: ['j2n6s200_joint_1', 'j2n6s200_joint_2', 'j2n6s200_joint_3', 'j2n6s200_joint_4', 'j2n6s200_joint_5', 'j2n6s200_joint_6', 'j2n6s200_joint_finger_1', 'j2n6s200_joint_finger_2']

    Joint limits:
        [[      -inf        inf]
        [0.82030475 5.4628806 ]
        [0.33161256 5.951573  ]
        [      -inf        inf]
        [      -inf        inf]
        [      -inf        inf]
        [0.         2.        ]
        [0.         2.        ]]
    """
