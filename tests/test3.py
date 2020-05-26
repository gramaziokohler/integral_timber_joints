from compas.geometry import Frame
from compas_fab.backends import RosClient
from compas_fab.robots import Configuration

group = "robot11_eaXYZ"

frame_WCF = Frame([19.823254, 6.008556, 0.922020],
                  [-1.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0])

start_configuration = Configuration(
    values = (
        18.700,
        0, -4.900,
        0, 0, 0, 0, 0, 0,
        -11.000, -4.900,
        0, 0, 0, 0, 0, 0,
        35.000,
        0, -4.900,
        0, 0, 0, 0, 0, 0,
        -11.000, -4.900,
        0, 0, 0, 0, 0, 0
        ),
    types = (
        2,
        2, 2,
        0, 0, 0, 0, 0, 0,
        2, 2,
        0, 0, 0, 0, 0, 0,
        2,
        2, 2,
        0, 0, 0, 0, 0, 0,
        2, 2,
        0, 0, 0, 0, 0, 0
        ),
    joint_names = (
        'bridge1_joint_EA_X',
        'robot11_joint_EA_Y', 'robot11_joint_EA_Z',
        'robot11_joint_1', 'robot11_joint_2', 'robot11_joint_3', 'robot11_joint_4', 'robot11_joint_5', 'robot11_joint_6',
        'robot12_joint_EA_Y', 'robot12_joint_EA_Z',
        'robot12_joint_1', 'robot12_joint_2', 'robot12_joint_3', 'robot12_joint_4', 'robot12_joint_5', 'robot12_joint_6',
        'bridge2_joint_EA_X',
        'robot21_joint_EA_Y', 'robot21_joint_EA_Z',
        'robot21_joint_1', 'robot21_joint_2', 'robot21_joint_3', 'robot21_joint_4', 'robot21_joint_5', 'robot21_joint_6',
        'robot22_joint_EA_Y', 'robot22_joint_EA_Z',
        'robot22_joint_1', 'robot22_joint_2', 'robot22_joint_3', 'robot22_joint_4', 'robot22_joint_5', 'robot22_joint_6'
        )
    )


with RosClient('192.168.183.129') as client:

    robot = client.load_robot(load_geometry=False,
                              urdf_param_name='/robot_description',
                              srdf_param_name='/robot_description_semantic',
                              precision=None,
                              local_cache_directory=None)

    for i in range(10):
        configuration = robot.inverse_kinematics(
            frame_WCF,
            start_configuration=start_configuration,
            group=group,
            return_full_configuration=True)

        print("Found configuration =", configuration.values)
