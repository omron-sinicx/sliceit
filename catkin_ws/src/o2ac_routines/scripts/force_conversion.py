
from ur_control import conversions, spalg, transformations, filters
import numpy as np

import glob

import collections

np.set_printoptions(suppress=True)
np.set_printoptions(linewidth=np.inf)

files = glob.glob("/root/o2ac-ur/catkin_ws/src/o2ac_routines/log/clean_slice_dataset/**/*ft_sensor*.npy", recursive=True)
print(len(files))

filter = filters.ButterLowPass(10, 500, 2)

# files = ["/root/o2ac-ur/catkin_ws/src/o2ac_routines/log/slice_dataset22/hold_free_cucumber/50mm/ft_sensor-2023-02-09_00-45-48.npy"]

for file in files:
    # print(file)
    ft_sensor = np.load(file)

    wrench_filename = file.replace("ft_sensor", "filter")
    wrench_filename = file.replace("ft_sensor", "filtered")

    # knife_pose = [0.13799633, 0.50516513, 0.17502719, 0.49901697, 0.50161294, 0.49833507, 0.50102762]
    # sensor_pose = [-0.01975098, 0.50624583, 0.37520602, -0.70707867, 0.70712959, -0.0014217, -0.00234098]
    # Tsk = transformations.transform_between_poses(sensor_pose, knife_pose)
    # Tks = transformations.transform_between_poses(knife_pose, sensor_pose)
    conversions.from_pose_to_list()

    pose_knife_to_sensor = [0.0,  0.21, -0.158,  0.0,  0.707, -0.707, 0.0]

    r_ks = transformations.quaternion_rotate_vector([0.0,  0.707, -0.707, 0.0], pose_knife_to_sensor[:3])

    knife_wrench = []

    for ft in ft_sensor:

        torque2force = spalg.sensor_torque_to_tcp_force(tcp_position=[0.0, -0.158, 0.0], sensor_torques=ft[3:6])
        # print(ft[:3], torque2force)
        ft[:3] += torque2force
        ft[3:6] = np.zeros(3)

        wrench = spalg.convert_wrench(ft[:6], pose_knife_to_sensor)
        knife_wrench.append(wrench.tolist())

    filtered = filter(np.array(knife_wrench))
    # filtered = np.array(knife_wrench)
    filtered = np.append(filtered, ft_sensor[:, -1].reshape(-1, 1), axis=1)

    np.save(wrench_filename, filtered)
