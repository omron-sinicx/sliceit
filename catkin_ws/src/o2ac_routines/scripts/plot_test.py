import matplotlib.pyplot as plt

import numpy as np

filename = "catkin_ws/src/o2ac_routines/log/slice_dataset2/hold_free_cucumber/50mm/knife_wrench-2023-02-09_00-45-48.npy"
data = np.load(filename)

# time = np.linspace(0, duration, len(data))

# Force only
for i, ax in enumerate(["X", "Y", "Z"]):
    plt.plot(np.array(data)[:, -1], np.array(data)[:, i], label=ax)
# plt.ylim((-10, 5))

plt.xlabel("Time (s)")
plt.ylabel("Force (N)")
plt.legend()
plt.grid(True)
plt.show()

### Force and torque ###
# fig, axs = plt.subplots(2)

# for i, ax in enumerate(["X", "Y", "Z"]):
#     axs[0].plot(time, np.array(data)[:, i], label=ax)
#     axs[1].plot(time, np.array(data)[:, i+3], label=ax)
# axs[0].set_ylim((-10,5))
# # axs[1].set_ylim((-10,5))

# axs[0].set_xlabel("Time (s)")
# axs[0].set_ylabel("Force (N)")
# axs[1].set_xlabel("Time (s)")
# axs[1].set_ylabel("Torque (Nm)")
# plt.legend()
# plt.show()


# Variables
# stiffness of grasp (how close it is to the fixture)
#
