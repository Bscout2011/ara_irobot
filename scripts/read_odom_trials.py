
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

results = Path("/home/ara/roomba_ws/results/odom_2m_1ms")

data = []
for fn in results.iterdir():
    with open(fn, 'rb') as f:
        arr = np.load(f)
        data.append(arr)

i = 0
odom = data[0]

prev_odom = np.roll(odom, 1, axis=1)
dist = np.linalg.norm(odom[:,:2] - prev_odom[:,:2], axis=1)[1:]
print(dist)

fig, axs = plt.subplots(2, 1)

axs[0].plot(dist, label="Position")
# axs[1].plot(odom[:,2], label="x Vel")
# axs[2].plot(odom[:,3], label="y vel")

plt.show()
