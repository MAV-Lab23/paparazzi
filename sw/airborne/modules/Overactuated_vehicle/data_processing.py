import pandas as pd
import numpy as np
from matplotlib import pyplot as plt


# Data processing test case
path_balance = "~/Documents/OJF_experiment/Balance/Jul12/9ms_700rads_unsteady_single.txt"
path_drone = "~/Documents/OJF_experiment/Drone/Jul12/fr_0013.log"

# balance_data = pd.read_csv(path_balance, sep='\t')
col_names = ["time", "Fx", "Fy", "Fz", "Mx", "My", "Mz", "Comment"]
balance_data = pd.read_csv(path_balance, sep='\t', skiprows=[i for i in range(0, 21)])
balance_data.columns = col_names
balance_data.drop(columns="Comment", inplace=True)
# balance_data.drop(balance_data.columns[-1])

fig, ((ax1, ax2, ax3), (ax4, ax5, ax6)) = plt.subplots(2, 3)


ax1.plot(balance_data["time"], balance_data["Fx"])
ax1.set_ylabel("Fx")
ax1.set_xlabel("time")

ax2.plot(balance_data["time"], balance_data["Fy"])
ax2.set_ylabel("Fy")
ax2.set_xlabel("time")

ax3.plot(balance_data["time"], balance_data["Fz"])
ax3.set_ylabel("Fz")
ax3.set_xlabel("time")

ax4.plot(balance_data["time"], balance_data["Mx"])
ax4.set_ylabel("Mx")
ax4.set_xlabel("time")

ax5.plot(balance_data["time"], balance_data["My"])
ax5.set_ylabel("My")
ax5.set_xlabel("time")

ax6.plot(balance_data["time"], balance_data["Mz"])
ax6.set_ylabel("Mz")
ax6.set_xlabel("time")



plt.show()