from scipy import interpolate
import numpy as np

x = [0, 1, 2, 3, 4, 5]
y = [0, 3, 1, 2, 1, 3]

tck, u = interpolate.splprep([x, y], k=2, s=0)
unew = np.arange(0, 1.01, 0.01)
out = interpolate.splev(unew, tck)

import matplotlib.pyplot as plt
plt.plot(x, y, 'ro', out[0], out[1], 'b')
plt.show()
