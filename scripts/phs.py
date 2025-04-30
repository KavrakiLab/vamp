import vamp
import matplotlib.pyplot as plt
import math

n = 1000

a = vamp.sphere.Configuration([0.5, 0.5, 1])
b = vamp.sphere.Configuration([0.5, 0.5, 0])

vamp.sphere.set_lows([0, 0, 0])
vamp.sphere.set_highs([1, 1, 1])

phs = vamp.sphere.ProlateHyperspheroid(a, b)
phs.set_transverse_diameter(1.1)

rng = vamp.sphere.xorshift()
# rng = vamp.sphere.halton()

phs_rng = vamp.sphere.phs_sampler(phs, rng)

samples = [phs_rng.next().to_list() for _ in range(n)]
# samples = [rng.next().to_list() for _ in range(n)]

x, y, z = list(map(list, zip(*samples)))

# fig, axs = plt.subplots(1, 3, sharey=True, tight_layout=True)
# axs[0].hist(x)
# axs[1].hist(y)
# axs[2].hist(z)
# plt.show()

fig = plt.figure()
ax = fig.add_subplot(projection = '3d')
ax.scatter(x, y, z)

ax.axes.set_xlim3d(left = 0, right = 1)
ax.axes.set_ylim3d(bottom = 0, top = 1)
ax.axes.set_zlim3d(bottom = 0, top = 1)

plt.show()
