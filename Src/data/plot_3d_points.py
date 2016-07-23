from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot    as plt
import pandas               as pd

fig     = plt.figure()
ax      = fig.add_subplot(111, projection='3d')

table = pd.read_table('points_out.yml' , sep=' ')
# table = pd.read_table('out_no_header.ply' , sep=' ')
print table.shape
# take elements of vectors , with jumps of ##
step_jump = 1
max_ndx = table.shape[0]-1
x = table.values[range(0,max_ndx,step_jump),0]
y = table.values[range(0,max_ndx,step_jump),1]
z = table.values[range(0,max_ndx,step_jump),2]
Zrange = max(z)-min(z)
print Zrange
# z-30 is far away , z  0 is close
# x[z<min(z)+0.1*Zrange]=-20
# y[z<min(z)+0.1*Zrange]=0
# x = table.values[range(0,156000,100)  ,0]
# y = table.values[range(0,156000,100),1]
# z = table.values[range(0,156000,100),2]

ax.scatter(x, y, z, c='r', marker='o')

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
# plt view rotate how?
plt.show()