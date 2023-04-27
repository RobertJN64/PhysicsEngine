from util import *
import math

# Pendulum Physics

# Coordinates:
# (0,0) - pivot point
# 1 meter string length
# 30 degree swing angle -0.5 meters to 0.5 meters
# Coordinates are computed based on circular motion

# Velocity
# Direction is computed from the angle of the previous point to the next point
# Magnitude is computed from PE = KE

# Accel
# Direction is computed from the velocity change
# Magnitude is computed from the velocity change / time between points

# Settings
ENABLE_POINTS = True
ENABLE_VEL_VECTORS = True
ENABLE_ACCEL_VECTORS = True
ENABLE_FT_VECTORS = True
ENABLE_MG_VECTORS = True

start_x = -0.75 #meters
end_x = 0.75 #meters
n_points = 1000 #number of points to simulate

# Constants
g = 9.8 #m/s/s
mass = 1 #kg
length = 1 #meter

#Render View Constants
v_scale_f = 0.05 #Multiply velocity mag before render
accel_scale_f = 0.01
force_scale_f = 0.01

#region STEP 0: Setup Graphs
import matplotlib.pyplot as pyplot
vel_fig = pyplot.figure(figsize=(12, 8))
vel_ax = vel_fig.add_subplot()
#endregion

#region STEP 1: Generate Points
assert -start_x < length and end_x < length, "Pendulum too short"

points = []
x_delta = (end_x - start_x) / (n_points - 1)
for n in range(n_points):
    x = start_x + n * x_delta
    y = -((length ** 2 - x ** 2) ** 0.5)
    points.append((x, y))

if ENABLE_POINTS:
    vel_ax.scatter([p[0] for p in points[2:-2]], [p[1] for p in points[2:-2]])
vel_ax.add_patch(pyplot.Circle((0,0), radius=0.01))
#endregion

#region STEP 2: Generate Velocity Vectors
velocities = []
y_start = points[0][1]

for n in range(n_points):
    x, y = points[n]
    delta_h = y - y_start
    vel_mag = (2 * g * -delta_h) ** 0.5 #velocity = sqrt(2 * g * delta_h)

    #Find direction (check previous and next points)
    x1, y1 = get_prev_point(points, n)
    x2, y2 = get_next_point(points, n)
    v_angle_rad = math.atan2(y2 - y1, x2 - x1)

    velocities.append(Vector(vel_mag, v_angle_rad))

    delta_x = math.cos(v_angle_rad) * vel_mag
    delta_y = math.sin(v_angle_rad) * vel_mag

    if ENABLE_VEL_VECTORS and 1 < n < n_points - 2:
        vel_ax.arrow(x, y, delta_x * v_scale_f, delta_y * v_scale_f,
                     head_width=0.03, head_length=0.03, color='green', length_includes_head=True)
#endregion

#region STEP 3: Generate Accel Vectors
accels = [None, None] #add null accel at beginning

for n in range(2, n_points - 2):
    x, y = points[n]
    vec1 = velocities[n - 1]
    vec2 = velocities[n + 1]

    dx = math.cos(vec2.angle) * vec2.mag - math.cos(vec1.angle) * vec1.mag
    dy = math.sin(vec2.angle) * vec2.mag - math.sin(vec1.angle) * vec1.mag


    #Calc factor to divide by
    x1, y1 = get_prev_point(points, n)
    x2, y2 = get_next_point(points, n)
    dist = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
    time = dist / velocities[n].mag

    accels.append((dx / time, dy / time))

    if ENABLE_ACCEL_VECTORS:
        vel_ax.arrow(x, y, dx * accel_scale_f / time, dy * accel_scale_f / time,
                     head_width=0.03, head_length=0.03, color='orange', length_includes_head=True)

#endregion

#region STEP 4: FBD
forces = [None, None]

for n in range(2, n_points - 2):
    x, y = points[n]
    accel_x, accel_y = accels[n]

    MG_y = -g * mass
    FT_y = accel_y * mass - MG_y
    FT_x = accel_x * mass #Force Tension is only horizontal force

    if ENABLE_FT_VECTORS:
        vel_ax.arrow(x, y, FT_x * force_scale_f, FT_y * force_scale_f,
                     head_width=0.03, head_length=0.03, color='black', length_includes_head=True)

    forces.append((FT_x, FT_y, MG_y))

    if ENABLE_MG_VECTORS:
        vel_ax.arrow(x, y, 0, MG_y * force_scale_f,
                     head_width=0.03, head_length=0.03, color='black', length_includes_head=True)

#endregion

vel_ax.set_xlim(-1, 1)
vel_ax.set_ylim(-1.1, 0.3)

graph_fig = pyplot.figure(figsize=(6, 6))
graph_ax = graph_fig.add_subplot()

graph_ax.plot([p[0] for p in points], [v.mag for v in velocities], label='velocity', color='green')
graph_ax.plot([p[0] for p in points[2:-2]], [(a[0] ** 2 + a[1] ** 2) ** 0.5 for a in accels[2:]], label='accel', color='orange')
graph_ax.plot([p[0] for p in points[2:-2]], [(f[0] ** 2 + f[1] ** 2) ** 0.5 for f in forces[2:]], label='FT', color='black')
graph_ax.plot([p[0] for p in points], [mass * g * (3 - 2 * math.cos(math.asin(end_x)))] * len(points), label='Max FT', color='yellow')
graph_ax.plot([p[0] for p in points], [mass * g] * len(points), label='MG', color='blue')
graph_ax.legend()


pyplot.show()
