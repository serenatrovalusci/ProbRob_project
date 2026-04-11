# import numpy as np
# import matplotlib.pyplot as plt
# f = open("dataset.txt")

# lines = f.read().splitlines()

# c = 0
# x = []
# y = []
# for l in lines:
# 	c += 1
# 	if(c < 10):
# 		continue
# 	tokens = l.split(":")
# 	tracker_pose = tokens[-1].strip()
# 	xy = tracker_pose.split(" ")
# 	x.append(float(xy[0]))
# 	y.append(float(xy[1]))


# x_np = np.asarray(x)
# y_np = np.asarray(y)

# fig = plt.figure()
# ax = fig.add_subplot(111)
# ax.scatter(x, y)
# ax.axis("equal")
# plt.show()


import numpy as np
import matplotlib.pyplot as plt

f = open("dataset.txt")
lines = f.read().splitlines()

c = 0
x_tracker, y_tracker = [], []
x_model, y_model = [], []

for l in lines:
    c += 1
    if c < 10:
        continue

    tokens = l.split("model_pose:")
    # Parse model_pose (comes right after "model_pose:")
    model_and_rest = tokens[-1].strip()
    # model_pose values come before "tracker_pose:"
    model_part = model_and_rest.split("tracker_pose:")[0].strip()
    tracker_part = model_and_rest.split("tracker_pose:")[1].strip()

    model_xy = model_part.split()
    tracker_xy = tracker_part.split()

    x_model.append(float(model_xy[0]))
    y_model.append(float(model_xy[1]))

    x_tracker.append(float(tracker_xy[0]))
    y_tracker.append(float(tracker_xy[1]))

fig = plt.figure()
ax = fig.add_subplot(111)
ax.scatter(x_tracker, y_tracker, s=5, label="tracker_pose", color="steelblue")
ax.scatter(x_model, y_model, s=5, label="model_pose", color="tomato")
ax.axis("equal")
ax.legend()
ax.set_title("Model Pose vs Tracker Pose")
plt.show()

