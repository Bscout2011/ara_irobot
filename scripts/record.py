import rosbag
import matplotlib.pyplot as plt

fn = "/home/ara/romba_ws/src/ara_irobot/scripts/cliff_all_pass_2_lines.bag"

cliff_msgs = rosbag.Bag(fn)

cliff_signal_left = []
cliff_signal_front_left = []
cliff_signal_front_right = []
cliff_signal_right = []

for topic, msg, t in cliff_msgs.read_messages():
    cliff_signal_left.append(msg.cliff_signal_left)
    cliff_signal_front_left.append(msg.cliff_signal_front_left)
    cliff_signal_front_right.append(msg.cliff_signal_front_right)
    cliff_signal_right.append(msg.cliff_signal_right)


plt.plot(cliff_signal_left, label="Left")
plt.plot(cliff_signal_front_left, alpha=.5, label="Front Left")
plt.plot(cliff_signal_front_right, alpha=.5, label="Front Right")
plt.plot(cliff_signal_right, alpha=.5, label="Right")
plt.ylabel("Ground Sensor Signal Strength")
plt.legend()
plt.show()

# print cliff_msgs