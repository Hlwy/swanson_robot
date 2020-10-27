import rosbag

from tf.msg import tfMessage

inputBagPath = "input.bag"
newBagPath = "output.bag"

with rosbag.Bag(newBagPath, mode='w') as outbag:
    for topic, msg, stamp in rosbag.Bag(inputBagPath).read_messages():
        if topic == "/tf" and msg.transforms:
            newList = [];
            for m in msg.transforms:
                if m.header.frame_id != "map": newList.append(m)
                else: print("map frame removed.")
            if len(newList) > 0:
                msg.transforms = newList
                outbag.write(topic, msg, stamp)
        else: outbag.write(topic, msg, stamp)

print("rosbag filtering finished.")
