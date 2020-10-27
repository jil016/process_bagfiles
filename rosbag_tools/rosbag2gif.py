import imageio
import rosbag
import yaml
import cv2
import numpy as np
from cv_bridge import CvBridge


def rosbag2gif(
    inpath="",
    outpath="",
    topic="",
    image_size=None,
    duration=0.01,
    length=10,
    ifcompress=True,
):
    bag = rosbag.Bag(inpath, "r")
    message_count = bag.get_type_and_topic_info()[1][topic].message_count

    n_frames = min(length // duration, message_count)

    sample_idxs = np.linspace(
        1, message_count, num=n_frames, endpoint=False, dtype=int
    )

    sample_idxs

    with imageio.get_writer(outpath, mode="I", duration=duration) as writer:
        iterator = bag.read_messages(topics=[topic])
        for topic, msg, _ in bag.read_messages(topics=[topic]):
            bridge = CvBridge()
            image = bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            if image_size:
                image = cv2.resize(
                    image, image_size, interpolation=cv2.INTER_AREA
                )

            writer.append_data(image)

        writer.close()
    pass


def rosbag2video():
    pass


if __name__ == "__main__":
    rosbag2gif(
        inpath="/home/erl/rosbag/mit_medfield/mit_medfield_modified.bag",
        outpath="/home/erl/rosbag/mit_medfield/mit_medfield_modified.gif",
        topic="/flea3/image_raw",
        image_size=None,
        duration=0.1,
        length=10,
        ifcompress=False,
    )
