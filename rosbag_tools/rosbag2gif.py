import imageio
import rosbag
import yaml
import tqdm
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

    cnt = 0
    idx = 0
    with imageio.get_writer(outpath, mode="I", duration=duration) as writer:
        iterator = bag.read_messages(topics=[topic])
        for topic, msg, _ in iterator:
            if idx >= n_frames:
                break
            if cnt == sample_idxs[idx]:
                idx += 1
                bridge = CvBridge()
                image = bridge.imgmsg_to_cv2(
                    msg, desired_encoding="passthrough"
                )
                if image_size:
                    image = cv2.resize(
                        image, image_size, interpolation=cv2.INTER_AREA
                    )
                writer.append_data(image)

            elif cnt > sample_idxs[idx]:
                raise ValueError("Index error")

            cnt += 1

        print("GIF is saved to {}".format(outpath))
        writer.close()
    pass


if __name__ == "__main__":
    # import argparse
    # parser = argparse.ArgumentParser(description='Transform rosbag to gif')
    # parser.add_argument('bagfile', nargs=1, help='input bag file')
    # parser.add_argument('--max-offset', nargs=1, help='max time offset (sec) to correct.', default='60', type=float)
    # args = parser.parse_args()

    rosbag2gif(
        inpath="/home/erl/rosbag/realsense_indoor2_1216.bag",
        outpath="/home/erl/rosbag/realsense_indoor2_1216.gif",
        topic="/camera/infra1/image_rect_raw",
        image_size=None,
        duration=0.1,
        length=15,
        ifcompress=False,
    )
