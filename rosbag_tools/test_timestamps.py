import imageio
import rosbag
import yaml
import tqdm
import cv2
import numpy as np
import subprocess, yaml
from cv_bridge import CvBridge


class rosbag_file:
    def __init__(self, filepath="", imu_topic=[], img_topic=[]):
        self.if_stereo = True if len(img_topic == 2) else False
        self.filepath = filepath
        self.imu_topic = imu_topic
        self.img_topic = img_topic
        self.rosbag = rosbag.Bag(filepath, "r")

        self.imu_data = []
        self.img_data = []

        pass

    def _load_imu(self,):
        pass

    def _load_img(self,):
        pass


def cmp_imu_img(bag="", imu_topic="", img_topic=""):
    bag = rosbag.Bag(inpath, "r")
    import subprocess, yaml

    info_dict = yaml.load(
        subprocess.Popen(
            ["rosbag", "info", "--yaml", "input.bag"], stdout=subprocess.PIPE
        ).communicate()[0]
    )

    header_t = []
    msg_t = []
    start_time = 0
    end_time = 0
    duration = 0
    pass


def cross_cmp_timestamps(
    bag1="",
    bag2="",
    imu_topic1="",
    imu_topic2="",
    img_topic1="",
    img_topic2="",
):

    pass


def examine_fps():
    """
    Sliding window
    """

    pass


def show_timestamps(
    bag1="", bag2="", imu_topic1="", topic2="", length=10, ifcompress=True,
):
    pass


if __name__ == "__main__":
    # import argparse
    # parser = argparse.ArgumentParser(description='Transform read & compare timestamps')
    # parser.add_argument('bagfile', nargs=1, help='input bag file')
    # parser.add_argument('--max-offset', nargs=1, help='max time offset (sec) to correct.', default='60', type=float)
    # args = parser.parse_args()
    bag = rosbag_file(
        filepath="/home/erl/rosbag/rs_d435i_test_bfs_chair1.bag",
    )
