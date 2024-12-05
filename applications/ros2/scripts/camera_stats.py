#!/usr/bin/env python3

import rosbag
import numpy as np
import argparse
from tqdm import tqdm


def get_camera_stats(bagfile):
    """Get camera stats. Used to verify recording is working as expected:

    The following info is returned
        - width
        - height
        - encoding
        - timestamps
        - pixel_sum
        - avg_fps
        - num_blank_images

    Parameters
    ----------
        bagfile: path to bag file containing topics of the pattern /.../image_raw

    Returns
    -------
        dict of camera stats for each camera
    """

    bag = rosbag.Bag(bagfile)
    assert bag

    ret = {}

    class CameraStat():
        def __init__(self):
            self.width = 0,
            self.height = 0,
            self.encoding = "",
            self.timestamps= []
            self.pixel_sum = []
            self.avg_fps = 0
            self.num_blank_images = 0

    for topic, msg, t in tqdm(bag.read_messages(), total=bag.get_message_count()):
        if "/image_raw" not in topic:
            continue

        t = t.to_sec()

        if topic not in ret:
            ret[topic] = CameraStat()

        ret[topic].width = msg.width
        ret[topic].height = msg.height
        ret[topic].encoding = msg.encoding
        ret[topic].timestamps.append(t)
        ret[topic].pixel_sum.append(np.sum(np.frombuffer(msg.data, dtype=np.uint8)))

    for key, data in ret.items():
        a = np.min(data.timestamps)
        b = np.max(data.timestamps)
        c = b - a

        if c > 0:
            data.avg_fps = len(data.timestamps) / c

        data.num_blank_images = np.sum(data.pixel_sum == 0)

    return ret


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Get basic camera stats from bag file")
    parser.add_argument("bag", help="bag file")

    args = parser.parse_args()

    stats = get_camera_stats(args.bag)

    for topic, data in stats.items():
        print(topic)
        print("-" * len(topic))
        print(f"  width: {data.width}")
        print(f"  height: {data.height}")
        print(f"  encoding: {data.encoding}")
        print(f"  average FPS: {data.avg_fps:.2f}")

        per = float(data.num_blank_images) / len(data.timestamps)

        print(f"  blank images: {data.num_blank_images}/{len(data.timestamps)} ({per*100:.2f}%)")
        print("")