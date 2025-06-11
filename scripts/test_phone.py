#!/usr/bin/env python

import argparse
import os
import numpy as np
from magiclaw.utils.math_utils import convert_pose
from scipy.spatial.transform import Rotation as R
from magiclaw.modules.zmq import PhoneSubscriber
from magiclaw.config import ZMQConfig

# Configure OpenCV for headless environment BEFORE importing cv2
os.environ["OPENCV_VIDEOIO_PRIORITY_MSMF"] = "0"  # Disable Windows Media Foundation
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "video_codec;h264"

# Import CV2 after environment setup
import cv2


def phone_test(params_path: str, headless: bool = False, save_images: bool = False):
    """
    Test the phone.
    
    Args:
        params_path (str): The path of the phone parameters.
        headless (bool): Run in headless mode (no GUI).
        save_images (bool): Save phone frames as images.
    """

    zmq_config = ZMQConfig()
    zmq_config.read_config_file(params_path)

    # Create a PhonePublisher instance
    phone_subscriber = PhoneSubscriber(
        host=zmq_config.phone_host,
        port=zmq_config.phone_port,
    )
    conversion_matrix = np.array(
        [
            [1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1],
        ],
        dtype=np.float32,
    )
    
    color_img_list = []
    depth_img_list = []
    try:
        while True:
            color_img, depth_img, depth_width, depth_height, local_pose, global_pose = phone_subscriber.subscribeMessage()
            # if color_img is not None:
            #     # Display the color image
            #     cv2.imshow("Color Image", color_img)
            #     if save_images:
            #         color_img_list.append(color_img)
            # if depth_img is not None:
            #     # Display the depth image
            #     cv2.imshow("Depth Image", depth_img)
            #     if save_images:
            #         depth_img_list.append(depth_img)
            if global_pose is not None:
                # Display the pose
                global_pose = convert_pose(np.array(global_pose), conversion_matrix)
                global_pose = R.from_quat(global_pose[3:]).as_euler('xyz', degrees=True)
                print(f"Pose: {global_pose[0]:6.3f}, {global_pose[1]:6.3f}, {global_pose[2]:6.3f} (degrees)")
    except KeyboardInterrupt:
        print("Exiting...")
    # Release resources
    cv2.destroyAllWindows()
    phone_subscriber.close()
    if save_images:
        # Save images to disk
        for i, img in enumerate(color_img_list):
            cv2.imwrite(f"color_image_{i}.png", img)
        for i, img in enumerate(depth_img_list):
            cv2.imwrite(f"depth_image_{i}.png", img)
            
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Phone Test")
    parser.add_argument(
        "--params_path",
        type=str,
        default="./configs/phone/phone_0.yaml",
        help="Path to the phone parameters file.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run in headless mode (no GUI).",
    )
    parser.add_argument(
        "--save_images",
        action="store_true",
        help="Save phone frames as images.",
    )
    args = parser.parse_args()

    # Run the phone test
    phone_test(args.params_path, args.headless, args.save_images)