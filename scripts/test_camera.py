#!/usr/bin/env python

"""
Camera connection test

This script is to test the camera connection.
It initializes the camera based on the provided parameter file.
It can run in headless mode (no GUI) and save images, or display them in a window.

Example usage:

```bash
python test_camera.py --params_path camera/config/file/path.yaml --headless --save_images
```

"""

import argparse
import os
import yaml

# Configure OpenCV for headless environment BEFORE importing cv2
os.environ["OPENCV_VIDEOIO_PRIORITY_MSMF"] = "0"  # Disable Windows Media Foundation
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "video_codec;h264"

# Import CV2 after environment setup
import cv2
from magiclaw.devices.camera import UsbCamera, WebCamera


def camera_test(params_path: str, headless: bool = False, save_images: bool = False):
    """Test the camera."""

    # Initialize camera
    with open(params_path, "r") as f:
        camera_params = yaml.load(f.read(), Loader=yaml.Loader)

    # Load detector parameters
    with open(f"./config/detector.yaml", "r") as f:
        detector_params = yaml.load(f, Loader=yaml.Loader)

    camera_name = os.path.basename(params_path).split(".")[0]
    # Initialize camera based on the mode
    if camera_params["mode"] == "usb":
        camera = UsbCamera(
            name=camera_name,
            camera_params=camera_params,
            detector_params=detector_params,
        )
    elif camera_params["mode"] == "web":
        camera = WebCamera(
            name=camera_name,
            camera_params=camera_params,
            detector_params=detector_params,
        )
    else:
        raise ValueError("\033[31mUnsupported camera mode!\033[0m")

    # Create output directory for saved images if needed
    output_dir = None
    if save_images or headless:
        output_dir = os.path.join(".", "camera_test_output")
        os.makedirs(output_dir, exist_ok=True)
        print(f"Images will be saved to {output_dir}")

    # Determine if we have display capability
    has_display = False
    if not headless:
        # Simple check to see if we're running in a graphical environment
        has_display = "DISPLAY" in os.environ and os.environ["DISPLAY"]
        print(f"DISPLAY environment variable: {os.environ.get('DISPLAY', 'Not set')}")

        if has_display:
            # Don't even try to use imshow if not in a graphical environment
            try:
                cv2.namedWindow(camera_name, cv2.WINDOW_NORMAL)
                print("OpenCV GUI initialized successfully")
            except Exception as e:
                print(f"Failed to initialize OpenCV GUI: {e}")
                has_display = False

    print(f"Running in {'headless' if headless or not has_display else 'display'} mode")

    # Start capturing
    frame_count = 0
    try:
        while True:
            # Read image
            pose, img = camera.readImageAndPose()
            if img is None or img.size == 0:
                print("Warning: Empty image received, skipping frame")
                continue

            pose = camera.poseToReferece(pose)
            pose = camera.poseVectorToEuler(pose).flatten()

            frame_count += 1

            # Print the pose
            print(
                f"Pose: [{pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f}, {pose[3]:.2f}, {pose[4]:.2f}, {pose[5]:.2f}], Press Ctrl+C to exit"
            )

            # Save images periodically if requested
            if (save_images or headless) and output_dir and frame_count % 30 == 0:
                img_path = os.path.join(output_dir, f"frame_{frame_count:04d}.jpg")
                cv2.imwrite(img_path, img)
                print(f"Saved image to {img_path}")

            # Display the image if we have a display
            if has_display and not headless:
                try:
                    cv2.imshow(camera_name, img)
                    key = cv2.waitKey(1)
                    if key == 27 or key == ord("q"):  # ESC or 'q' key
                        print("User requested exit")
                        break
                except Exception as e:
                    print(f"Display error: {e}")
                    has_display = False
                    print("Switching to headless mode...")
            else:
                # Small delay to prevent CPU overuse
                import time

                time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nUser interrupted. Exiting...")
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        # Clean up
        if has_display and not headless:
            cv2.destroyAllWindows()
        camera.release()
        print("Camera released.")


if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Camera connection test")
    parser.add_argument(
        "--params_path",
        type=str,
        help="The path of the camera parameters.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run in headless mode (no GUI)",
    )
    parser.add_argument(
        "--save_images",
        action="store_true",
        help="Save camera frames as images",
    )
    args = parser.parse_args()
    
    # Check if the parameters file exists
    if args.params_path is None:
        print("Please provide the path to the camera parameters file.")
        exit(1)
    if not os.path.exists(args.params_path):
        print(f"Camera parameters file not found: {args.params_path}")
        exit(1)
    if not os.path.isfile(args.params_path):
        print(f"Camera parameters path is not a file: {args.params_path}")
        exit(1)
    if not os.path.splitext(args.params_path)[1] == ".yaml":
        print(f"Camera parameters file is not a YAML file: {args.params_path}")
        exit(1)
    
    # Test camera
    camera_test(args.params_path, args.headless, args.save_images)
