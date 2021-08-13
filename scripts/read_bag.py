import argparse

from matplotlib.colors import LogNorm
import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib.pyplot as plt
import os

# A required callback method that goes into the trackbar function.
def nothing(x):
    pass

def slide_threshold(img):
    # Get the new values of the trackbar in real time as the user changes them
    l_h = cv2.getTrackbarPos("L - H", "Aligned Example")
    l_s = cv2.getTrackbarPos("L - S", "Aligned Example")
    l_v = cv2.getTrackbarPos("L - V", "Aligned Example")
    u_h = cv2.getTrackbarPos("U - H", "Aligned Example")
    u_s = cv2.getTrackbarPos("U - S", "Aligned Example")
    u_v = cv2.getTrackbarPos("U - V", "Aligned Example")

    # Set the lower and upper HSV range according to the value selected by the trackbar
    lower_range = np.array([l_h, l_s, l_v])
    upper_range = np.array([u_h, u_s, u_v])

    # Filter the image and get the binary mask, where white represents your target color
    mask = cv2.inRange(img, lower_range, upper_range)
    # You can also visualize the real part of the target color (Optional)
    res = cv2.bitwise_and(img, img, mask=mask)
    return res

def extract_from_bag(bag_fname, color_fname=None, depth_fname=None):

    config = rs.config()
    pipeline = rs.pipeline()

    # make it so the stream does not continue looping
    config.enable_stream(rs.stream.color)
    config.enable_stream(rs.stream.depth)
    rs.config.enable_device_from_file(config, bag_fname, repeat_playback=True)
    profile = pipeline.start(config)
    # this makes it so no frames are dropped while writing video
    playback = profile.get_device().as_playback()
    playback.set_real_time(False)

    colorizer = rs.colorizer()

    align_to = rs.stream.color
    align = rs.align(align_to)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    depth_matrices = []
    bgr_colors = []
    hsv_colors = []

    cv2.namedWindow('Aligned Example', cv2.WINDOW_AUTOSIZE)

    # Now create 6 trackbars that will control the lower and upper range of 
    # H,S and V channels. The Arguments are like this: Name of trackbar, 
    # window name, range,callback function. For Hue the range is 0-179 and
    # for S,V its 0-255.
    # cv2.createTrackbar("L - H", "Aligned Example", 0, 179, nothing)
    # cv2.createTrackbar("L - S", "Aligned Example", 0, 255, nothing)
    # cv2.createTrackbar("L - V", "Aligned Example", 0, 255, nothing)
    # cv2.createTrackbar("U - H", "Aligned Example", 179, 179, nothing)
    # cv2.createTrackbar("U - S", "Aligned Example", 255, 255, nothing)
    # cv2.createTrackbar("U - V", "Aligned Example", 255, 255, nothing)

    i = 0
    while True:

        # when stream is finished, RuntimeError is raised, hence this
        # exception block to capture this
        try:
            # frames = pipeline.wait_for_frames()
            frames = pipeline.wait_for_frames(timeout_ms=100)
            if frames.size() <2:
                # Inputs are not ready yet
                continue
        except (RuntimeError):
            print('frame count', i-1)
            pipeline.stop()
            break

        # align the depth to color frame
        aligned_frames = align.process(frames)

        # get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        # validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        depth_colormap = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
        scaled_depth_image = depth_image * depth_scale

        color_image = np.asanyarray(color_frame.get_data())

        # convert color image to BGR for OpenCV
        color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
        # Convert to HSV
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # Output color in the center
        x_center_coord, y_center_coord = color_frame.height // 2, color_frame.height // 2
        bgr_center_color = color_image[x_center_coord, y_center_coord]
        hsv_center_color = hsv[x_center_coord, y_center_coord]
        # print("HSV Center Color:", center_color)
        bgr_colors.append(bgr_center_color)
        hsv_colors.append(hsv_center_color)

        # res = slide_threshold(hsv)  # mask areas based on HSV colorspace
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        images = np.hstack((hsv, hsv))
        
        cv2.imshow('Aligned Example', images)

        if color_fname is not None:
            fname = "frame{:06d}".format(i) + ".png"
            cv2.imwrite(color_fname + fname, color_image)  # Don't Save

        if depth_fname is not None:
            depth_matrices.append(scaled_depth_image)

        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

        i += 1

    # release everything now that job finished
    if depth_fname is not None:
        np.save(depth_fname, np.array(depth_matrices))  # Don't save
        print("Size of depth matrices:", len(depth_matrices))

    # Show Histograms
    print("Showing histograms")
    bgr_colors = np.array(bgr_colors)
    hsv_colors = np.array(hsv_colors)

    fig, axs = plt.subplots(1, 2)
    nbins = 10
    
    axs[0].hist2d(bgr_colors[:,0], bgr_colors[:,1], bins=nbins, norm=LogNorm())
    axs[0].set_xlabel("B")
    axs[0].set_ylabel("G")
    axs[0].set_xlim([0, 255])
    axs[0].set_ylim([0, 255])

    axs[1].hist2d(hsv_colors[:,1], hsv_colors[:,2], bins=nbins, norm=LogNorm())
    axs[1].set_xlabel("S")
    axs[1].set_ylabel("V")
    axs[1].set_xlim([0, 255])
    axs[1].set_ylim([0, 255])

    plt.show()



if __name__ == "__main__":
    # parser = argparse.ArgumentParser()
    # parser.add_argument("-i", "--input", type=str, help=".bag file to read")
    # parser.add_argument("-c", "--rgbfilename", type=str, help=".mp4 file to save RGB stream")
    # parser.add_argument("-d", "--depthfilename", type=str, help=".npy file to save depth stream")
    # args = parser.parse_args()

    fn = "/home/ara/roomba_ws/data/beam_stereo.bag"

    extract_from_bag(bag_fname=fn)