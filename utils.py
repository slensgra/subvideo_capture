import cv2
import numpy
import argparse
import sys

def initialize_argument_parser():
    parser = argparse.ArgumentParser(description="Displays and saves a subvideo of a given video stream (file, or ROS topic)")

    parser.add_argument("--width", type=int, required=True, help="Width of the subvideo in the original video")
    parser.add_argument("--height", type=int, required=True,  help="Height of the subvideo in the original video")
    parser.add_argument("--min_corner_x", type=int, required=True, help="X coordinate of the minimum corner of the view window")
    parser.add_argument("--min_corner_y", type=int, required=True, help="Y coordinate of the minimum corner of the view window")
    parser.add_argument("--scale_factor", type=int, required=True, help="Each pixel in the original image is expanded to a scale_factor by scale_factor square in the expanded image.")

    parser.add_argument("--outfile", type=str, help="Output the expanded video stream to this file")
    parser.add_argument("--directory", type=str, help="Directory to pull frame files from. Used by subvideo_from_files.py. Must end in a '/'")
    parser.add_argument("--file_regex", type=str, help="Regex used to select files from the given directory. Used by subvideo_from_files.py")
    parser.add_argument("--topic", type=str, help="ROS topic to pull images from. Used by subvideo_from_ROS.py")

    return parser


def draw_corners(image, corners, color, min_corner_x, min_corner_y, width, height, scale_factor):
    for corner in corners:
        if all(corner >= numpy.array([min_corner_x, min_corner_y])):
            if all(corner < numpy.array([min_corner_x + width, min_corner_y + height])):
                corner_scaled = corner - numpy.array([min_corner_x, min_corner_y])
                corner_scaled = float(scale_factor) * corner_scaled

                cv2.circle(image, tuple(map(int, corner_scaled)), 2, color, 2)


def cut_and_scale_image(raw_image, min_corner_x, min_corner_y, width, height, scale_factor, unrefined_corners=None, refined_corners=None):
    cropped_image = raw_image[min_corner_y:min_corner_y+height, min_corner_x:min_corner_x+width]

    scaled_image = cv2.resize(
        cropped_image,
        (scale_factor*width, scale_factor*height),
        interpolation=cv2.INTER_NEAREST
    )
    scaled_image_color = scaled_image

    if unrefined_corners is not None:
        draw_corners(scaled_image, unrefined_corners, (255, 0, 255), min_corner_x, min_corner_y, width, height, scale_factor)

    if refined_corners is not None:
        draw_corners(scaled_image, refined_corners, (255, 0, 0), min_corner_x, min_corner_y, width, height, scale_factor)

    return scaled_image_color
