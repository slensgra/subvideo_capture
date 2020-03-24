import sys
import os
import re
import numpy
import cv2
from __future__ import print_function

import utils


def get_frame_number(fname):
    return int(fname.split("_")[-1].split(".")[0])


def get_frame_files(directory, frame_regex):
    all_file_names = os.listdir(directory)

    matching_file_names = [
        directory + file_name for file_name in all_file_names if re.match(frame_regex, file_name)
    ]

    matching_file_names.sort(key=get_frame_number)

    return matching_file_names


def read_corners_file(corners_file):
    corners = []
    with open(corners_file) as corners_stream:
        corners_string = corners_stream.read()
        corners_lines = corners_string.split("\n")

        for line in corners_lines:
            if (" " in line):
                corners.append(numpy.array([float(line.split(" ")[0]), float(line.split(" ")[1])]))

    return corners


def main():
    print(
        "Reading frames from directory {} matching regex {}".format(directory, file_regex)
    )

    parser = utils.initialize_argument_parser()

    if not parser.file_regex or not parser.directory:
        print("Error! To capture a subvideo from files, the 'directory' and 'file_regex' arguments")
        parser.print_help()
        sys.exit(1)

    if parser.outfile:
        video_out = cv2.VideoWriter(
            outfile,
            cv2.VideoWriter_fourcc('M','J','P','G'),
            20,
            (parser.width * parser.scale_factor, parser.height * parser.scale_factor)
        )

    frame_files = get_frame_files(parser.directory, parser.file_regex)

    print("Got {} matching files".format(len(frame_files)))
    for frame_file in frame_files:
        print("Processing ", frame_file)

        frame_number = get_frame_number(frame_file)
        corners_file = directory + "corner_locations_" + str(frame_number) + ".txt"
        unrefined_corners_file = directory + "unrefined_corner_locations_" + str(frame_number) + ".txt"
        unrefined_corners = read_corners_file(unrefined_corners_file)
        corners = read_corners_file(corners_file)

        raw_image = cv2.imread(frame_file)

        scaled_image_color = cut_and_scale_image(
            raw_image=raw_image,
            min_corner_x=parser.min_corner_x,
            min_corner_y=parser.min_corner_y,
            width=parser.width,
            height=parser.height,
            scale_factor=parser.scale_factor,
            unrefined_corners_file=unrefined_corners,
            refined_corners_file=corners
        )

        video_out.write(scaled_image_color)
        cv2.imshow("frame", scaled_image_color)
        cv2.waitKey(10)

    if parser.outfile:
        video_out.release()


if __name__ == "__main__":
    main()
