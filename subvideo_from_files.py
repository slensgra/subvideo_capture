from __future__ import print_function

import sys
import os
import re
import numpy
import cv2

import utils

class SubvideoFromDirectory:
    def __init__(self):
        pass

    def get_frame_number(self, fname):
        return int(fname.split("_")[-1].split(".")[0])

    def get_frame_files(self, directory, frame_regex):
        all_file_names = os.listdir(directory)

        matching_file_names = [
            directory + file_name for file_name in all_file_names if re.match(frame_regex, file_name)
        ]

        matching_file_names.sort(key=self.get_frame_number)

        return matching_file_names

    def read_corners_file(self, corners_file):
        corners = []
        with open(corners_file) as corners_stream:
            corners_string = corners_stream.read()
            corners_lines = corners_string.split("\n")

            for line in corners_lines:
                if (" " in line):
                    corners.append(numpy.array([float(line.split(" ")[0]), float(line.split(" ")[1])]))

        return corners

    def run(self):
        parser = utils.initialize_argument_parser()
        args = parser.parse_args()

        if not args.file_regex or not args.directory:
            print("Error! To capture a subvideo from files, the 'directory' and 'file_regex' arguments")
            parser.print_help()
            sys.exit(1)

        if args.outfile:
            video_out = cv2.VideoWriter(
                args.outfile,
                cv2.VideoWriter_fourcc('M','J','P','G'),
                20,
                (args.width * args.scale_factor, args.height * args.scale_factor)
            )

        print(
            "Reading frames from directory {} matching regex {}".format(args.directory, args.file_regex)
        )

        frame_files = self.get_frame_files(args.directory, args.file_regex)

        print("Got {} matching files".format(len(frame_files)))
        for frame_file in frame_files:
            print("Processing ", frame_file)

            frame_number = self.get_frame_number(frame_file)
            corners_file = args.directory + "corner_locations_" + str(frame_number) + ".txt"
            unrefined_corners_file = args.directory + "unrefined_corner_locations_" + str(frame_number) + ".txt"
            unrefined_corners = self.read_corners_file(unrefined_corners_file)
            corners = self.read_corners_file(corners_file)

            raw_image = cv2.imread(frame_file)

            scaled_image_color = utils.cut_and_scale_image(
                raw_image=raw_image,
                min_corner_x=args.min_corner_x,
                min_corner_y=args.min_corner_y,
                width=args.width,
                height=args.height,
                scale_factor=args.scale_factor,
                unrefined_corners=unrefined_corners,
                refined_corners=corners
            )

            if args.outfile:
                video_out.write(scaled_image_color)

            cv2.imshow("frame", scaled_image_color)
            cv2.waitKey(10)

        if args.outfile:
            video_out.release()


if __name__ == "__main__":
    video_maker = SubvideoFromDirectory()
    video_maker.run()
