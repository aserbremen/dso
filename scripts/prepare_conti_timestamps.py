
import sys
import os

def main():
    if len(sys.argv) < 2:
        print("Usage ", sys.argv[0], " <path_to_ecal_videofiles>")
        return -1

    image_path = sys.argv[1]
    timestamps_file = os.path.join(image_path, "camera_front_center_60fov.h264.timestamps")

    with open(timestamps_file, mode='r') as file:
        lines = [line.strip("\n").split(" ") for line in file]

    times_file = os.path.join(image_path, "times.txt")
    with open(times_file, mode='w') as file:

        for id, ts in lines:
            ts = ts[:-6] + "." + ts[-6:]
            # print(id, ts)
            file.write("{} {}\n".format(id, ts))

    print("created dso timestamps file at {}".format(times_file))

if __name__ == "__main__":
    main()