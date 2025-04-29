#! /usr/bin/env python3

import subprocess
import sys
import time


def main(pid: int, *args):
    while subprocess.Popen(["ps", "-p", f"{pid}"], stdout=subprocess.DEVNULL).wait() == 0:
        time.sleep(1)
        # print(f"waiting for {pid} to die")

    subprocess.Popen(
        [
            "roslaunch",
            *args
        ],
        start_new_session=True
    )
    sys.exit()


if __name__ == "__main__":
    main(int(sys.argv[1]), *sys.argv[2:])
