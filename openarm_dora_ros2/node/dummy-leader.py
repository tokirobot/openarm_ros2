#!/usr/bin/env python
# -*- coding: utf-8 -*-
import dora
import pyarrow as pa
import math
import time


def main():
    node = dora.Node()
    start_time = time.time()

    print("🚀 Dummy Leader is running! Sending sine wave (0~90 deg) to joint 4...")

    for event in node:
        if event["type"] == "INPUT" and event["id"] == "tick":
            t = time.time() - start_time

            angle_rad = (math.pi / 4) + (math.pi / 4) * math.sin(t*2.0)

            positions = [0.0] * 8

            positions[7] = angle_rad

            node.send_output("left_position", pa.array(positions))
            node.send_output("right_position", pa.array(positions))


if __name__ == '__main__':
    main()
