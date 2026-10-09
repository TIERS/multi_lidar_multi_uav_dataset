# /// script
# requires-python = ">=3.9"
# dependencies = ["rosbags", "numpy", "scipy"]
# ///
"""Write the MOCAP ground truth of a sequence in the Ouster frame (os_sensor = base_link).

Output is a TUM trajectory file: timestamp x y z qx qy qz qw

    python3 scripts/gt_to_lidar.py Holybro01.bag
"""
import argparse
from pathlib import Path

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial.transform import Rotation

# world -> os_sensor, as translation [x, y, z] and quaternion [qx, qy, qz, qw]
TRANSFORMS = {
    "indoor_2023-07-27": ([16.2629, -3.9006, -0.8272], [0.00353, -0.01129, 0.69236, 0.72146]),
    "indoor_2023-08-01": ([16.1233, -4.1902, -0.8008], [0.00045, -0.00881, 0.68473, 0.72875]),
    "outdoor_2023-08-05": ([20.1323, -1.2087, -1.0444], [-0.0072, -0.01412, 0.99791, -0.0626]),
}


def setup_of(name):
    if "Out" in name:
        return "outdoor_2023-08-05"
    if "Stnd" in name or "Stdn" in name:
        return "indoor_2023-08-01"
    return "indoor_2023-07-27"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", type=Path)
    parser.add_argument("-o", "--output", type=Path, help="default: <bag name>_gt_os_sensor.txt")
    args = parser.parse_args()

    setup = setup_of(args.bag.stem)
    t, q = TRANSFORMS[setup]
    rot = Rotation.from_quat(q)

    typestore = get_typestore(Stores.ROS1_NOETIC)
    rows = []
    with Reader(args.bag) as reader:
        conns = [c for c in reader.connections if c.topic.startswith("/vrpn_client_node/")]
        for conn, _, raw in reader.messages(connections=conns):
            msg = typestore.deserialize_ros1(raw, conn.msgtype)
            p, o = msg.pose.position, msg.pose.orientation
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            pos = rot.apply([p.x, p.y, p.z]) + t
            ori = (rot * Rotation.from_quat([o.x, o.y, o.z, o.w])).as_quat()
            rows.append([stamp, *pos, *ori])

    rows = np.array(sorted(rows))
    out = args.output or args.bag.with_name(f"{args.bag.stem}_gt_os_sensor.txt")
    np.savetxt(out, rows, fmt=["%.9f"] + ["%.6f"] * 7)
    print(f"{setup}: wrote {len(rows)} poses ({conns[0].topic}) to {out}")


if __name__ == "__main__":
    main()
