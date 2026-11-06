"""ROS pose conversion. Quaternion fields are ordered x, y, z, w."""

import numpy as np
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import Header


def stamped_pose(frame, stamp, xyz, xyzw):
    return PoseStamped(header=Header(frame_id=frame, stamp=stamp),
                       pose=Pose(position=Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])),
                                 orientation=Quaternion(x=float(xyzw[0]), y=float(xyzw[1]),
                                                        z=float(xyzw[2]), w=float(xyzw[3]))))


def pose_values(stamped):
    p, q = stamped.pose.position, stamped.pose.orientation
    return np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w])
