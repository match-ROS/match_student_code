#!/usr/bin/env python3
import rospy
import tf2_ros
import tf_conversions
import csv
from geometry_msgs.msg import TransformStamped

def mat_from_tf(tf_msg):
    t = tf_msg.transform.translation
    r = tf_msg.transform.rotation
    M = tf_conversions.transformations.quaternion_matrix([r.x, r.y, r.z, r.w])
    M[0,3], M[1,3], M[2,3] = t.x, t.y, t.z
    return M

def main():
    rospy.init_node("compare_poses")

    wall_frame = "wall"
    scanner_gt_frame = "vertical_profiler_link"
    scanner_est_frame = "scanner_icp_refined"

    out_gt = rospy.get_param("~gt_csv", "groundtruth_poses.csv")
    out_est = rospy.get_param("~est_csv", "estimated_poses.csv")

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(20)

    gt_file = open(out_gt, "w", newline="")
    est_file = open(out_est, "w", newline="")
    gt_writer = csv.writer(gt_file)
    est_writer = csv.writer(est_file)

    gt_writer.writerow(["t", "x", "y", "z", "qx", "qy", "qz", "qw"])
    est_writer.writerow(["t", "x", "y", "z", "qx", "qy", "qz", "qw"])

    rospy.loginfo("Recording GT and Estimated Scanner Poses...")

    while not rospy.is_shutdown():
        stamp = rospy.Time.now()

        try:
            gt_tf = tf_buffer.lookup_transform(wall_frame, scanner_gt_frame, stamp, rospy.Duration(0.1))
            gt = gt_tf.transform
            gt_writer.writerow([stamp.to_sec(), gt.translation.x, gt.translation.y, gt.translation.z,
                                gt.rotation.x, gt.rotation.y, gt.rotation.z, gt.rotation.w])
        except:
            pass

        try:
            est_tf = tf_buffer.lookup_transform(wall_frame, scanner_est_frame, stamp, rospy.Duration(0.1))
            est = est_tf.transform
            est_writer.writerow([stamp.to_sec(), est.translation.x, est.translation.y, est.translation.z,
                                 est.rotation.x, est.rotation.y, est.rotation.z, est.rotation.w])
        except:
            pass

        rate.sleep()

    gt_file.close()
    est_file.close()

if __name__ == "__main__":
    main()
