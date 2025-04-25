#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <limits>
#include <cmath>
#include <algorithm>

class LaserScanMerger {
public:
    LaserScanMerger() : tf_listener(tf_buffer) {
        ros::NodeHandle nh;
        front_sub = nh.subscribe("/f_scan", 10, &LaserScanMerger::frontCallback, this);
        back_sub  = nh.subscribe("/b_scan", 10, &LaserScanMerger::backCallback, this);
        merged_pub = nh.advertise<sensor_msgs::LaserScan>("/merged_scan", 10);

        front_received = false;
        back_received = false;
    }

    void frontCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        front_scan = *msg;
        front_received = true;
        mergeScans();
    }

    void backCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        back_scan = *msg;
        back_received = true;
        mergeScans();
    }

    void mergeScans() {
      
        if (!front_received || !back_received)
            return;

        // Create a new LaserScan in the unified coordinate frame "front_laser_link"
        sensor_msgs::LaserScan merged;
        merged.header.frame_id = "front_laser_link";
        merged.header.stamp = ros::Time::now();
   
        merged.angle_min = -M_PI;
        merged.angle_max = M_PI;
        
        // Use the angular resolution of front_scan
        merged.angle_increment = front_scan.angle_increment;  

        
        int num_readings = std::round((merged.angle_max - merged.angle_min) / merged.angle_increment);
        merged.ranges.assign(num_readings, std::numeric_limits<float>::infinity());

        // Set merged range_min and range_max, the extremes of both scans
        merged.range_min = std::min(front_scan.range_min, back_scan.range_min);
        merged.range_max = std::max(front_scan.range_max, back_scan.range_max);

        // --------- Process front_scan data (already in front_laser_link frame) ---------
        for (size_t i = 0; i < front_scan.ranges.size(); ++i) {
            float r = front_scan.ranges[i];
            if (r < front_scan.range_min || r > front_scan.range_max)
                continue;
            // Compute polar coordinates (x, y) of front laser point
            double angle = front_scan.angle_min + i * front_scan.angle_increment;
            double x = r * cos(angle);
            double y = r * sin(angle);
            
            double point_angle = atan2(y, x);
            int index = std::round((point_angle - merged.angle_min) / merged.angle_increment);
            if (index >= 0 && index < num_readings) {
                double dist = sqrt(x*x + y*y);
                merged.ranges[index] = std::min(merged.ranges[index], static_cast<float>(dist));
            }
        }

        // --------- Process back_scan data (needs transformation to front_laser_link frame) ---------
        geometry_msgs::TransformStamped transformStamped;
        try {
            //gets the transform from back_laser_link to front_laser_link
            
            transformStamped = tf_buffer.lookupTransform("front_laser_link", "back_laser_link", 
                                                           ros::Time(0), ros::Duration(1.0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("TF transform failed: %s", ex.what());
            return;
        }

        for (size_t i = 0; i < back_scan.ranges.size(); ++i) {
            float r = back_scan.ranges[i];
            if (r < back_scan.range_min || r > back_scan.range_max)
                continue;
            double angle = back_scan.angle_min + i * back_scan.angle_increment;
            double x = r * cos(angle);
            double y = r * sin(angle);

            // Create point and transform it to front_laser_link frame
            geometry_msgs::PointStamped pt_in, pt_out;
            pt_in.header.frame_id = "back_laser_link";
            pt_in.header.stamp = ros::Time(0);
            pt_in.point.x = x;
            pt_in.point.y = y;
            pt_in.point.z = 0;

            tf2::doTransform(pt_in, pt_out, transformStamped);

            double new_angle = atan2(pt_out.point.y, pt_out.point.x);
            double dist = sqrt(pt_out.point.x * pt_out.point.x + pt_out.point.y * pt_out.point.y);
            int index = std::round((new_angle - merged.angle_min) / merged.angle_increment);
            if (index >= 0 && index < num_readings) {
                merged.ranges[index] = std::min(merged.ranges[index], static_cast<float>(dist));
            }
        }

        merged_pub.publish(merged);
    }

private:
    ros::Subscriber front_sub;
    ros::Subscriber back_sub;
    ros::Publisher merged_pub;

    sensor_msgs::LaserScan front_scan;
    sensor_msgs::LaserScan back_scan;
    bool front_received, back_received;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_scan_merger");
    LaserScanMerger merger;
    ros::spin();
    return 0;
}
