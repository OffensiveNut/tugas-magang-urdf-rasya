#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <vector>

using namespace std;

class RoboticArm
{
public:
    RoboticArm(double length1, double length2, double length3, ros::NodeHandle &nh)
        : length1(length1), length2(length2), length3(length3), node_handle(nh)
    {
        joint_state_pub = node_handle.advertise<sensor_msgs::JointState>("joint_states", 10);
        marker_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 10);

        marker.header.frame_id = "world";
        marker.ns = "trail";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.x = 0.01;
        marker.scale.y = 0.03;
        marker.scale.z = 0.03;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
    }

    struct JointAngles
    {
        double joint1;
        double joint2;
        double joint3;
    };

    JointAngles computeIK(double target_x, double target_y, double target_z)
    {
        JointAngles angles;
        angles.joint1 = target_y;

        double z_offset = angles.joint1 + length1;
        double target_distance_2d = sqrt(target_x * target_x + (target_z - z_offset) * (target_z - z_offset));

        if (target_distance_2d > length2 + length3)
        {
            angles.joint2 = angles.joint3 = 0;
            return angles;
        }

        double angle3 = acos((length2 * length2 + length3 * length3 - target_distance_2d * target_distance_2d) /
                             (2 * length2 * length3));
        angles.joint3 = angle3;

        double angle2 = atan2(target_z - z_offset, target_x) -
                        atan2(length3 * sin(angle3), length2 + length3 * cos(angle3));
        angles.joint2 = angle2;

        return angles;
    }

    void moveArm(double target_x, double target_y, double target_z)
    {
        JointAngles angles = computeIK(target_x, target_y, target_z);
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.header.stamp = ros::Time::now();
        joint_state_msg.name = {"joint1", "joint2", "joint3"};
        joint_state_msg.position = {angles.joint1, angles.joint2, angles.joint3};
        joint_state_pub.publish(joint_state_msg);

        marker.header.stamp = ros::Time::now();
        marker.pose.position.x = target_x;
        marker.pose.position.y = target_y;
        marker.pose.position.z = target_z;
        marker_pub.publish(marker);

        ros::Duration(1).sleep();
    }

    void drawCharacter(const vector<pair<double, double>> &coordinates, double x)
    {
        for (const auto &point : coordinates)
        {
            moveArm(x, point.first, point.second);
        }
    }

    void drawRASYA()
    {
        vector<pair<double, double>> R = {{0.0, 0.0}, {0.0, 0.4}, {0.2, 0.4}, {0.2, 0.2}, {0.0, 0.2}, {0.2, 0.0}, {0.3, 0.0}};
        vector<pair<double, double>> A = {{0.3, 0.0}, {0.4, 0.4}, {0.45, 0.2}, {0.35, 0.2}, {0.45, 0.2}, {0.5, 0.0}, {0.6, 0.0}};
        vector<pair<double, double>> S = {{0.6, 0.0}, {0.8, 0.0}, {0.8, 0.2}, {0.6, 0.2}, {0.6, 0.4}, {0.8, 0.4}, {0.9, 0.4}};
        vector<pair<double, double>> Y = {{0.9, 0.4}, {1.0, 0.275}, {1.1, 0.4}, {1.0, 0.275}, {1.0, 0.0}};
        vector<pair<double, double>> A2 = {{1.2, 0.0}, {1.3, 0.4}, {1.35, 0.2}, {1.25, 0.2}, {1.35, 0.2}, {1.4, 0.0}};

        double x = 0.325;
        drawCharacter(R, x);
        drawCharacter(A, x);
        drawCharacter(S, x);
        drawCharacter(Y, x);
        drawCharacter(A2, x);
    }

private:
    double length1, length2, length3;
    ros::NodeHandle &node_handle;
    ros::Publisher joint_state_pub;
    ros::Publisher marker_pub;
    visualization_msgs::Marker marker;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_state_publisher");
    ros::NodeHandle nh;
    RoboticArm arm(0.2, 0.2, 0.2, nh);

    arm.drawRASYA();

    return 0;
}
