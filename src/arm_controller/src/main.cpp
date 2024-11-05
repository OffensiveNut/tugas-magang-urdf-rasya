#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_state_publisher");
    ros::NodeHandle nh;
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    ros::Rate loop_rate(5000);
    double i = 0,j=0,k=0;
    while (ros::ok())
    {
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(3);
        joint_state.position.resize(joint_state.name.size());
        joint_state.name = {"joint1", "joint2", "joint3"};
        
        if(i<2.3562) i += 0.0001;
        if(j>-1.5707) j -= 0.0001;
        if(k<2.3562) k += 0.0001;
        joint_state.position = {i, j, i}; // in radians | hip, shoulder, elbow
        joint_state_pub.publish(joint_state);
        printf("%lf %lf %lf\n",i,j,k);

        ros::spinOnce();
        loop_rate.sleep();
        if(i>=2.3562 && j<=-1.5707 && k>=2.3562)ros::shutdown();
    }
    return 0;
}
