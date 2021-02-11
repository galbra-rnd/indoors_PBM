#include <ros/ros.h>
#include "Pbm/PbmBootstrapper.hpp"

int main(int argc, char **argv)
{
    const std::string nodeName("indoors_pbm");
    ros::init(argc, argv, nodeName);

    ros::AsyncSpinner spinner(4);
    ros::NodeHandle nh;

    try
    {
        PbmBootStrapper pbmbs(nh);

        int bit_freq;
        int spin_freq;
        ros::Timer timerPublishBIT;
        ros::Timer timerPBMSpin;
        if (!nh.param(ros::this_node::getName() + "/bit_freq", bit_freq, 1))
        {
            ROS_WARN_STREAM("No param " << ros::this_node::getName() << "/bit_freq set. Setting defult: 1HZ.");
            timerPublishBIT = nh.createTimer(ros::Duration(1.0 / bit_freq), std::bind(&PbmBootStrapper::PublishBIT, &pbmbs));
        }
        else if (bit_freq != 0)
        {
            timerPublishBIT = nh.createTimer(ros::Duration(1.0 / bit_freq), std::bind(&PbmBootStrapper::PublishBIT, &pbmbs));
        }
        else
        {
            ROS_ERROR_STREAM("Param:" << ros::this_node::getName() << "Cant be set to 0!!!!");
        }

        nh.param(ros::this_node::getName() + "/spin_freq", spin_freq, 1);
        timerPBMSpin = nh.createTimer(ros::Duration(1.0 / spin_freq), std::bind(&PbmBootStrapper::Spin, &pbmbs));

        spinner.start();
        ros::waitForShutdown();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM(e.what());
    }

    return (0);
}