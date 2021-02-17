#include "Pbm/PbmBootstrapper.hpp"

bool init_params(PbmBootStrapper &pbmbs, ros::NodeHandle &nh)
{

    bool ret = true;

    /**
     * @brief Load initiated parameters for components threshold
     * 
     */

    // ThresholdValues bat_time_left_estimation_thresh;

    // nh.param("/PBM/thresholds/battery_threshold/red", bat_time_left_estimation_thresh.red, 100.f);
    // nh.param("/PBM/thresholds/battery_threshold/orange", bat_time_left_estimation_thresh.orange, 300.f);
    // nh.param("/PBM/thresholds/battery_threshold/green", bat_time_left_estimation_thresh.green, 900.f);
    // pbmbs.m_components_thresholds[Components::BATTERY_TIME] = bat_time_left_estimation_thresh;

    // ThresholdValues TTH_estimation_thresh;

    // nh.param("/PBM/thresholds/tth_threshold/red", TTH_estimation_thresh.red, 100.f);
    // nh.param("/PBM/thresholds/tth_threshold/orange", TTH_estimation_thresh.orange, 300.f);
    // nh.param("/PBM/thresholds/tth_threshold/green", TTH_estimation_thresh.green, 900.f);
    // pbmbs.m_components_thresholds[Components::TIME_TO_HOME] = TTH_estimation_thresh;

    std::map<std::string, std::string> loaded_policies_from_params;
    if (nh.getParam("/PBM/available_policies", loaded_policies_from_params))
    {
        for (const auto &policy_path : loaded_policies_from_params)
        {
            ROS_INFO_STREAM("Loaded policy: " << policy_path.first << ". In path: " << policy_path.second);
            pbmbs.m_ros_params.loaded_policies.push_back(policy_path.second);
        }
    }
    else
    {
        ROS_ERROR_STREAM("Missing parameter: /PBM/available_policies");
        ret = false;
    }

    if (!nh.getParam("/PBM/topics_to_subscribe/battery_estimation", pbmbs.m_subscribers_topic[PBM_SUBSCRIBERS_TOPIC::BATTERY_ESTIMATION]))
    {
        ROS_ERROR_STREAM("Missing parameter: /PBM/topics_to_subscribe/battery_estimation");
        ret = false;
    }

    if (!nh.getParam("/PBM/topics_to_subscribe/dist_to_home", pbmbs.m_subscribers_topic[PBM_SUBSCRIBERS_TOPIC::DIST_FROM_HOME]))
    {
        ROS_ERROR_STREAM("Missing parameter: /PBM/topics_to_subscribe/dist_to_home");
        ret = false;
    }


    if (!nh.param("/motion_control/position/velocity_max_forward", pbmbs.m_ros_params.velocity_max_formard, 1.0f))
    {
        ROS_ERROR_STREAM("Missing parameter: /motion_control/position/velocity_max_forward. Setting default: 0.5f");
    }

    return ret;
}
