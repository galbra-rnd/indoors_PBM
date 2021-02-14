
#include "Pbm/PbmBootstrapper.hpp"

bool handle_battery_thresholds(const YAML::Node &threshold, PbmBootStrapper &pbmbs)
{
    ThresholdValues bat_time_left_estimation_thresh;
    auto settings_map = threshold.as<std::map<std::string, float>>();
    for (const auto &i : settings_map)
    {
        if (i.first == "red")
            bat_time_left_estimation_thresh.red = i.second;
        if (i.first == "orange")
            bat_time_left_estimation_thresh.orange = i.second;
        if (i.first == "green")
            bat_time_left_estimation_thresh.green = i.second;
        ROS_INFO_STREAM(i.first << " " << i.second);
    }
    pbmbs.m_components_thresholds[Components::BATTERY_TIME] = bat_time_left_estimation_thresh;
    return true;
};

bool handle_tth_threshold(const YAML::Node &threshold, PbmBootStrapper &pbmbs)
{
    ThresholdValues bat_time_left_estimation_thresh;
    auto settings_map = threshold.as<std::map<std::string, float>>();
    for (const auto &i : settings_map)
    {
        if (i.first == "red")
            bat_time_left_estimation_thresh.red = i.second;
        if (i.first == "orange")
            bat_time_left_estimation_thresh.orange = i.second;
        if (i.first == "green")
            bat_time_left_estimation_thresh.green = i.second;
        ROS_INFO_STREAM(i.first << " " << i.second);
    }
    pbmbs.m_components_thresholds[Components::TIME_TO_HOME] = bat_time_left_estimation_thresh;
    return true;
};
// bool adapt_available_policies(std::map<std::string, std::string> loaded_policies_from_params, PbmBootStrapper &pbmbs)
// {
//     std::vector<std::string> policies_vector;
//     for (const auto &policy_path : loaded_policies_from_params)
//     {
//         ROS_INFO_STREAM("Loaded policy: " << policy_path.first << ". In path: " << policy_path.second);
//         pbmbs.m_ros_params.loaded_policies.push_back(policy_path.second);
//     }
//     ROS_INFO_STREAM("Loaded policy: " << policies_vector[0]);
// }
// /**
//  * @brief Convert distance from home, into Time to home.
//  *
//  * @param[in] distance_to_home float - given from a PathPlanner subscriber.
//  * @param[out] time_to_home float& - return value.
//  * @param[in] pbmbs PbmBootStrapper & - a PBM reference for getting the Velocity parameter.
//  * @return true - if convertion succeeded
//  * @return false - otherwise.
//  */
// bool convert_dist_to_time(float distance_to_home, float &time_to_home, PbmBootStrapper &pbmbs)
// {

// }