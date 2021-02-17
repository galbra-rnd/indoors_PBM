
#include "Pbm/PbmBootstrapper.hpp"
#include <sys/types.h>
#include <dirent.h>
#include <sys/stat.h>
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

const std::string get_log_file_name()
{
    DIR *dp;
    int num_of_files = 0;
    struct dirent *ep;
    auto homedir = getenv("HOME");
    
    strcat(homedir, "/.ros/PBM/");

    dp = opendir(homedir);

    if (dp == NULL)
    {
        auto res = mkdir(homedir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        dp = opendir(homedir);
    }
    if (dp != NULL)
    {
        while (ep = readdir(dp))
            num_of_files++;

        (void)closedir(dp);
    }
    num_of_files -= 2; // Reduce the '.' and the '..' files from num_of_files.

    std::string file_name("PBM_log_");

    file_name = homedir + file_name + std::to_string(num_of_files);
    return file_name;
}