#include "Pbm/PbmBootstrapper.hpp"

bool init_publishers(PbmBootStrapper &pbmbs, ros::NodeHandle &nh)
{

    bool ret = true;

    pbmbs.m_publishers_dict[PBM_PUBLISHERS::VERSION] = nh.advertise<std_msgs::String>("/version/" + ros::this_node::getName(), 1, true);
    std_msgs::String name_and_version;
    name_and_version.data = PROJECT_VER;
    pbmbs.m_publishers_dict[PBM_PUBLISHERS::VERSION].publish(name_and_version);

    pbmbs.m_publishers_dict[PBM_PUBLISHERS::MISSIONS] = nh.advertise<diagnostic_msgs::DiagnosticArray>(ros::this_node::getName() + "/out/missions_status", 1, true);
    pbmbs.m_publishers_dict[PBM_PUBLISHERS::BIT] = nh.advertise<diagnostic_msgs::DiagnosticArray>("/bit/" + ros::this_node::getName(), 1, true);
    // pg.m_deltaPublisher = nh.advertise<geometry_msgs::PoseStamped>(pg.outgoing_delta_topic, 1);
    // pg.m_modePublisher = nh.advertise<std_msgs::String>(pg.outgoing_mode_topic, 1);

    // // Debug publishers
    // pg.m_publish_debug_timing_tracker_update = nh.advertise<std_msgs::Float32>("point_and_go/debug/timing/tracker_update", 1);
    // pg.m_publish_debug_timing_tracker_initiation = nh.advertise<std_msgs::Float32>("point_and_go/debug/timing/tracker_initiation", 1);
    // pg.m_publish_debug_timing_tracker_instantiation = nh.advertise<std_msgs::Float32>("point_and_go/debug/timing/tracker_instantiation", 1);
    // pg.m_publish_debug_remainig_heading_angle = nh.advertise<std_msgs::Float32>("point_and_go/debug/remainig_heading_angle", 1);
    // pg.m_publish_tracked_point_from_mpc = nh.advertise<geometry_msgs::PoseStamped>("point_and_go/debug/tracked_setpoint_from_mpc", 1);
    // pg.m_publish_debug_sliding_window_max_sum = nh.advertise<std_msgs::Float32>("point_and_go/debug/timing/sliding_window/max_sum", 1);
    // pg.m_publish_debug_sliding_window_window_sum = nh.advertise<std_msgs::Float32>("point_and_go/debug/timing/sliding_window/window_sum", 1);
    // pg.m_publish_debug_sliding_window_thresh_for_true = nh.advertise<std_msgs::Float32>("point_and_go/debug/timing/sliding_window/thresh_for_true", 1, true);
    // pg.m_publish_debug_sliding_window_thresh_for_false = nh.advertise<std_msgs::Float32>("point_and_go/debug/timing/sliding_window/thresh_for_false", 1, true);
    // // pg.m_publish_msg_tracker_update_tock = nh.advertise<std_msgs::Float32>("point_and_go/debug/timing/calc_time_tracker_update", 1);
    // pg.m_imgPublisher = nh.advertise<sensor_msgs::Image>(ros::this_node::getName() + "/laser_scan_image", 1);
    // pg.m_imageToAnchorPublisher = nh.advertise<sensor_msgs::Image>("point_and_go/point_and_go_anchor/laser_scan_image", 1);

    // // Wall heading
    // pg.m_publish_execute_to_wall_heading = nh.advertise<std_msgs::Bool>("/wall_heading/in/execute", 1);

    // pg.m_drone_position_in_laserscan_advertizer = nh.advertise<geometry_msgs::Point>("point_and_go/point_and_go_anchor/drone_position_in_laser_scan/", 1, true);
    return ret;
}

void publish_current_missions_status(const Missions &missions_data, PbmBootStrapper &pbmbs)
{
    diagnostic_msgs::DiagnosticArray missions_array;
    diagnostic_msgs::DiagnosticStatus missions_status;
    missions_status.name = "Loaded missions status";
    for (const auto &current_mission_status : missions_data.missions)
    {
        diagnostic_msgs::KeyValue mission_state;

        mission_state.key = mission_to_string(current_mission_status.mission);
        mission_state.value = mission_state_to_string(current_mission_status.state);
        missions_status.values.push_back(mission_state);
    }

    missions_array.status.push_back(missions_status);
    pbmbs.m_publishers_dict[PBM_PUBLISHERS::MISSIONS].publish(missions_array);
}