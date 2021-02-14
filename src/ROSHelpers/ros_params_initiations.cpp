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

    // if (!nh.getParam("/PBM/available_missions", pbmbs.m_available_missions))
    // {
    //     ROS_ERROR_STREAM("Missing parameter: /PBM/available_missions");
    //     ret = false;
    // }

    // if (!nh.param("/PBM/factors/time_to_home", pbmbs.m_ros_params.factors.time_to_home, 1.0f))
    // {
    //     ROS_ERROR_STREAM("Missing parameter: /PBM/factors/time_to_home. Setting default: 0.5f");
    // }

    if (!nh.param("/motion_control/position/velocity_max_forward", pbmbs.m_ros_params.velocity_max_formard, 1.0f))
    {
        ROS_ERROR_STREAM("Missing parameter: /motion_control/position/velocity_max_forward. Setting default: 0.5f");
    }

    // nh.param(ros::this_node::getName() + "/add_weighted_params/alpha", pg.m_add_weighted_params.alpha, 0.5);
    // nh.param(ros::this_node::getName() + "/add_weighted_params/beta", pg.m_add_weighted_params.beta, 1.0);
    // nh.param(ros::this_node::getName() + "/add_weighted_params/gamma", pg.m_add_weighted_params.gamma, 5.0);

    // nh.param(ros::this_node::getName() + "/mpc_statuses/sliding_window_settings/window_size", pg.m_mpc_statuses.sliding_window_settings.window_size, 4);
    // nh.param(ros::this_node::getName() + "/mpc_statuses/sliding_window_settings/thresh_for_setting_false", pg.m_mpc_statuses.sliding_window_settings.THRES_FOR_SETING_FALSE, -8);
    // nh.param(ros::this_node::getName() + "/mpc_statuses/sliding_window_settings/thresh_for_setting_true", pg.m_mpc_statuses.sliding_window_settings.THRES_FOR_SETING_TRUE, -4);
    // nh.param(ros::this_node::getName() + "/mpc_statuses/waiting_for_wall_heading_timeout", pg.m_mpc_statuses.WALL_ALIGN_TIMEOUT, 10);
    // nh.param(ros::this_node::getName() + "/mpc_statuses/waiting_for_mpc_wall_heading_timeout", pg.m_mpc_statuses.MPC_WALL_ALIGN_TIMEOUT, 50);
    // nh.param(ros::this_node::getName() + "/mpc_statuses/lost_tracker_timer", pg.m_mpc_statuses.LOST_TRACKER_TIMEOUT, 1);
    // nh.param(ros::this_node::getName() + "/mpc_statuses/heading_error_threshold_rad", pg.m_mpc_statuses.HEADING_ERROR_THRESHOLD_RAD, 0.0017f);
    // nh.param(ros::this_node::getName() + "/mpc_statuses/set_point_changed_counter", pg.m_mpc_statuses.SET_POINT_CHANGED_COUNTER, 10);
    // nh.param(ros::this_node::getName() + "/mpc_statuses/maximum_tracker_reopen_tries", pg.m_mpc_statuses.MAX_TRACKER_REOPEN_TRIES, 5);
    // nh.param(ros::this_node::getName() + "/mpc_statuses/distance_dead_zone", pg.m_mpc_statuses.DISTANCE_DEAD_ZONE, 0.5f);

    // ROS_INFO_STREAM("m_mpc_statuses.SET_POINT_CHANGED_COUNTER: " << pg.m_mpc_statuses.SET_POINT_CHANGED_COUNTER);

    // nh.param(ros::this_node::getName() + "/benchmark", pg.benchmark, false);
    // if (pg.benchmark)
    //     ROS_INFO_STREAM("param " << ros::this_node::getName() << "/benchmark found. Benchmarking.");

    // if (!nh.param(ros::this_node::getName() + "/show_debug_images", pg.show_debug_images, false))
    // {
    //     ROS_INFO_STREAM("No param " << ros::this_node::getName() << "/show_debug_images set. Setting defult: false");
    // }

    // // Drone Position in LaserScanImage
    // if (!nh.param(ros::this_node::getName() + "/laser_scan_params/laser_scan_tracker_confidence_threshold", pg.m_laser_image_params.TRACKER_CONFIDENCE_THRESHOLD, 0.5f))
    // {
    //     ROS_WARN_STREAM("No param " << ros::this_node::getName() << "/laser_scan_params/laser_scan_tracker_confidence_threshold set. So setting default TRACKER_CONFIDENCE_THRESHOLD: 0.5ff");
    // }
    // if (!nh.param(ros::this_node::getName() + "/laser_scan_params/laser_scan_image_resolution", pg.m_laser_image_params.resolution, 0.1f))
    // {
    //     ROS_WARN_STREAM("No param " << ros::this_node::getName() << "/laser_scan_params/laser_scan_image_resolution set. So setting default laser_scan_image_resolution: 0.1f");
    // }
    // if (!nh.param(ros::this_node::getName() + "/laser_scan_params/max_range", pg.m_laser_image_params.max_range, 40.0f))
    // {
    //     ROS_WARN_STREAM("No param " << ros::this_node::getName() << "/laser_scan_params/max_range set. So setting default max_range: 40.0f");
    // }
    // if (!nh.param(ros::this_node::getName() + "/laser_scan_params/drone_position_meters/distance_from_left", pg.m_laser_image_params.drone_shift_from_left_m, 5.0f))
    // {
    //     ROS_WARN_STREAM("No param " << ros::this_node::getName() << "/laser_scan_params/drone_position_meters/distance_from_left set. So setting default drone_shift_from_left_m: 5.0f");
    // }
    // pg.m_laser_image_params.UpdateSettings();

    // // Topics
    // if (!nh.param(ros::this_node::getName() + "/incomming_mpc_range_topic", pg.incomming_mpc_range_topic, std::string("")))
    // {
    //     ROS_ERROR_STREAM("No param " << ros::this_node::getName() << "/incomming_mpc_range_topic set. Please set this parameter in: src/Indoors_main/Indoors_Point_n_Go/config/PointAndGo.yaml.\nExiting.");
    //     return false;
    // }
    // if (!nh.param(ros::this_node::getName() + "/incomming_rp_scan_topic", pg.incomming_rp_scan_topic, std::string("")))
    // {
    //     ROS_ERROR_STREAM("No param " << ros::this_node::getName() << "/incomming_rp_scan_topic set. Please set this parameter in: src/Indoors_main/Indoors_Point_n_Go/config/PointAndGo.yaml.\nExiting.");
    //     return false;
    // }
    // if (!nh.param(ros::this_node::getName() + "/incomming_wall_heading_ang_topic", pg.incomming_wall_heading_ang_topic, std::string("")))
    // {
    //     ROS_ERROR_STREAM("No param " << ros::this_node::getName() << "/incomming_wall_heading_ang_topic set. Please set this parameter in: src/Indoors_main/Indoors_Point_n_Go/config/PointAndGo.yaml.\nExiting.");
    //     return false;
    // }
    // if (!nh.param(ros::this_node::getName() + "/incomming_set_point_changed_topic", pg.incomming_set_point_changed_topic, std::string("")))
    // {
    //     ROS_ERROR_STREAM("No param " << ros::this_node::getName() << "/incomming_set_point_changed_topic set. Please set this parameter in: src/Indoors_main/Indoors_Point_n_Go/config/PointAndGo.yaml.\nExiting.");
    //     return false;
    // }
    // if (!nh.param(ros::this_node::getName() + "/incomming_tracked_set_point_topic", pg.incomming_tracked_set_point_topic, std::string("")))
    // {
    //     ROS_ERROR_STREAM("No param " << ros::this_node::getName() << "/incomming_tracked_set_point_topic set. Please set this parameter in: src/Indoors_main/Indoors_Point_n_Go/config/PointAndGo.yaml.\nExiting.");
    //     return false;
    // }
    // if (!nh.param(ros::this_node::getName() + "/incomming_pixle_topic", pg.incomming_pixle_topic, std::string("")))
    // {
    //     ROS_ERROR_STREAM("No param " << ros::this_node::getName() << "/incomming_pixle_topic set. Please set this parameter in: src/Indoors_main/Indoors_Point_n_Go/config/PointAndGo.yaml.\nExiting.");
    //     return false;
    // }
    // if (!nh.param(ros::this_node::getName() + "/outgoing_delta_topic", pg.outgoing_delta_topic, std::string("/DEFAULT_TOPIC_delta_topic")))
    // {
    //     ROS_ERROR_STREAM("No param " << ros::this_node::getName() << "/outgoing_delta_topic set. Setting default: /DEFAULT_TOPIC_delta_topic");
    //     return false;
    // }
    // if (!nh.param(ros::this_node::getName() + "/mode_topic", pg.outgoing_mode_topic, std::string("/DEFAULT_TOPIC_mode_topic")))
    // {
    //     ROS_ERROR_STREAM("No param " << ros::this_node::getName() << "/mode_topic set. Setting default: /DEFAULT_TOPIC_mode_topic");
    //     return false;
    // }
    // if (!nh.param(ros::this_node::getName() + "/ir_image/FovX", pg.m_laser_image_params.ir_fov.fovX, 59.0f))
    // {
    //     ROS_ERROR_STREAM("No param " << ros::this_node::getName() << "/ir_image/FovX set. Setting default: 59.");
    // }
    // if (!nh.param(ros::this_node::getName() + "/ir_image/Padding", pg.m_laser_image_params.ir_fov.padding, 5.0f))
    // {
    //     ROS_ERROR_STREAM("No param " << ros::this_node::getName() << "/ir_image/Padding set. Setting default: 5.");
    // }
    // // if (!nh.param(ros::this_node::getName() + "/FovY", pg.m_FovY, 46.0))
    // // {
    // //     ROS_ERROR_STREAM("No param " << ros::this_node::getName() << "/FovX set. Setting default: 46.");
    // // }
    // if (!nh.param(ros::this_node::getName() + "/cvTracker", pg.m_trackerType, std::string("MOSSE")))
    // {
    //     ROS_ERROR_STREAM("No param " << ros::this_node::getName() << "/cvTracker set. Setting default: MOSSE.");
    // }
    // // if (!nh.param(ros::this_node::getName() + "/fixed_boundingbox", pg.m_is_fixed_boundingbox, false))
    // // {
    // //     ROS_WARN_STREAM("No param " << ros::this_node::getName() << "/fixed_boundingbox. Setting default: false");
    // // }
    // if (!nh.param(ros::this_node::getName() + "/boundingbox_dimension", pg.m_boundingbox_dimension, std::map<std::string, float>{{"width", 50}, {"height", 50}}))
    // {
    //     ROS_WARN_STREAM("No param " << ros::this_node::getName() << "/boundingbox_dimension set. So setting default boundingbox_dimension: [width:50,height:50] ."); // << ros::this_node::getName() << "/param_roi set. So service default m_sRoi.roi: [0,0,0,0] .");
    // }
    // if (!nh.param(ros::this_node::getName() + "/safety_distance", pg.m_safety_distance_M, 4.0f))
    // {
    //     pg.m_mpc_statuses.range_to_obstacle.data = pg.m_safety_distance_M;
    //     pg.m_mpc_statuses.MIN_RANGE_TO_OBSTACLE = pg.m_safety_distance_M;
    //     ROS_WARN_STREAM("No param " << ros::this_node::getName() << "/safety_distance set. So setting default boundingbox_dimension: 4[m]");
    // }

    // if (!nh.param(ros::this_node::getName() + "/obstacle_ahead_threshold", pg.obstacle_ahead_threshold, 2.0f))
    // {
    //     ROS_WARN_STREAM("No param " << ros::this_node::getName() << "/obstacle_ahead_threshold set. So setting default obstacle_ahead_threshold: 2[m]");
    // }
    return ret;
}
