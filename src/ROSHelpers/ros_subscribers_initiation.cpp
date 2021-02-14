#include "Pbm/PbmBootstrapper.hpp"

/**
 * @brief Initiate module subscribers.
 * 
 * @param pbmbs PbmBootStrapper & a reference to the PinkBox module.
 * @param nh ros::NodeHandle & used for subscribing to other ros nodes
 * @return true 
 * @return false 
 */
bool init_subscribers(PbmBootStrapper &pbmbs, ros::NodeHandle &nh)
{

    bool ret = true;

    pbmbs.m_subscribers.push_back(nh.subscribe<std_msgs::Float32>(pbmbs.m_subscribers_topic[PBM_SUBSCRIBERS_TOPIC::BATTERY_ESTIMATION], 1, [&](const std_msgs::Float32ConstPtr &msg) {
        if (pbmbs.m_energy_data_mediator->SetBatteryEstimation(msg->data))
        {
            ROS_INFO_STREAM("Successfully inserted new battery estimation data: " << msg->data);
            return;
        }
        ROS_WARN_STREAM("BAD battery estimation received: " << msg->data);
    }));

    pbmbs.m_subscribers.push_back(nh.subscribe<std_msgs::Float32>(pbmbs.m_subscribers_topic[PBM_SUBSCRIBERS_TOPIC::DIST_FROM_HOME], 1, [&](const std_msgs::Float32ConstPtr &msg) {
        /**
         * @brief Time to home calculation: \n
         * \f$ TTH =\frac{DistToHome}{VehicleVelocity}\times factor\f$
         * 
         */
        
        float time_to_home;

        float distance_to_home = msg->data;
        auto vehicle_velocity = pbmbs.m_ros_params.velocity_max_formard;
        auto factor_time_to_home = pbmbs.m_ros_params.factors.time_to_home;

        time_to_home = (distance_to_home / vehicle_velocity) * factor_time_to_home;
        if (pbmbs.m_energy_data_mediator->SetTimeToHomeEstimation(time_to_home))
        {
            ROS_INFO_STREAM("Successfully inserted new time_to_home estimation data: " << time_to_home);
            return;
        }
        ROS_WARN_STREAM("BAD time_to_home estimation calculation: " << time_to_home);
        ROS_WARN_STREAM("received distance_to_home from pathplanner: " << msg->data);
        ROS_WARN_STREAM("received vehicle_velocity from parameters: " << vehicle_velocity);
        ROS_WARN_STREAM("received factor_time_to_home from parameters: " << factor_time_to_home);
        ROS_WARN_STREAM("calculated time_to_home: " << time_to_home);
    }));

    // pg.m_incommingPixleSub = nh.subscribe<geometry_msgs::InertiaStamped>(pg.incomming_pixle_topic, 1, &PointAndGo::pixleCallback, &pg);
    // pg.m_mpc_tracked_set_point_cb_sub = nh.subscribe<geometry_msgs::PoseStamped>(pg.incomming_tracked_set_point_topic, 1, &PointAndGo::mpc_tracked_set_point_cb, &pg);
    // pg.m_mpc_set_point_changed_cb_sub = nh.subscribe<std_msgs::Bool>(pg.incomming_set_point_changed_topic, 1, &PointAndGo::mpc_set_point_changed_cb, &pg);
    // pg.m_mpc_range_cb_sub = nh.subscribe<std_msgs::Float32>(pg.incomming_mpc_range_topic, 1, &PointAndGo::mpc_range_cb, &pg);
    // pg.m_laser_sub = nh.subscribe<sensor_msgs::LaserScan>(pg.incomming_rp_scan_topic, 1, &PointAndGo::laser_cb, &pg);
    // pg.m_wall_heading_cb_sub = nh.subscribe<std_msgs::Float32>(pg.incomming_wall_heading_ang_topic, 1, &PointAndGo::wall_heading_cb, &pg);

    return ret;
}