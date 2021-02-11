#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include "indoors_common/Cmd.h"
#include "ROSHelpers/ros_communication.hpp"
#include "ROSHelpers/bit/Bit.hpp"
#include "ROSHelpers/ros_params_containers.hpp"
#include "EnergyMonitorMediator/EnergyMonitorMediator.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "TreeNodes/IsBatteryOkConditionNode.hpp"
#include "TreeNodes/IsGoHomeOkConditionNode.hpp"
#include "PinkBoxMonitorConfig.h"
class PbmBootStrapper
{
    friend bool init_params(PbmBootStrapper &, ros::NodeHandle &);
    friend bool init_subscribers(PbmBootStrapper &, ros::NodeHandle &);
    friend bool init_publishers(PbmBootStrapper &, ros::NodeHandle &);
    friend void publish_current_missions_status(const Missions &missions_data, PbmBootStrapper &pbmbs);

private:
    const char *m_policy;
    ros::NodeHandle m_nh;
    /**
     * @brief We use the BehaviorTreeFactory to register our custom nodes
     * 
     */
    BT::BehaviorTreeFactory m_factory;
    BT::Tree m_generated_tree;
    std::shared_ptr<IEnergyMonitorMediator> m_energy_data_mediator;
    bool m_is_policy_set = false;
    std::unordered_map<Components, ThresholdValues> m_components_thresholds;
    std::unordered_map<PBM_PUBLISHERS, ros::Publisher> m_publishers_dict;
    std::unordered_map<PBM_SUBSCRIBERS_TOPIC, std::string> m_subscribers_topic;
    std::list<ros::Subscriber> m_subscribers;
    std::vector<std::string> m_available_missions;
    ros::Publisher m_BitPublisher;
    ros::ServiceServer m_set_command;
    Bit m_Bithandler;
    RosParams m_ros_params;
    void load_missions();
    void load_thresholds();
    void register_all_nodes(std::shared_ptr<IEnergyMonitorMediatorDataProvider> energy_data_provider);

    /**
     * @brief This is Pipeline entrance right after three tick.
     * For that moment it does nothing but calling publish_current_missions_status
     * 
     * @param missions_data - const Missions & filled with current mission availability (fresh out of last tick)
     */
    void handel_mission_data(const Missions &missions_data);
    void init();

public:
    /**
     * @brief Set the a policy.
     * 
     * @param policy const char* - representing tree_xml according to BT TreeForming language.
     */
    void SetPolicy(const char *policy);
    bool SetCommandService(indoors_common::Cmd::Request &req, indoors_common::Cmd::Response &res);
    void PublishBIT();
    void Spin();
    PbmBootStrapper(ros::NodeHandle &nh);
    ~PbmBootStrapper();
};