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
    pbmbs.m_publishers_dict[PBM_PUBLISHERS::COMMAND_LAND] = nh.advertise<geometry_msgs::InertiaStamped>(BPM_COMMANDS::TOPICS[BPM_COMMANDS::LAND_TO_FFK], 1, true);
    pbmbs.m_publishers_dict[PBM_PUBLISHERS::DEBUG_TREE_TICK_TIMING] = nh.advertise<std_msgs::Float32>(ros::this_node::getName() + "/debug/tree_tick_timming", 1, true);
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