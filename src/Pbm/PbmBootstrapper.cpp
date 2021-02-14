
#include "PbmBootstrapper.hpp"

bool PbmBootStrapper::SetCommandService(indoors_common::Cmd::Request &req, indoors_common::Cmd::Response &res)
{
    // std::string errMsg;
    // auto cmd = string_to_command(req.command);
    // res.isSucceeded = true;
    // switch (cmd)
    // {
    // case PointAndGoCommands::POINT_AND_GO:
    //     setMode(PointAndGoModes::OPER);
    //     break;
    // case PointAndGoCommands::STOP:
    //     if (!m_Bithandler.SetBitMessage(1004))
    //     {
    //         ROS_ERROR_STREAM("ID 1004 is not available in PointAndGoStatuses");
    //     }
    //     setMode(PointAndGoModes::IDLE);
    //     break;
    // case PointAndGoCommands::RESET:
    //     handle_reset_command();
    //     break;
    // default:
    //     res.isSucceeded = false;
    //     break;
    // }

    return true;
}

void PbmBootStrapper::SetPolicy(const std::string &policy)
{
    if (policy.size() > 0)
    {
        std::cout << policy << '\n';
        m_policy = policy;
        m_is_policy_set = true;
    }
}

void PbmBootStrapper::register_all_nodes(std::shared_ptr<IEnergyMonitorMediatorDataProvider> energy_data_provider)
{
    // IsBatteryOk batteryOk_node("IsBatteryOk", energy_data_provider);
    auto batteryOk_node = std::make_shared<IsBatteryOk>("IsBatteryOk", energy_data_provider);
    m_factory.registerSimpleCondition("IsBatteryOk", std::bind(&IsBatteryOk::tick, batteryOk_node));

    auto goHomeOkConditionNode_node = std::make_shared<IsGoHomeOkConditionNode>("IsGoHomeOkConditionNode", energy_data_provider);
    m_factory.registerSimpleCondition("IsGoHomeOkConditionNode", std::bind(&IsGoHomeOkConditionNode::tick, goHomeOkConditionNode_node));
}

void PbmBootStrapper::load_policy(int policy)
{
    if (m_ros_params.loaded_policies.size() < policy)
    {
        ROS_ERROR_STREAM("No such policy loaded. There are only " << m_ros_params.loaded_policies.size() << " policies loaded.");
        return;
    }

    std::string policy_base = PROJECT_SOURCE_DIR;

    std::string policy_setting = policy_base + "/" + m_ros_params.loaded_policies[policy] + "/Policy.yaml";
    YAML::Node config = YAML::LoadFile(policy_setting);

    load_thresholds(config["thresholds"]);
    load_factors(config["factors"]);
    load_missions(config["available_missions"]);

    std::string policy_behavior = policy_base + "/" + m_ros_params.loaded_policies[policy] + "/Policy.xml";
    SetPolicy(policy_behavior.c_str());
}

void PbmBootStrapper::load_factors(const YAML::Node &factors)
{
    auto factors_map = factors.as<std::map<std::string, float>>();
    for (const auto &i : factors_map)
    {
        if (i.first == "time_to_home")
            m_ros_params.factors.time_to_home = i.second;
        ROS_INFO_STREAM(i.first << " " << i.second);
    }
}

void PbmBootStrapper::load_missions(const YAML::Node &missions)
{
    m_available_missions = missions.as<std::vector<std::string>>();

    std::vector<MissionsAvailable> loaded_missions;
    for (const auto &mission : m_available_missions)
    {
        ROS_INFO_STREAM("Found mission: " << mission);
        MissionsAvailable new_mission;
        auto available_mission = string_to_available_mission(mission);
        if (available_mission == 0)
        {
            ROS_WARN_STREAM("Mission: " << mission << ". Is not implemented");
        }
        else
        {
            loaded_missions.push_back(available_mission);
        }
    }
    m_energy_data_mediator->LoadMissions(loaded_missions);
}

void PbmBootStrapper::load_thresholds(const YAML::Node &thresholds)
{
    for (const auto &threshold : thresholds)
    {
        auto key = threshold.first.as<std::string>();
        ROS_INFO_STREAM(key);
        if (key == "battery_threshold")
            handle_battery_thresholds(threshold.second, *this);
        if (key == "tth_threshold")
            handle_tth_threshold(threshold.second, *this);
    }
    m_energy_data_mediator->LoadThresholdValues(m_components_thresholds);
}

void PbmBootStrapper::createTree()
{
    // Trees are created at deployment-time (i.e. at run-time, but only once at the beginning).
    // The currently supported format is XML.
    // IMPORTANT: when the object "tree" goes out of scope, all the TreeNodes are destroyed
    if (m_is_policy_set)
    {
        ROS_INFO_STREAM(m_policy);
        m_generated_tree = m_factory.createTreeFromFile(m_policy);
    }
}
void PbmBootStrapper::handel_mission_data(const Missions &missions_data)
{
    publish_current_missions_status(missions_data, *this);
}

void PbmBootStrapper::init()
{
    init_subscribers(*this, m_nh);
    init_publishers(*this, m_nh);

    std::shared_ptr<EnergyMonitorMediator> energy_monitor = std::make_shared<EnergyMonitorMediator>();

    /**
   * @brief This IEnergyMonitorMediator can insert new data into the EnergyMonitorMediator.
   * Also, It provides acces to all PossibleMissions and their status.
   * 
   */
    m_energy_data_mediator = energy_monitor;

    load_policy(0);

    /**
   * @brief This IEnergyMonitorMediatorDataProvider have acces to inserted data.
   * And can change MissionAvailability according to tests performed in TreeNodes.
   * 
   */
    std::shared_ptr<IEnergyMonitorMediatorDataProvider> energy_data_provider = energy_monitor;
    register_all_nodes(std::move(energy_data_provider));

    createTree();


    // auto pub = nh.advertise<std_msgs::String>("/bla", 1);
    // std::unordered_map<PBM_PUBLISHERS,ros::Publisher> m_publishers_dict;

    // Test1
    // float VALID_BATTERY_ESTIMATION = 501; /** Needs to be above ORANGE threshold  */
    // energy_data_mediator->SetBatteryEstimation(VALID_BATTERY_ESTIMATION);

    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.

    // ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
}

void PbmBootStrapper::Spin()
{
    try
    {
        m_generated_tree.tickRoot();
        auto missions = m_energy_data_mediator->GetMissionsData();
        handel_mission_data(missions);
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << "::" << e.what());
        m_Bithandler.SetBitMessage(BASE + 1);
        std::cerr << e.what() << '\n';
    }
}

PbmBootStrapper::PbmBootStrapper(ros::NodeHandle &nh)
    : m_nh(nh)
{

    // report version
    ROS_INFO_STREAM(PROJECT_NAME << " Initiated."
                                 << " version: " << PROJECT_VER);

    if (!m_Bithandler.SetBitMessage(1000))
    {
        ROS_ERROR_STREAM("ID 1000 is not available in PBMStatuses");
    }

    if (!init_params(*this, nh))
    {
        if (!m_Bithandler.SetBitMessage(1098))
        {
            ROS_ERROR_STREAM("ID 1098 is not available in PBMStatuses");
        }
    }
    else
    {
        init();
    }

    m_BitPublisher = nh.advertise<diagnostic_msgs::DiagnosticArray>(std::string("/bit/" + ros::this_node::getName()), 1);
    m_set_command = nh.advertiseService(ros::this_node::getName() + std::string("/set_command"), &PbmBootStrapper::SetCommandService, this);
}

PbmBootStrapper::~PbmBootStrapper()
{
}

void PbmBootStrapper::PublishBIT()
{
    diagnostic_msgs::DiagnosticArray msg;
    m_Bithandler.HandleBitArray(msg);
    m_BitPublisher.publish(msg);
}