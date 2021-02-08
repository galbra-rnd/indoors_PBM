#include "gtest/gtest.h"
#include <memory>

#include "EnergyMonitor.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "TreeNodes/IsBatteryOkConditionNode.hpp"
#include "TreeNodes/IsGoHomeOkConditionNode.hpp"
/**
 * @test Create BatteryEstimationConditionNode test, and see if it returns Success of Failure as expected.
 * 
 * Setup: \n
 * * Define a tree \n
 * * Create an energy_data_provider (implementing MockEnergyMonitorMediator)
 * * Create a tree factory.
 * Run: \n
 * * Create an IsBatteryOk node.
 * * Load battery threshholds
 * * Dependency inject the energy_data_provider
 * * Register this Node into created tree factory.
 * * Create a tree from tree factory.
 * Test1: \n
 * * Set VALID_BATTERY_ESTIMATION data in energy_data_provider.battery
 * * ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
 * Test2: \n
 * * Set INVALID_BATTERY_ESTIMATION data in energy_data_provider.battery
 * * ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
 */
TEST(EnergyMonitorTreeBatteyNodeOnly, BatteryEstimationConditionNode)
{
  /**
   * @brief Create the most simple Behavior tree.
   * This tree contain's an empty implementation of IsBatteryOk Node.
   * 
   */
  static const char *xml_text = R"(

  <root main_tree_to_execute = "MainTreeBatteryTest" >

      <BehaviorTree ID="MainTreeBatteryTest">
          <ReactiveSequence name="root_sequence">
              <IsBatteryOk   name="IS battery Ok ConditionNode"/>
          </ReactiveSequence>
      </BehaviorTree>

  </root>
  )";
  // Setup
  std::shared_ptr<EnergyMonitorMediator> energy_monitor = std::make_shared<EnergyMonitorMediator>();

  /**
   * @brief This IEnergyMonitorMediator can insert new data into the EnergyMonitorMediator.
   * Also, It provides acces to all PossibleMissions and their status.
   * 
   */
  std::shared_ptr<IEnergyMonitorMediator> energy_data_mediator = energy_monitor;

  int RED_THRESH = 250;
  int GREEN_THRESH = 500;

  std::unordered_map<Components, ThresholdValues> components_thresholds;
  ThresholdValues bat_thresh;
  bat_thresh.red = RED_THRESH;
  bat_thresh.orange = 300;
  bat_thresh.green = GREEN_THRESH;
  components_thresholds[Components::BATTERY_TIME] = bat_thresh;

  energy_data_mediator->LoadThresholdValues(components_thresholds);

  /**
   * @brief This IEnergyMonitorMediatorDataProvider have acces to inserted data.
   * And can change MissionAvailability according to tests performed in TreeNodes.
   * 
   */
  std::shared_ptr<IEnergyMonitorMediatorDataProvider> energy_data_provider = energy_monitor;

  // We use the BehaviorTreeFactory to register our custom nodes
  BT::BehaviorTreeFactory factory;

  // Run
  IsBatteryOk batteryOk_node("IsBatteryOk", energy_data_provider);
  factory.registerSimpleCondition("IsBatteryOk", std::bind(&IsBatteryOk::tick, &batteryOk_node));

  // Trees are created at deployment-time (i.e. at run-time, but only once at the beginning).
  // The currently supported format is XML.
  // IMPORTANT: when the object "tree" goes out of scope, all the TreeNodes are destroyed
  auto tree = factory.createTreeFromText(xml_text);

  // Test1
  float VALID_BATTERY_ESTIMATION = 501; /** Needs to be above ORANGE threshold  */
  energy_data_mediator->SetBatteryEstimation(VALID_BATTERY_ESTIMATION);

  // To "execute" a Tree you need to "tick" it.
  // The tick is propagated to the children based on the logic of the tree.
  // In this case, the entire sequence is executed, because all the children
  // of the Sequence return SUCCESS.
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

  // Test2
  float INVALID_BATTERY_ESTIMATION = RED_THRESH - 1;
  energy_data_mediator->SetBatteryEstimation(INVALID_BATTERY_ESTIMATION);

  // To "execute" a Tree you need to "tick" it.
  // The tick is propagated to the children based on the logic of the tree.
  // In this case, the entire sequence is executed, because all the children
  // of the Sequence return SUCCESS.
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
}

/**
 * @test Test if setting bad battery estimation cause mission FLY to become MISSION::RED
 * 
 * Setup: \n
 * * Define a tree \n
 * * Create an energy_data_provider \n
 * * Create a tree factory. \n
 * Run: \n
 * * Load FLY mission into IEnergyMonitor.
 * * Create an IsBatteryOk node.
 * * Dependency inject the energy_data_provider
 * * Register this Node into created tree factory.
 * * Create a tree from tree factory.
 * Test: \n
 * * Set INVALID_BATTERY_ESTIMATION data in energy_data_provider.battery
 * * See if relevant loaded mission become MISSION::RED
 */
TEST(EnergyMonitorTreeBatteyNodeOnlyTest, AvailableMissions)
{
  /**
   * @brief Create the most simple Behavior tree.
   * This tree contain's an empty implementation of IsBatteryOk Node.
   * 
   */
  static const char *xml_text = R"(

  <root main_tree_to_execute = "MainTreeBatteryTest" >

      <BehaviorTree ID="MainTreeBatteryTest">
          <ReactiveSequence name="root_sequence">
              <IsBatteryOk   name="IS battery Ok ConditionNode"/>
          </ReactiveSequence>
      </BehaviorTree>

  </root>
  )";
  // Setup
  std::shared_ptr<EnergyMonitorMediator> energy_monitor = std::make_shared<EnergyMonitorMediator>();
  std::vector<MissionsAvailable> missions{MissionsAvailable::FLY, MissionsAvailable::GO_HOME};

  /**
   * @brief This IEnergyMonitorMediator can insert new data into the EnergyMonitorMediator.
   * Also, It provides acces to all PossibleMissions and their status.
   * 
   */
  std::shared_ptr<IEnergyMonitorMediator> energy_data_mediator = energy_monitor;

  // Prepare components threshold
  int RED_THRESH = 250;
  int GREEN_THRESH = 500;

  std::unordered_map<Components, ThresholdValues> components_thresholds;
  ThresholdValues bat_thresh;
  bat_thresh.red = RED_THRESH;
  bat_thresh.orange = 300;
  bat_thresh.green = GREEN_THRESH;
  components_thresholds[Components::BATTERY_TIME] = bat_thresh;

  // Load thresholds into IEnergyMonitorMediator.
  energy_data_mediator->LoadThresholdValues(components_thresholds);

  /**
   * @brief This IEnergyMonitorMediatorDataProvider have acces to inserted data.
   * And can change MissionAvailability according to tests performed in TreeNodes.
   * 
   */
  std::shared_ptr<IEnergyMonitorMediatorDataProvider> energy_data_provider = energy_monitor;

  // We use the BehaviorTreeFactory to register our custom nodes
  BT::BehaviorTreeFactory factory;

  // Run
  energy_monitor->LoadMissions(missions);

  /**
   * @brief Sanity check. Test if all newly loaded mission are green.
   * 
   */
  std::cout << "Sanity check: \n";
  for (const auto &mission : energy_monitor->GetMissionsData().missions)
  {
    std::cout << "Mission: " << mission_to_string(mission.mission) << "\n";
    std::cout << "Availability: " << mission_state_to_string(mission.state) << "\n\n";
    ASSERT_EQ(mission.state, MISSION::GREEN);
  }

  IsBatteryOk batteryOk_node("IsBatteryOk", energy_data_provider);
  factory.registerSimpleCondition("IsBatteryOk", std::bind(&IsBatteryOk::tick, &batteryOk_node));

  // Trees are created at deployment-time (i.e. at run-time, but only once at the beginning).
  // The currently supported format is XML.
  // IMPORTANT: when the object "tree" goes out of scope, all the TreeNodes are destroyed
  auto tree = factory.createTreeFromText(xml_text);

  // Test
  float INVALID_BATTERY_ESTIMATION = 0.9;
  energy_data_mediator->SetBatteryEstimation(INVALID_BATTERY_ESTIMATION);

  // To "execute" a Tree you need to "tick" it.
  // The tick is propagated to the children based on the logic of the tree.
  // In this case, the entire sequence is executed, because all the children
  // of the Sequence return SUCCESS.
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  for (const auto &mission : energy_monitor->GetMissionsData().missions)
  {
    std::cout << "Mission: " << mission_to_string(mission.mission) << "\n";
    std::cout << "Availability: " << mission_state_to_string(mission.state) << "\n\n";
  }
}

/**
 * @test Test if setting bad battery estimation cause mission FLY to become MISSION::RED
 * 
 * Setup: \n
 * * Define a tree \n
 * * Create an energy_data_provider \n
 * * Create a tree factory. \n
 * Run: \n
 * * Load FLY mission into IEnergyMonitor. \n
 * * Load GO_HOME mission into IEnergyMonitor. \n
 * * Create an IsBatteryOk node. \n
 * * Create an ISGoHomeOkConditionNode node. \n
 * * Dependency inject the energy_data_provider \n
 * * Register this Nodes into created tree factory. \n
 * * Create a tree from tree factory. \n
 * Test1: \n
 * * Set VALID_BATTERY_ESTIMATION data in energy_data_provider.battery \n
 * * See if relevant loaded mission become MISSION::GREEN \n
 * Test2: \n
 * * Slowly Raise VALID_TimeToHome data in energy_data_provider.GoHome \n
 * * See if relevant loaded mission transfuse from  MISSION::GREEN to MISSION::RED \n
 */
TEST(EnergyMonitorTreeBatteryAndGoHomeNodesTest, AvailableMissions)
{
  /**
   * @brief Create the a Behavior tree. with ReactiveSequence, and 2 nodes.
   * This tree contain's an empty implementation of IsBatteryOk Node.
   * 
   */
  static const char *xml_text = R"(

  <root main_tree_to_execute = "MainTreeBatteryTest" >

      <BehaviorTree ID="MainTreeBatteryTest">
          <ReactiveSequence name="root_sequence">
              <IsBatteryOk   name="IS battery Ok ConditionNode"/>
              <ISGoHomeOkConditionNode   name="Is GoHome OK ConditionNode"/>
          </ReactiveSequence>
      </BehaviorTree>

  </root>
  )";
  // Setup
  std::shared_ptr<EnergyMonitorMediator> energy_monitor = std::make_shared<EnergyMonitorMediator>();
  std::vector<MissionsAvailable> missions{MissionsAvailable::FLY, MissionsAvailable::GO_HOME};

  /**
   * @brief This IEnergyMonitorMediator can insert new data into the EnergyMonitorMediator.
   * Also, It provides acces to all PossibleMissions and their status.
   * 
   */
  std::shared_ptr<IEnergyMonitorMediator> energy_data_mediator = energy_monitor;

  // Prepare components threshold
  int RED_THRESH = 250;
  int GREEN_THRESH = 500;

  std::unordered_map<Components, ThresholdValues> components_thresholds;
  ThresholdValues bat_thresh;
  bat_thresh.red = RED_THRESH;
  bat_thresh.orange = 300;
  bat_thresh.green = GREEN_THRESH;
  components_thresholds[Components::BATTERY_TIME] = bat_thresh;

  /**
   * @brief GO_HOME thresh is the relation between TTH (Time To Home, Given from PathPlanner) and the BatteryLeft estimation.
   * Suggested Threshold values:
   * * Assuming \f$BatteryLeft != 0\f$.
   * * RED    = 0.    It means that if \f$1 - \frac{TTH}{BatteryLeft} <= 0\f$, we cant go home.
   * * ORANGE = 0.5.  It means that if \f$1 - \frac{TTH}{BatteryLeft} <= 0.5\f$, we can go home. But battery is running out.
   * * GREEN  = 1.    It means that if \f$1 - \frac{TTH}{BatteryLeft} <= 1\f$, we can go home.
   */
  ThresholdValues GoHome_thresh;
  GoHome_thresh.red = 0;
  GoHome_thresh.orange = 0.5;
  GoHome_thresh.green = 1;
  components_thresholds[Components::TIME_TO_HOME] = GoHome_thresh;

  // Load thresholds into IEnergyMonitorMediator.
  energy_data_mediator->LoadThresholdValues(components_thresholds);

  /**
   * @brief This IEnergyMonitorMediatorDataProvider have acces to inserted data.
   * And can change MissionAvailability according to tests performed in TreeNodes.
   * 
   */
  std::shared_ptr<IEnergyMonitorMediatorDataProvider> energy_data_provider = energy_monitor;

  // We use the BehaviorTreeFactory to register our custom nodes
  BT::BehaviorTreeFactory factory;

  // Run
  energy_data_mediator->LoadMissions(missions);

  IsBatteryOk batteryOk_node("IsBatteryOk", energy_data_provider);
  factory.registerSimpleCondition("IsBatteryOk", std::bind(&IsBatteryOk::tick, &batteryOk_node));

  IsGoHomeOkConditionNode GoHomeOk_node("ISGoHomeOkConditionNode", energy_data_provider);
  factory.registerSimpleCondition("ISGoHomeOkConditionNode", std::bind(&IsGoHomeOkConditionNode::tick, &GoHomeOk_node));

  // Trees are created at deployment-time (i.e. at run-time, but only once at the beginning).
  // The currently supported format is XML.
  // IMPORTANT: when the object "tree" goes out of scope, all the TreeNodes are destroyed
  auto tree = factory.createTreeFromText(xml_text);

  // Test1
  float INVALID_BATTERY_ESTIMATION = GREEN_THRESH + 1;
  energy_data_mediator->SetBatteryEstimation(INVALID_BATTERY_ESTIMATION);

  // To "execute" a Tree you need to "tick" it.
  // The tick is propagated to the children based on the logic of the tree.
  // In this case, the entire sequence is executed, because all the children
  // of the Sequence return SUCCESS.
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  for (const auto &mission : energy_monitor->GetMissionsData().missions)
  {
    std::cout << "Mission: " << mission_to_string(mission.mission) << "\n";
    std::cout << "Availability: " << mission_state_to_string(mission.state) << "\n\n";
  }

  // Test2
  float VALID_TIME_TO_HOME_secs = 0; // At the beginning we are close to home
  while (VALID_TIME_TO_HOME_secs < GREEN_THRESH * 2)
  {
    energy_data_mediator->SetTimeToHomeEstimation(VALID_TIME_TO_HOME_secs);
    tree.tickRoot();
    VALID_TIME_TO_HOME_secs += 15;

    for (const auto &mission : energy_monitor->GetMissionsData().missions)
    {
      std::cout << "Mission: " << mission_to_string(mission.mission) << "\n";
      std::cout << "Availability: " << mission_state_to_string(mission.state) << "\n\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
