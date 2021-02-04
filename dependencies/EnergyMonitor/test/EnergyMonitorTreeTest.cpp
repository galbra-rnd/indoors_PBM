#include "gtest/gtest.h"
#include <memory>

#include "mocks/mock_EnergyMonitorMediator.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "TreeNodes/IsBatteryOkConditionNode.hpp"
/**
 * @test Create BatteryEstimationConditionNode test, and see if it returns Success of Failure as expected.
 * 
 * Setup: \n
 * * Define a tree \n
 * * Create an energy_data_provider (implementing MockEnergyMonitorMediator)
 * * Create a tree factory.
 * Run: \n
 * * Create an IsBatteryOk node.
 * * Dependency inject the energy_data_provider
 * * Register this Node into created tree factory.
 * * Create a tree from tree factory.
 * Test1: \n
 * * Set VALID_BATTERY_ESTIMATION data in energy_data_provider.battery
 * * ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
 * Test2: \n
 * * Set IVALID_BATTERY_ESTIMATION data in energy_data_provider.battery
 * * ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
 */
TEST(EnergyMonitorTreeTest, BatteryEstimationConditionNode)
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
  float VALID_BATTERY_ESTIMATION = 2;
  energy_data_mediator->SetBatteryEstimation(VALID_BATTERY_ESTIMATION);

  // To "execute" a Tree you need to "tick" it.
  // The tick is propagated to the children based on the logic of the tree.
  // In this case, the entire sequence is executed, because all the children
  // of the Sequence return SUCCESS.
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

  // Test2
  float IVALID_BATTERY_ESTIMATION = 0.9;
  energy_data_mediator->SetBatteryEstimation(IVALID_BATTERY_ESTIMATION);

  // To "execute" a Tree you need to "tick" it.
  // The tick is propagated to the children based on the logic of the tree.
  // In this case, the entire sequence is executed, because all the children
  // of the Sequence return SUCCESS.
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);

}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
