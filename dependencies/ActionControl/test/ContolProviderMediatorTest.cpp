#include "gtest/gtest.h"
#include <memory>
#include "ControlProviderMediator/ControlProviderMediator.hpp"
/** 
 * EnergyMonitorMediatorDataManagerTest test the IEnergyMonitorMediatorDataProvider.
 * An interface EnergyMonitorMediator expose for the Behavior trees.
 */

/**
 * @test Test Behavior trees interface.
 * This simple test has two parts: \n
 * Test:
 * * Insert new data using the IEnergyMonitorMediator interface. This interface will be exposed to EnergyMonitor. Through it, new data is inserted.
 * * Perform some evaluation inside a TreeNode, and set new mission status using the IEnergyMonitorMediatorDataProvider interface, that EnergyMonitor expose to Behavior trees.
 * 
 */
TEST(ContolProviderMediatorTest, FunctionInitiationTest)
{
  // Setup
  std::unordered_map<ControlAction::Commands, std::function<void()>> control_action_dict;

  auto simple_func = []() { std::cout << "\ntemp\n"; };
  control_action_dict[ControlAction::Commands::Land] = simple_func;

  std::shared_ptr<IControlProviderMediator> temp = std::make_shared<ControlProviderMediator>(control_action_dict,spdlog::stdout_color_mt("FunctionInitiationTest"));
  temp->Land();
}

TEST(ContolProviderMediatorTest, CallingEmptyFunctionTest)
{
  // Setup
  std::unordered_map<ControlAction::Commands, std::function<void()>> control_action_dict;

  auto simple_func = []() { std::cout << "\ntemp\n"; };
  control_action_dict[ControlAction::Commands::Land] = simple_func;

  std::shared_ptr<IControlProviderMediator> temp = std::make_shared<ControlProviderMediator>(control_action_dict,spdlog::stdout_color_mt("CallingEmptyFunctionTest"));
  temp->GoHome();
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
