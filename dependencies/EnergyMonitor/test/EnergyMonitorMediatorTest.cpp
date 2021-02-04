#include "gtest/gtest.h"
#include <memory>
#include "EnergyMonitor.hpp"

TEST(EnergyMonitorMediatorTest, SetBatteryTimeEstimation)
{
  std::shared_ptr<IEnergyMonitorMediator> em = std::make_shared<EnergyMonitorMediator>();
  ASSERT_TRUE(em->SetBatteryEstimation(350));
}

/**
 * @test Basic Test.
 * Test Getting battery estimation.
 * 
 */
TEST(EnergyMonitorMediatorTest, GetBatteryTimeEstimation)
{
  // Setup
  std::shared_ptr<IEnergyMonitorMediator> em = std::make_shared<EnergyMonitorMediator>();
  float stimation = 350;

  // Run
  em->SetBatteryEstimation(stimation);
  auto getter = std::dynamic_pointer_cast<IEnergyMonitorMediatorDataProvider>(em);

  // Test
  ASSERT_EQ(getter->GetTimeEstimation(DataMonitoringType::BatteryTimeLeft), stimation);
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
