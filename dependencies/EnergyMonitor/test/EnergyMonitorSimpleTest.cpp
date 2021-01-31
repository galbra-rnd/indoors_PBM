#include "gtest/gtest.h"
#include "EnergyMonitor.hpp"

TEST (EnergyMonitorTest, FirstInitialisation) { 
    EnergyMonitor em;
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
