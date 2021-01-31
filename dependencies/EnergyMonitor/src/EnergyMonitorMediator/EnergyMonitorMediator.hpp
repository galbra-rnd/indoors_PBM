#pragma once

#include "IEnergyMonitorMediator.hpp"
class EnergyMonitorMediator: public IEnergyMonitorMediator
{
public:
    void GetSomeData() override;
    EnergyMonitorMediator(/* args */);
    ~EnergyMonitorMediator();    
private:
    /* data */

};


