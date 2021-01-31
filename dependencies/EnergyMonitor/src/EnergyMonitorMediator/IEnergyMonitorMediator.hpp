#pragma once
#include "iostream"
class IEnergyMonitorMediator
{
public:
    virtual void GetSomeData() = 0;
    virtual ~IEnergyMonitorMediator(){};
};