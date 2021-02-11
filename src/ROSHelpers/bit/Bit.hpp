#pragma once
#include <string>
#include <diagnostic_msgs/DiagnosticArray.h>
#include "ROSHelpers/bit/PBMStatuses.hpp"
#include <ros/ros.h>
#include <mutex>
#define TICKS_TO_OK 10 //sec
class Bit
{
public:
    Bit();
    Bit(ros::NodeHandle &nh, double pubFreq = 1);
    ~Bit();
    bool SetBitLevel(int);
    bool SetBitMessage(const std::string &);
    bool SetBitMessage(int);
    void HandleBitArray(diagnostic_msgs::DiagnosticArray &);

private:
    std::mutex m_bitArray_mutex;
    int m_TocksToOK;
    diagnostic_msgs::DiagnosticArray m_BitArray;
    void resetBitMessage();
};