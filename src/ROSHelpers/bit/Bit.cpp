#include "Bit.hpp"

Bit::~Bit()
{
}

Bit::Bit()
{
    m_BitArray.status.push_back(diagnostic_msgs::DiagnosticStatus());
}

bool Bit::SetBitMessage(int statusIdx)
{
    const std::lock_guard<std::mutex> lock(m_bitArray_mutex);

    auto message = pbm_statuses::IdStatusMap[statusIdx];
    if (message.second == "NO SUCH MESSAGE IN POINT_AND_GO::IdStatusMap")
    {
        return false;
    }

    m_BitArray.header.stamp = ros::Time::now();
    m_BitArray.status[0].hardware_id = std::to_string(statusIdx);
    m_BitArray.status[0].level = message.first;
    m_BitArray.status[0].message = message.second;
    return true;
}

bool Bit::SetBitMessage(const std::string &msg)
{
    const std::lock_guard<std::mutex> lock(m_bitArray_mutex);
    m_BitArray.header.stamp = ros::Time::now();
    m_BitArray.status[0].message = msg;
}

bool Bit::SetBitLevel(int level)
{
    const std::lock_guard<std::mutex> lock(m_bitArray_mutex);
    assert(level >= 0 && level <= 2 && "Level should be between 0 to 3");
    m_BitArray.status[0].level = level;
}

void Bit::HandleBitArray(diagnostic_msgs::DiagnosticArray &outMsg)
{
    const std::lock_guard<std::mutex> lock(m_bitArray_mutex);
    if ((m_TocksToOK--) <= 0)
    {
        resetBitMessage();
    }
    outMsg = m_BitArray;
}

void Bit::resetBitMessage()
{
    m_TocksToOK = TICKS_TO_OK;
    m_BitArray.status[0].level = diagnostic_msgs::DiagnosticStatus::OK;
    m_BitArray.status[0].message = "Everything is OK";
}