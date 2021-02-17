#include "LandActionNode.hpp"

LandActionNode::LandActionNode(const std::string &name, std::shared_ptr<IControlProviderMediator> ControlProvider)
    : BT::SyncActionNode(name, {}),
      m_ControlProvider(ControlProvider)
{
}

// You must override the virtual function tick()
BT::NodeStatus LandActionNode::tick()
{
    m_ControlProvider->Land();
    return BT::NodeStatus::SUCCESS;
}
