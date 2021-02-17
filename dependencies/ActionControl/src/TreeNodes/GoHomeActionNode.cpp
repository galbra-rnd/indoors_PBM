#include "GoHomeActionNode.hpp"

GoHomeActionNode::GoHomeActionNode(const std::string &name, std::shared_ptr<IControlProviderMediator> ControlProvider)
    : BT::SyncActionNode(name, {}),
      m_ControlProvider(ControlProvider)
{
}

// You must override the virtual function tick()
BT::NodeStatus GoHomeActionNode::tick()
{
    m_ControlProvider->GoHome();
    return BT::NodeStatus::SUCCESS;
}
