#pragma once
#include "behaviortree_cpp_v3/action_node.h"
#include "ControlProviderMediator/IControlProviderMediator.hpp"

/**
 * @brief Simple action Node.
 * Activates Land command supplied by the IControlProviderMediator
 * 
 * 
 */
class LandActionNode : public BT::SyncActionNode
{
public:
    LandActionNode(const std::string &name, std::shared_ptr<IControlProviderMediator> ControlProvider);

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<IControlProviderMediator> m_ControlProvider;
};
