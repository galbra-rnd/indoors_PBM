#pragma once
#include "behaviortree_cpp_v3/action_node.h"
#include "ControlProviderMediator/IControlProviderMediator.hpp"

/**
 * @brief CSimple action Node.
 * Activates Land command supplied by the IControlProviderMediator
 * 
 */
class GoHomeActionNode : public BT::SyncActionNode
{
public:
    GoHomeActionNode(const std::string &name, std::shared_ptr<IControlProviderMediator> ControlProvider);

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

private:
    std::shared_ptr<IControlProviderMediator> m_ControlProvider;
};
