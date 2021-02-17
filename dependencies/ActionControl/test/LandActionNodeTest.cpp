#include "gtest/gtest.h"
#include <memory>
#include <unordered_map>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "ControlProviderMediator/ControlProviderMediator.hpp"
#include "TreeNodes/LandActionNode.hpp"

static const char *xml_text = R"(

  <root main_tree_to_execute = "MainTreeBatteryTest" >

      <BehaviorTree ID="MainTreeBatteryTest">
          <ReactiveSequence name="root_sequence">
              <LandActionNode   name="This it the Land action Node"/>
          </ReactiveSequence>
      </BehaviorTree>

  </root>
  )";

TEST(LandActionNodeTest, NodeCreation)
{

  // Setup
  std::unordered_map<ControlAction::Commands, std::function<void()>> control_action_dict;

  auto simple_func = []() { std::cout << "\ntemp\n"; };
  control_action_dict[ControlAction::Commands::Land] = simple_func;

  std::shared_ptr<IControlProviderMediator> control_provider_mediator = std::make_shared<ControlProviderMediator>(control_action_dict,spdlog::stdout_color_mt("NodeCreation"));

  // We use the BehaviorTreeFactory to register our custom nodes
  BT::BehaviorTreeFactory factory;
  // Run
  LandActionNode landAction_node("LandActionNode", control_provider_mediator);
  factory.registerSimpleAction("LandActionNode", std::bind(&LandActionNode::tick, &landAction_node));

  // Trees are created at deployment-time (i.e. at run-time, but only once at the beginning).
  // The currently supported format is XML.
  // IMPORTANT: when the object "tree" goes out of scope, all the TreeNodes are destroyed
  auto tree = factory.createTreeFromText(xml_text);

  // To "execute" a Tree you need to "tick" it.
  // The tick is propagated to the children based on the logic of the tree.
  // In this case, the entire sequence is executed, because all the children
  // of the Sequence return SUCCESS.
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
