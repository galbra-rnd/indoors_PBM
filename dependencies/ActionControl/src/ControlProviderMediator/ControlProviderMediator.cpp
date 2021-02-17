#include "ControlProviderMediator.hpp"

void ControlProviderMediator::GoHome()
{
    try
    {
        m_control_action_dict[ControlAction::Commands::GoHome]();
        m_logger->info("Calling GoHome callback");
    }
    catch (const std::exception &e)
    {
        m_logger->error(std::string(__PRETTY_FUNCTION__) + " Could not call callback: " + e.what());
    }
}

void ControlProviderMediator::Land()
{
    try
    {
        m_control_action_dict[ControlAction::Commands::Land]();
        m_logger->info("Calling Land callback");
    }
    catch (const std::exception &e)
    {
        m_logger->error(std::string(__PRETTY_FUNCTION__) + " Could not call callback: " + e.what());
    }
}

ControlProviderMediator::ControlProviderMediator(std::unordered_map<ControlAction::Commands, std::function<void()>> &control_action_dict, std::shared_ptr<spdlog::logger> logger)
    : m_control_action_dict(control_action_dict),
      m_logger(logger)
{
    m_logger->info(m_logger->name() + " initiated");
}

ControlProviderMediator::~ControlProviderMediator()
{
}