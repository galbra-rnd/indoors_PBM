#pragma once
#include <unordered_map>
#include <functional>
#include "ControlProviderMediator/IControlProviderMediator.hpp"
#include "utils/ActionUtils.hpp"
#include <iostream>

class ControlProviderMediator : public IControlProviderMediator
{
public:
    /**
     * @brief Land callback function.
     * 
     */
    void Land() override;
    /**
     * @brief GoHome callback function.
     * 
     */
    void GoHome() override;

    std::shared_ptr<spdlog::logger> GetLogger() override { return m_logger; };
    ControlProviderMediator(std::unordered_map<ControlAction::Commands, std::function<void()>> &control_action_dict, std::shared_ptr<spdlog::logger> logger);
    ~ControlProviderMediator();

private:
    std::unordered_map<ControlAction::Commands, std::function<void()>> m_control_action_dict;
    std::shared_ptr<spdlog::logger> m_logger;
};