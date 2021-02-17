#pragma once
#include "utils/ActionUtils.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
class IControlProviderMediator
{
public:
    virtual void Land() = 0;
    virtual void GoHome() = 0;
    virtual std::shared_ptr<spdlog::logger> GetLogger() = 0;
    virtual ~IControlProviderMediator(){};
};
