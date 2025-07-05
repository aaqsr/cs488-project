#include "util/error.hpp"
#include "util/logger.hpp"

IrrecoverableError::IrrecoverableError(std::string msg) : msg{std::move(msg)}
{
    Logger::GetInstance().log(std::string{">>> ERROR: "} + this->msg);
}
