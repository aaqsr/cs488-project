#include "error.hpp"
#include "logger.hpp"

IrrecoverableError::IrrecoverableError(std::string msg) : msg{std::move(msg)}
{
    Logger::GetInstance().log(std::string{">>> ERROR: "} + this->msg);
}
