#pragma once

#include <exception>
#include <string>

struct IrrecoverableError : public std::exception
{
    std::string msg;
    IrrecoverableError(std::string msg);
};
