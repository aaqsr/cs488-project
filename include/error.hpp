#pragma once

#include <exception>
#include <iostream>
#include <string>

struct IrrecoverableError : public std::exception
{
    std::string msg;

    IrrecoverableError(std::string msg) : msg{std::move(msg)}
    {
      std::cout << this->msg;
    }
};
