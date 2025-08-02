#pragma once

#include <exception>
#include <string>

/** @defgroup util Utilities Module
 *  Classes that provide general utilities for the rest of the codebase.
 */

/**
 * @struct IrrecoverableError
 * @brief A custom exception class for errors from which the application
 * cannot recover.
 * <blockquote>Exceptions, for the exceptional.</blockquote>
 * @ingroup util
 *
 * @details This exception is intended to be thrown when the program reaches a
 * state so corrupted or unexpected that continuing execution is impossible or
 * would lead to undefined behaviour. Examples include critical asset loading
 * failures, corrupted data structures, or violations of fundamental
 * invariants.
 *
 * @section Architecture
 * The constructor for this class is coupled with the `Logger` singleton. Upon
 * creation, an `IrrecoverableError` immediately logs its own error message.
 * This ensures that any fatal error is recorded before the stack unwinds and
 * the program potentially terminates, which is a robust strategy for
 * post-mortem debugging.
 */
struct IrrecoverableError : public std::exception
{
    /**
     * @brief The error message describing the fatal condition.
     */
    std::string msg;

    /**
     * @brief Constructs the exception object and logs the error message.
     * @param msg A string detailing the reason for the irrecoverable error.
     */
    explicit IrrecoverableError(std::string msg);
};
