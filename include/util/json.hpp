#pragma once

#include "util/error.hpp"
#include <charconv>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace json
{

class JsonValue;

using JsonObject = std::map<std::string, JsonValue>;
using JsonArray = std::vector<JsonValue>;

class ParsingError : public std::runtime_error, public IrrecoverableError
{
    size_t lineNum;
    size_t colNum;

   public:
    ParsingError(const std::string& message, size_t line, size_t col);

    [[nodiscard]] size_t line() const;
    [[nodiscard]] size_t col() const;
};

// variant-based class to hold any valid JSON type.
class JsonValue
{
    // underlying variant that holds one of the possible JSON types.
    std::variant<std::nullptr_t, bool, double, std::string, JsonArray,
                 JsonObject>
      value;

   public:
    // Constructors for each JSON type
    JsonValue(std::nullptr_t = nullptr);
    JsonValue(bool b);
    JsonValue(double d);
    JsonValue(int i);
    JsonValue(const std::string& s);
    JsonValue(std::string&& s);
    JsonValue(const char* s);
    JsonValue(const JsonArray& a);
    JsonValue(JsonArray&& a);
    JsonValue(const JsonObject& o);
    JsonValue(JsonObject&& o);

    // Helper functions to check the contained type
    [[nodiscard]] bool isNull() const;
    [[nodiscard]] bool isBool() const;
    [[nodiscard]] bool isNumber() const;
    [[nodiscard]] bool isString() const;
    [[nodiscard]] bool isArray() const;
    [[nodiscard]] bool isObject() const;

    // type-safe accessors. !!throws std::bad_variant_access on type mismatch!!
    bool& asBool();
    double& asNumber();
    std::string& asString();
    JsonArray& asArray();
    JsonObject& asObject();

    [[nodiscard]] const bool& asBool() const;
    [[nodiscard]] const double& asNumber() const;
    [[nodiscard]] const std::string& asString() const;
    [[nodiscard]] const JsonArray& asArray() const;
    [[nodiscard]] const JsonObject& asObject() const;

    friend void serialise(const JsonValue& val, std::ostream& os, int indent);
};

[[nodiscard]] JsonValue parse(std::string_view source);

void serialise(const JsonValue& val, std::ostream& os, int indent = 0);

std::ostream& operator<<(std::ostream& os, const JsonValue& val);

} // namespace json
