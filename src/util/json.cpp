#include "util/json.hpp"
#include "util/error.hpp"

namespace json
{

namespace detail
{

enum class TokenType : uint8_t {
    LeftBrace,
    RightBrace,
    LeftBracket,
    RightBracket,
    Comma,
    Colon,
    String,
    Number,
    True,
    False,
    Null,
    EndOfFile,
    Unknown
};

struct Token
{
    TokenType type;
    std::string_view lexeme;
    size_t line;
    size_t col;
};

class Lexer
{
    std::string_view source;
    const char* start;
    const char* current;
    size_t lineNum = 1;
    size_t colNum = 1;
    size_t lineStart = 1;
    size_t colStart = 1;

    [[nodiscard]] bool isAtEnd() const;
    char advance();
    [[nodiscard]] char peek() const;
    [[nodiscard]] char peekNext() const;
    void skipWhitespaceAndComments();
    [[nodiscard]] Token makeToken(TokenType type) const;
    Token stringToken();
    Token numberToken();
    Token identifierToken();

  public:
    Lexer(std::string_view source);
    Token nextToken();
};
} // namespace detail

class Parser
{
    detail::Lexer lexer;
    detail::Token currentToken;
    detail::Token previousToken;

    Parser(std::string_view source);

    void advance();
    void consume(detail::TokenType type, const std::string& message);
    JsonValue parseValue();
    JsonValue parseString();
    JsonValue parseNumber();
    JsonObject parseObject();
    JsonArray parseArray();

  public:
    static JsonValue parse(std::string_view source);
};

JsonValue parse(std::string_view source)
{
    return Parser::parse(source);
}

ParsingError::ParsingError(const std::string& message, size_t line, size_t col)
  : std::runtime_error(message + " (at line " + std::to_string(line) +
                       ", col " + std::to_string(col) + ")"),
    IrrecoverableError(message + " (at line " + std::to_string(line) +
                       ", col " + std::to_string(col) + ")"),
    lineNum(line), colNum(col)
{
}
size_t ParsingError::line() const
{
    return lineNum;
}
size_t ParsingError::col() const
{
    return colNum;
}
JsonValue::JsonValue(std::nullptr_t) : value(nullptr)
{
}
JsonValue::JsonValue(bool b) : value(b)
{
}
JsonValue::JsonValue(double d) : value(d)
{
}
JsonValue::JsonValue(int i) : value(static_cast<double>(i))
{
}
JsonValue::JsonValue(const std::string& s) : value(s)
{
}
JsonValue::JsonValue(std::string&& s) : value(std::move(s))
{
}
JsonValue::JsonValue(const char* s) : value(std::string(s))
{
}
JsonValue::JsonValue(const JsonArray& a) : value(a)
{
}
JsonValue::JsonValue(JsonArray&& a) : value(std::move(a))
{
}
JsonValue::JsonValue(const JsonObject& o) : value(o)
{
}
JsonValue::JsonValue(JsonObject&& o) : value(std::move(o))
{
}
bool JsonValue::isNull() const
{
    return std::holds_alternative<std::nullptr_t>(value);
}
bool JsonValue::isBool() const
{
    return std::holds_alternative<bool>(value);
}
bool JsonValue::isNumber() const
{
    return std::holds_alternative<double>(value);
}
bool JsonValue::isString() const
{
    return std::holds_alternative<std::string>(value);
}
bool JsonValue::isArray() const
{
    return std::holds_alternative<JsonArray>(value);
}
bool JsonValue::isObject() const
{
    return std::holds_alternative<JsonObject>(value);
}
bool& JsonValue::asBool()
{
    return std::get<bool>(value);
}
double& JsonValue::asNumber()
{
    return std::get<double>(value);
}
std::string& JsonValue::asString()
{
    return std::get<std::string>(value);
}
JsonArray& JsonValue::asArray()
{
    return std::get<JsonArray>(value);
}
JsonObject& JsonValue::asObject()
{
    return std::get<JsonObject>(value);
}
const bool& JsonValue::asBool() const
{
    return std::get<bool>(value);
}
const double& JsonValue::asNumber() const
{
    return std::get<double>(value);
}
const std::string& JsonValue::asString() const
{
    return std::get<std::string>(value);
}
const JsonArray& JsonValue::asArray() const
{
    return std::get<JsonArray>(value);
}
const JsonObject& JsonValue::asObject() const
{
    return std::get<JsonObject>(value);
}
bool detail::Lexer::isAtEnd() const
{
    return current >= source.data() + source.length();
}
char detail::Lexer::advance()
{
    if (isAtEnd()) {
        return '\0';
    }
    current++;
    colNum++;
    return current[-1];
}
char detail::Lexer::peek() const
{
    if (isAtEnd()) {
        return '\0';
    }
    return *current;
}
char detail::Lexer::peekNext() const
{
    if (current + 1 >= source.data() + source.length()) {
        return '\0';
    }
    return current[1];
}
void detail::Lexer::skipWhitespaceAndComments()
{
    while (true) {
        char c = peek();
        switch (c) {
            case ' ':
            case '\r':
            case '\t': advance(); break;
            case '\n':
                advance();
                lineNum++;
                colNum = 1;
                break;
            case '/':
                if (peekNext() == '/') { // Single-line comment
                    while (peek() != '\n' && !isAtEnd()) {
                        advance();
                    }
                } else if (peekNext() == '*') { // Multi-line comment
                    advance();                  // Consume '/'
                    advance();                  // Consume '*'
                    while ((peek() != '*' || peekNext() != '/') && !isAtEnd()) {
                        if (peek() == '\n') {
                            lineNum++;
                            colNum = 1;
                        }
                        advance();
                    }
                    if (!isAtEnd()) {
                        advance(); // Consume '*'
                    }
                    if (!isAtEnd()) {
                        advance(); // Consume '/'
                    }
                } else {
                    return; // Not a comment
                }
                break;
            default: return;
        }
    }
}
detail::Token detail::Lexer::makeToken(TokenType type) const
{
    return {.type = type,
            .lexeme = std::string_view(start, current - start),
            .line = lineStart,
            .col = colStart};
}
detail::Token detail::Lexer::stringToken()
{
    while (peek() != '"' && !isAtEnd()) {
        if (peek() == '\n') {
            lineNum++;
            colNum = 1;
        }
        if (peek() == '\\') {
            // Handle escaped characters simply by advancing lmao
            advance();
        }
        advance();
    }

    if (isAtEnd()) {
        return {.type = TokenType::Unknown,
                .lexeme = "Unterminated string.",
                .line = lineStart,
                .col = colStart};
    }

    advance(); // Consume the closing quote
    return makeToken(TokenType::String);
}
detail::Token detail::Lexer::numberToken()
{
    while (std::isdigit(peek()) != 0) {
        advance();
    }
    if (peek() == '.' && (std::isdigit(peekNext()) != 0)) {
        advance(); // Consume '.'
        while (std::isdigit(peek()) != 0) {
            advance();
        }
    }
    if (peek() == 'e' || peek() == 'E') {
        advance();
        if (peek() == '+' || peek() == '-') {
            advance();
        }
        while (std::isdigit(peek()) != 0) {
            advance();
        }
    }
    return makeToken(TokenType::Number);
}
detail::Token detail::Lexer::identifierToken()
{
    while (std::isalpha(peek()) != 0) {
        advance();
    }

    std::string_view text(start, current - start);
    if (text == "true") {
        return makeToken(TokenType::True);
    }
    if (text == "false") {
        return makeToken(TokenType::False);
    }
    if (text == "null") {
        return makeToken(TokenType::Null);
    }

    return makeToken(TokenType::Unknown);
}
detail::Lexer::Lexer(std::string_view source)
  : source(source), start(source.data()), current(source.data())
{
}
detail::Token detail::Lexer::nextToken()
{
    skipWhitespaceAndComments();
    start = current;
    lineStart = lineNum;
    colStart = colNum;

    if (isAtEnd()) {
        return makeToken(TokenType::EndOfFile);
    }

    char c = advance();
    switch (c) {
        case '{': return makeToken(TokenType::LeftBrace);
        case '}': return makeToken(TokenType::RightBrace);
        case '[': return makeToken(TokenType::LeftBracket);
        case ']': return makeToken(TokenType::RightBracket);
        case ',': return makeToken(TokenType::Comma);
        case ':': return makeToken(TokenType::Colon);
        case '"': return stringToken();
        default:
            if ((std::isdigit(c) != 0) || c == '-') {
                return numberToken();
            }
            if (std::isalpha(c) != 0) {
                return identifierToken();
            }
    }

    return makeToken(TokenType::Unknown);
}
Parser::Parser(std::string_view source) : lexer(source)
{
    // Prime the pump :)
    advance();
}
void Parser::advance()
{
    previousToken = currentToken;
    currentToken = lexer.nextToken();
    if (currentToken.type == detail::TokenType::Unknown) {
        throw ParsingError("Unexpected character or unterminated literal",
                           currentToken.line, currentToken.col);
    }
}
void Parser::consume(detail::TokenType type, const std::string& message)
{
    if (currentToken.type == type) {
        advance();
        return;
    }
    throw ParsingError(message, currentToken.line, currentToken.col);
}
JsonValue Parser::parseValue()
{
    switch (currentToken.type) {
        case detail::TokenType::LeftBrace: return parseObject();
        case detail::TokenType::LeftBracket: return parseArray();
        case detail::TokenType::String: return parseString();
        case detail::TokenType::Number: return parseNumber();
        case detail::TokenType::True: advance(); return {true};
        case detail::TokenType::False: advance(); return {false};
        case detail::TokenType::Null: advance(); return {nullptr};
        default:
            throw ParsingError("Expected a value (object, array, string, "
                               "number, true, false, or null).",
                               currentToken.line, currentToken.col);
    }
}
JsonValue Parser::parseString()
{
    // The lexeme includes the quotes, so we create a substring without
    // them. We also need to unescape the characters
    std::string result;
    auto view = currentToken.lexeme;
    result.reserve(view.length() - 2);
    for (size_t i = 1; i < view.length() - 1; ++i) {
        if (view[i] == '\\' && i + 1 < view.length() - 1) {
            i++;
            switch (view[i]) {
                case '"': result += '"'; break;
                case '\\': result += '\\'; break;
                case '/': result += '/'; break;
                case 'b': result += '\b'; break;
                case 'f': result += '\f'; break;
                case 'n': result += '\n'; break;
                case 'r': result += '\r'; break;
                case 't': result += '\t'; break;
                // uhhhh unicode escapes are complex, so we'll skip a full
                // implementation. if you're putting unicode escape
                // sequences in the JSON file...why...just why
                default: result += view[i]; // just add the character as is
            }
        } else {
            result += view[i];
        }
    }
    advance();
    return {std::move(result)};
}
JsonValue Parser::parseNumber()
{
    std::string numStr(currentToken.lexeme);
    try {
        size_t pos = 0;
        double value = std::stod(numStr, &pos);

        if (pos != numStr.length()) {
            throw ParsingError("Invalid characters in number literal.",
                               currentToken.line, currentToken.col);
        }

        advance();
        return {value};
    } catch (const std::invalid_argument&) {
        throw ParsingError("Invalid number format.", currentToken.line,
                           currentToken.col);
    } catch (const std::out_of_range&) {
        throw ParsingError("Number is out of range for a double.",
                           currentToken.line, currentToken.col);
    }
}
JsonObject Parser::parseObject()
{
    consume(detail::TokenType::LeftBrace, "Expected '{' to start an object.");
    JsonObject object;

    if (currentToken.type != detail::TokenType::RightBrace) {
        while (true) {
            if (currentToken.type != detail::TokenType::String) {
                throw ParsingError("Expected a string key for object member.",
                                   currentToken.line, currentToken.col);
            }
            std::string key = std::string(
              currentToken.lexeme.substr(1, currentToken.lexeme.length() - 2));
            advance();

            consume(detail::TokenType::Colon, "Expected ':' after object key.");

            object[std::move(key)] = parseValue();

            if (currentToken.type == detail::TokenType::RightBrace)
                break;
            consume(detail::TokenType::Comma,
                    "Expected ',' or '}' after object member.");
        }
    }

    consume(detail::TokenType::RightBrace, "Expected '}' to end an object.");
    return object;
}
JsonArray Parser::parseArray()
{
    consume(detail::TokenType::LeftBracket, "Expected '[' to start an array.");
    JsonArray array;

    if (currentToken.type != detail::TokenType::RightBracket) {
        while (true) {
            array.push_back(parseValue());
            if (currentToken.type == detail::TokenType::RightBracket)
                break;
            consume(detail::TokenType::Comma,
                    "Expected ',' or ']' after array element.");
        }
    }

    consume(detail::TokenType::RightBracket, "Expected ']' to end an array.");
    return array;
}
JsonValue Parser::parse(std::string_view source)
{
    // handle a UTF-8 Byte Order Mark (BOM) if present (WHY WINDOWS WHY)
    if (source.size() >= 3 && static_cast<unsigned char>(source[0]) == 0xEF &&
        static_cast<unsigned char>(source[1]) == 0xBB &&
        static_cast<unsigned char>(source[2]) == 0xBF)
    {
        source.remove_prefix(3);
    }

    Parser parser(source);
    return parser.parseValue();
}
void serialise(const JsonValue& val, std::ostream& os, int indent)
{
    std::visit(
      [&](auto&& arg) {
          using T = std::decay_t<decltype(arg)>;
          if constexpr (std::is_same_v<T, std::nullptr_t>) {
              os << "null";
          } else if constexpr (std::is_same_v<T, bool>) {
              os << (arg ? "true" : "false");
          } else if constexpr (std::is_same_v<T, double>) {
              os << arg;
          } else if constexpr (std::is_same_v<T, std::string>) {
              os << '"' << arg << '"'; // Note: This is a simplified stringify,
                                       // doesn't escape characters.
          } else if constexpr (std::is_same_v<T, JsonArray>) {
              os << "[\n";
              for (size_t i = 0; i < arg.size(); ++i) {
                  os << std::string(indent + 2, ' ');
                  serialise(arg[i], os, indent + 2);
                  if (i < arg.size() - 1) {
                      os << ",";
                  }
                  os << "\n";
              }
              os << std::string(indent, ' ') << "]";
          } else if constexpr (std::is_same_v<T, JsonObject>) {
              os << "{\n";
              size_t i = 0;
              for (const auto& [key, value] : arg) {
                  os << std::string(indent + 2, ' ');
                  os << '"' << key << "\": ";
                  serialise(value, os, indent + 2);
                  if (i < arg.size() - 1) {
                      os << ",";
                  }
                  os << "\n";
                  i++;
              }
              os << std::string(indent, ' ') << "}";
          }
      },
      val.value);
}
std::ostream& operator<<(std::ostream& os, const JsonValue& val)
{
    serialise(val, os);
    return os;
}
} // namespace json
