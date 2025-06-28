#include "singleton.hpp"
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "linalg.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

using namespace linalg::aliases;

class Window : public Singleton<Window>
{
    static constexpr int width = 800;
    static constexpr int height = 600;
    inline static const char* title = "CS488";

    GLFWwindow* window;

  public:
    Window()
    {
        if (glfwInit() == 0) {
            throw std::runtime_error{"Failed to initialize GLFW."};
        }
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // for apple

        window = glfwCreateWindow(width, height, title, nullptr, nullptr);
        if (window == nullptr) {
            glfwTerminate();
            throw std::runtime_error{"Failed to create GLFW window"};
        }

        glfwMakeContextCurrent(window);

        // TODO: what is glew for??
        glewExperimental = 1;
        if (glewInit() != GLEW_OK) {
            glfwTerminate();
            throw std::runtime_error{"Failed to initialize GLEW."};
        }

        // Enable VSync
        // glfwSwapInterval(1);

        // Callback when window resizes
        // glfwSetFramebufferSizeCallback(
        //   window, [](GLFWwindow* window, int width, int height) {
        //   });

        // Use framebuffer size, not window size for viewport.
        // Why? bcs they might differ...sigh (example: on HiDPI or Retina
        // displays, framebuffer is typically 2x the window size)
        int framebufferWidth = width;
        int framebufferHeight = height;
        glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);
        glViewport(0, 0, framebufferWidth, framebufferHeight);
    }

    Window(const Window&) = delete;
    Window(Window&&) = delete;
    Window& operator=(const Window&) = delete;
    Window& operator=(Window&&) = delete;

    ~Window()
    {
        if (window != nullptr) {
            glfwDestroyWindow(window);
        }
        glfwTerminate();
    }

    GLFWwindow* getWindow()
    {
        return window;
    }

    [[nodiscard]] bool shouldClose() const
    {
        return glfwWindowShouldClose(window) == GL_TRUE;
    }

    void swapBuffers()
    {
        glfwSwapBuffers(window);
    }
};

class Shader
{
  public:
    Shader() = default;

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;
    Shader(Shader&&) = delete;
    Shader& operator=(Shader&&) = delete;

    ~Shader()
    {
        if (programId != 0) {
            glDeleteProgram(programId);
        }
    }

    void loadFromSource(const std::string& vertexSource,
                        const std::string& fragmentSource)
    {
        GLuint vertexShader = compileShader(vertexSource, GL_VERTEX_SHADER);
        if (vertexShader == 0) {
            throw std::runtime_error{"Vertex shader failed to compile"};
        }

        GLuint fragmentShader =
          compileShader(fragmentSource, GL_FRAGMENT_SHADER);
        if (fragmentShader == 0) {
            glDeleteShader(vertexShader);
            throw std::runtime_error{"Fragment shader failed to compile"};
        }

        try {
            linkProgram(vertexShader, fragmentShader);
        } catch (...) {
            glDeleteShader(vertexShader);
            glDeleteShader(fragmentShader);
            throw;
        }

        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
    }

    void loadFromFile(const std::string& vertexPath,
                      const std::string& fragmentPath)
    {
        std::string vertexSource = readFile(vertexPath);
        std::string fragmentSource = readFile(fragmentPath);

        if (vertexSource.empty() || fragmentSource.empty()) {
            throw std::runtime_error{"Failed to read shader file"};
        }

        loadFromSource(vertexSource, fragmentSource);
    }

    void use() const
    {
        glUseProgram(programId);
    }

    [[nodiscard]] GLuint getId() const
    {
        return programId;
    }

  private:
    [[nodiscard]] static GLuint compileShader(const std::string& source,
                                              GLenum type)
    {
        GLuint shader = glCreateShader(type);
        const char* sourceCStr = source.c_str();

        glShaderSource(shader, 1, &sourceCStr, nullptr);
        glCompileShader(shader);

        GLint success = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);

        if (success == 0) {
            std::array<GLchar, 1024> infoLog{};
            glGetShaderInfoLog(shader, sizeof(infoLog), nullptr,
                               infoLog.begin());

            const char* shaderType =
              (type == GL_VERTEX_SHADER) ? "VERTEX" : "FRAGMENT";

            glDeleteShader(shader);

            throw std::runtime_error{std::string{"ERROR: "} + shaderType +
                                     " shader compilation failed:\n" +
                                     infoLog.begin()};
        }

        return shader;
    }

    void linkProgram(GLuint vertexShader, GLuint fragmentShader)
    {
        programId = glCreateProgram();

        glAttachShader(programId, vertexShader);
        glAttachShader(programId, fragmentShader);
        glLinkProgram(programId);

        GLint success = 0;
        glGetProgramiv(programId, GL_LINK_STATUS, &success);

        if (success == 0) {
            std::array<GLchar, 1024> infoLog{};
            glGetProgramInfoLog(programId, sizeof(infoLog), nullptr,
                                infoLog.begin());

            glDeleteProgram(programId);
            programId = 0;

            throw std::runtime_error{
              std::string{"ERROR: Shader program linking failed:\n"} +
              infoLog.begin()};
        }
    }

    [[nodiscard]] static std::string readFile(const std::string& filePath)
    {
        std::ifstream file(filePath);
        if (!file.is_open()) {
            throw std::runtime_error{"ERROR: Could not open file: " + filePath};
        }

        std::stringstream buffer;
        buffer << file.rdbuf();
        file.close();

        return buffer.str();
    }

    GLuint programId{};
};

// TODO: Do we want struct of arrays instead for CPU cache locality when doing
// computations on one attribute only?
struct Vertex
{
    // If this changes, go update Mesh::setupMesh()!
    float3 position;
    float4 colour;

    // TODO: can we somehow generate this boilerplate?
    static void setupVertexAttributes()
    {
        // Position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              (void*)offsetof(Vertex, position));
        glEnableVertexAttribArray(0);

        // Color attribute
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              (void*)offsetof(Vertex, colour));
        glEnableVertexAttribArray(1);
    }
};

class Mesh
{
  public:
    Mesh() = default;

    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;
    Mesh(Mesh&&) = delete;
    Mesh& operator=(Mesh&&) = delete;
    ~Mesh()
    {
        if (VAO != 0) {
            glDeleteVertexArrays(1, &VAO);
        }
        if (VBO != 0) {
            glDeleteBuffers(1, &VBO);
        }
        if (EBO != 0) {
            glDeleteBuffers(1, &EBO);
        }
    }

    void setVertices(const std::vector<Vertex>& v)
    {
        vertices = v;
    }

    void setIndices(const std::vector<unsigned int>& i)
    {
        indices = i;
    }

    void setupMesh()
    {
        // Generate and bind VAO
        glGenVertexArrays(1, &VAO);
        glBindVertexArray(VAO);

        // Generate and bind VBO
        glGenBuffers(1, &VBO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)),
                     vertices.data(), GL_STATIC_DRAW);

        // Generate and bind EBO
        glGenBuffers(1, &EBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(
          GL_ELEMENT_ARRAY_BUFFER,
          static_cast<GLsizeiptr>(indices.size() * sizeof(unsigned int)),
          indices.data(), GL_STATIC_DRAW);

        Vertex::setupVertexAttributes();

        // Unbind VAO
        glBindVertexArray(0);
    }
    void draw() const
    {
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices.size()),
                       GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);
    }

  private:
    GLuint VAO{0}, VBO{0}, EBO{0};
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
};

class App
{
  public:
    void poll()
    {
    }
};

int main()
{
    Window& window = Window::GetInstance();

    Shader rainbowShader;
    Shader flatColourShader;

    rainbowShader.loadFromFile("shaders/vertex/rainbowTriTest.glsl",
                               "shaders/fragment/rainbowTriTest.glsl");

    flatColourShader.loadFromFile("shaders/vertex/flatTriTest.glsl",
                                  "shaders/fragment/flatTriTest.glsl");

    std::vector<Vertex> vertices1 = {
      // Bottom-left
      {{-0.7F, -0.5F, 0.0F}, {1.0F, 0.0F, 0.0F, 1.0F}}, // Red
      // Bottom-right
      {{-0.1F, -0.5F, 0.0F}, {0.0F, 1.0F, 0.0F, 1.0F}}, // Green
      // Top-right
      { {-0.1F, 0.5F, 0.0F}, {0.0F, 0.0F, 1.0F, 1.0F}}, // Blue
      // Top-left
      { {-0.7F, 0.5F, 0.0F}, {1.0F, 1.0F, 0.0F, 1.0F}}  // Yellow
    };

    std::vector<Vertex> vertices2 = {
      // Bottom-left
      {{0.2F, -0.5F, 0.0F}, {1.0F, 0.0F, 0.0F, 1.0F}}, // Red
      // Bottom-right
      {{0.8F, -0.5F, 0.0F}, {0.0F, 1.0F, 0.0F, 1.0F}}, // Green
      // Top
      { {0.5F, 0.5F, 0.0F}, {0.0F, 0.0F, 1.0F, 1.0F}}, // Blue
    };

    // Indices for two triangles forming a square
    std::vector<unsigned int> indices1 = {
      0, 1, 2, // First triangle (bottom-left, bottom-right, top-right)
      2, 3, 0  // Second triangle (top-right, top-left, bottom-left)
    };

    // indices for second triangle
    std::vector<unsigned int> indices2 = {
      0, 1, 2, // First triangle (bottom-left, bottom-right, top-right)
    };

    Mesh mesh;
    Mesh mesh2;

    mesh.setVertices(vertices1);
    mesh.setIndices(indices1);
    mesh.setupMesh();

    mesh2.setVertices(vertices2);
    mesh2.setIndices(indices2);
    mesh2.setupMesh();

    while (!window.shouldClose()) {
        // not clearing the back buffer causes trails
        glClear(GL_COLOR_BUFFER_BIT);

        /* RENDER COMMANDS HERE */

        // Enable this for Wireframe mode
        // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        rainbowShader.use();
        mesh.draw();

        flatColourShader.use();
        mesh2.draw();

        // Double buffering.
        // The front buffer contains the final output image that is
        // shown at the screen. Whilst all the rendering commands
        // draw to the back buffer. We swap the back buffer to the
        // front buffer so the image can be displayed without still
        // being rendered to.
        window.swapBuffers();
        glfwPollEvents();
    }
}
