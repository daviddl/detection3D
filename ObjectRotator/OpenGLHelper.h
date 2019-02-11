//
// Created by dlalancette on 19/02/19.
//

#ifndef DETECTION3D_OPENGLHELPER_H
#define DETECTION3D_OPENGLHELPER_H

#include <GLFW/glfw3.h>

#include <string>
#include <fstream>
#include <sstream>
#include <streambuf>
#include <vector>
#include <algorithm>
namespace OpenGLHelper
{
    /*
     * Initialize the OpenGL context
     */
    bool init()
    {
        if( !glfwInit() )
        {
            fprintf( stderr, "Failed to initialize GLFW\n" );
            getchar();
            return false;
        }

        glfwWindowHint(GLFW_SAMPLES, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

        return true;
    }

    /*
     * Create the GLFW Window in which the object will be displayed and rotated.
     */
    GLFWwindow *createWindow()
    {
        GLFWwindow *window = glfwCreateWindow( 1024, 768, "Object Rotator", NULL, NULL);
        if(window == NULL){
            fprintf( stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n" ); //TODO Replace Error message
            getchar();
            glfwTerminate();
            return nullptr;
        }
    }

    /*
     * Read the shader code in the specified file into a string.
     */
    std::string getShaderCode(const char *path)
    {
        std::string shaderCode;
        std::ifstream fileStream(path, std::ios::in);

        if(fileStream.is_open())
        {
            std::stringstream stringStream;
            stringStream << fileStream.rdbuf();
            shaderCode = stringStream.str();
            fileStream.close();

//            shaderCode.erase(std::remove(shaderCode.begin(), shaderCode.end(), '\n'), shaderCode.end());
        }
        else
        {
            printf("Could not open %s.\n", path);
            getchar();
        }

        return shaderCode;
    }

    /*
     * Get OpenGL to compile a shader.
     */
    GLuint compileShader(std::string shaderCode, int shaderType)
    {
        GLint result = GL_FALSE;
        int logLength;

        GLuint shaderId = glCreateShader(shaderType);

        char const *sourcePointer = shaderCode.c_str();
        glShaderSource(shaderId, 1, &sourcePointer, nullptr);
        glCompileShader(shaderId);

        // Check Vertex Shader
        glGetShaderiv(shaderId, GL_COMPILE_STATUS, &result);
        glGetShaderiv(shaderId, GL_INFO_LOG_LENGTH, &logLength);
        if (logLength > 0)
        {
            std::vector<char> errorMessage(size_t((logLength + 1)));
            glGetShaderInfoLog(shaderId, logLength, nullptr, &errorMessage[0]);
            printf("%s\n", &errorMessage[0]);
        }

        return shaderId;
    }

    /*
     * Compile the vertex and fragment shaders at the specified locations.
     */
    GLuint loadShaders(const char *vertexShaderPath, const char *fragmentShaderPath)
    {
        std::string vertexShaderCode = getShaderCode(vertexShaderPath);
        std::string fragmentShaderCode = getShaderCode(fragmentShaderPath);

        printf("Compiling shader: %s\n", vertexShaderPath);
        GLuint vertexShaderId = compileShader(vertexShaderCode, GL_VERTEX_SHADER);

        printf("Compiling shader: %s\n", fragmentShaderPath);
        GLuint fragmentShaderId = compileShader(fragmentShaderCode, GL_FRAGMENT_SHADER);

        printf("Linking program\n");
        GLuint programID = glCreateProgram();
        glAttachShader(programID, vertexShaderId);
        glAttachShader(programID, fragmentShaderId);
        glLinkProgram(programID);

        GLint result = GL_FALSE;
        int logLength;

        glGetProgramiv(programID, GL_LINK_STATUS, &result);
        glGetProgramiv(programID, GL_INFO_LOG_LENGTH, &logLength);
        if (logLength > 0)
        {
            std::vector<char> errorMessage(size_t(logLength + 1));
            glGetProgramInfoLog(programID, logLength, nullptr, &errorMessage[0]);
            printf("%s\n", &errorMessage[0]);
        }

        glDetachShader(programID, vertexShaderId);
        glDetachShader(programID, fragmentShaderId);

        glDeleteShader(vertexShaderId);
        glDeleteShader(fragmentShaderId);

        return programID;
    }
}

#endif //DETECTION3D_OPENGLHELPER_H
