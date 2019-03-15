#include "common.h"
#include "ShaderProgram.h"
#include "LiteMath.h"

#define GLFW_DLL
#include <GLFW/glfw3.h>

#include <vector>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

static GLsizei WIDTH = 1280, HEIGHT = 720;

using namespace LiteMath;

float3 g_camStartPos(0, 5, 20);
float3 g_camPos = g_camStartPos;
float  cam_rot[2] = {0,0};
int    mx = 0, my = 0;

GLint mode = 0;
bool stop = false;

bool keys[1024];
bool g_captureMouse = true;
GLfloat fieldOfView = 90.0;

void windowResize(GLFWwindow* window, int width, int height) {
	
	WIDTH  = width;
	HEIGHT = height;
	
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode) {
	
	//std::cout << key << std::endl;
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);
	if (action == GLFW_PRESS)
		keys[key] = true;
	else if (action == GLFW_RELEASE)
		keys[key] = false;
	
}

void checkKey() {
	
	float d = 0.1f;
	if (keys[GLFW_KEY_W])
		g_camPos += float3(cos(cam_rot[0])*sin(cam_rot[1]), sin(cam_rot[0]), -cos(cam_rot[0])*cos(cam_rot[1])) *d;
	if (keys[GLFW_KEY_A])
		g_camPos -= float3(sin(cam_rot[1]+PI/2.0), 0, -cos(cam_rot[1]+PI/2.0)) *d;
	if (keys[GLFW_KEY_S])
		g_camPos -= float3(cos(cam_rot[0])*sin(cam_rot[1]), sin(cam_rot[0]), -cos(cam_rot[0])*cos(cam_rot[1])) *d;
	if (keys[GLFW_KEY_D])
		g_camPos += float3(sin(cam_rot[1]+PI/2.0), 0, -cos(cam_rot[1]+PI/2.0)) *d;
	if (keys[GLFW_KEY_0]) {
		g_camPos = g_camStartPos;
		mode = 0;
		stop = false;
	}
	if (keys[GLFW_KEY_1])
		switch (mode) {
			case 0: mode = 1; break;
			case 1: mode = 0; break;
			case 2: mode = 3; break;
			case 3: mode = 2; break;
		}
	if (keys[GLFW_KEY_2])
		switch (mode) {
			case 0: mode = 2; break;
			case 1: mode = 3; break;
			case 2: mode = 0; break;
			case 3: mode = 1; break;
		}
	if (keys[GLFW_KEY_3])
		stop = !stop;
	
}

void mouseKey(GLFWwindow* window, int key, int action, int mods) {
	
	if (key == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE)
		g_captureMouse = !g_captureMouse;
	
	if (g_captureMouse)
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
	else
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
	
}

void mouseScroll(GLFWwindow* window, double xoffset, double yoffset) {
	
	fieldOfView -= yoffset;
	if (fieldOfView < 30) fieldOfView = 30.0;
	if (fieldOfView > 150) fieldOfView = 150.0;
	
}

void mouseMove(GLFWwindow* window, double xpos, double ypos) {
  
  xpos *= 0.25f;
  ypos *= 0.25f;

  int x1 = int(xpos);
  int y1 = int(ypos);
  
  if (g_captureMouse) {
	cam_rot[0] -= 0.01f*(y1 - my);
	if (cam_rot[0] < -PI/2.0f) cam_rot[0] = -PI/2.0f;
	if (cam_rot[0] > PI/2.0f) cam_rot[0] = PI/2.0f;
	cam_rot[1] += 0.01f*(x1 - mx);
  }

  mx = int(xpos);
  my = int(ypos);

}

int initGL() {
	
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize OpenGL context" << std::endl;
		return -1;
	}

	std::cout << "Vendor: "   << glGetString(GL_VENDOR) << std::endl;
	std::cout << "Renderer: " << glGetString(GL_RENDERER) << std::endl;
	std::cout << "Version: "  << glGetString(GL_VERSION) << std::endl;
	std::cout << "GLSL: "     << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;

	return 0;
	
}

unsigned int loadCubemap(std::vector<std::string> faces) {
	
    unsigned int textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);

    int width, height, nrChannels;
    for (unsigned int i = 0; i < faces.size(); i++)
    {
        unsigned char *data = stbi_load(faces[i].c_str(), &width, &height, &nrChannels, 0);
        if (data)
        {
            glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 
                         0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data
            );
            stbi_image_free(data);
        }
        else
        {
            std::cout << "Cubemap texture failed to load at path: " << faces[i] << std::endl;
            stbi_image_free(data);
        }
    }
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    return textureID;
	
}

int main() {
	
	if(!glfwInit())
		return -1;
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);
	
	GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "OpenGL ray marching", nullptr, nullptr);    
	if (window == nullptr) {
		
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
		
	}
	
	glfwMakeContextCurrent(window);
	glfwSetWindowSizeCallback(window, windowResize);
	glfwSetKeyCallback(window, key_callback);
	glfwSetCursorPosCallback(window, mouseMove);
	glfwSetMouseButtonCallback(window, mouseKey);
	glfwSetScrollCallback(window, mouseScroll);
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
	
	if(initGL())
		return -1;
	GLenum gl_error = glGetError();
	while (gl_error != GL_NO_ERROR)
		gl_error = glGetError();
	
	std::unordered_map<GLenum, std::string> shaders;
	shaders[GL_VERTEX_SHADER]   = "vertex.glsl";
	shaders[GL_FRAGMENT_SHADER] = "fragment.glsl";
	ShaderProgram program(shaders); GL_CHECK_ERRORS;
	
	glfwSwapInterval(1);
	
	GLfloat vertices[] = {
		-1.0f,  1.0f,	// v0 - top left corner
		-1.0f, -1.0f,	// v1 - bottom left corner
		1.0f,  1.0f,	// v2 - top right corner
		1.0f, -1.0f	  // v3 - bottom right corner
	};
	
    GLuint VBO, VAO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
	
    glBindVertexArray(VAO);

		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, 4 * 2 * sizeof(GLfloat), vertices, GL_STATIC_DRAW);

		glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), (GLvoid*)0);
		glEnableVertexAttribArray(0);

    glBindVertexArray(0);

//	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	std::vector<std::string> faces {
		"right.jpg",
		"left.jpg",
		"top.jpg",
		"bottom.jpg",
		"front.jpg",
		"back.jpg"
	};
	unsigned int cubemapTexture = loadCubemap(faces);
	
	GLfloat timeValue = glfwGetTime();
	
	while (!glfwWindowShouldClose(window))
	{
		glfwPollEvents();
		checkKey();
		
		glViewport(0, 0, WIDTH, HEIGHT);
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		
		program.StartUseShader();
		
		float4x4 rayMatrix   = mul(rotate_Y_4x4(-cam_rot[1]), rotate_X_4x4(+cam_rot[0]));
		
		program.SetUniform("g_rayMatrix", rayMatrix);
		program.SetUniform("g_camPos", g_camPos);
		
		program.SetUniform("g_screenWidth" , WIDTH);
		program.SetUniform("g_screenHeight", HEIGHT);
		program.SetUniform("fieldOfView", fieldOfView);
		program.SetUniform("mode", mode);
		
		if (!stop) timeValue = glfwGetTime();
		program.SetUniform("iTime", timeValue);
		
		
		glBindVertexArray(VAO);
		glBindTexture(GL_TEXTURE_CUBE_MAP, cubemapTexture);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
		glBindVertexArray(0);
			
		program.StopUseShader();
		
		glfwSwapBuffers(window);
	}
	
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
	
	glfwTerminate();
	return 0;
}
