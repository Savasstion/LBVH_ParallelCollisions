#include <iostream>
#include <Windows.h>
#include "Base Classes/Scene.h"
#include "Core/Graphics.h"
#include "Core/Physics.cuh"
#include "Utillities/FrameTimer.h"
#include <string>
#include <sstream>


#pragma comment (lib, "OpenGL32.lib")
#define WINDOW_TITLE "Parallel Collision Detection"

Scene currentScene = Scene();
unsigned long long pCount = 100;

LRESULT WINAPI WindowProcedure(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	switch (msg)
	{
	case WM_DESTROY:
		Physics::WriteAverageFrameTimeToFile("frame_time_log.txt");
		PostQuitMessage(0);
		break;

	case WM_KEYDOWN:
		switch (wParam) {
		case VK_ESCAPE: // Escape key
			Physics::WriteAverageFrameTimeToFile("frame_time_log.txt");
			PostQuitMessage(0);
			break;
		

		default: // For any other key presses
			break;
		}
		break;

	default:
		break;

	}

	return DefWindowProc(hWnd, msg, wParam, lParam);
}
//--------------------------------------------------------------------

bool initPixelFormat(HDC hdc)
{
	PIXELFORMATDESCRIPTOR pfd;
	ZeroMemory(&pfd, sizeof(PIXELFORMATDESCRIPTOR));

	pfd.cAlphaBits = 8;
	pfd.cColorBits = 32;
	pfd.cDepthBits = 24;
	pfd.cStencilBits = 0;

	pfd.dwFlags = PFD_DOUBLEBUFFER | PFD_SUPPORT_OPENGL | PFD_DRAW_TO_WINDOW;

	pfd.iLayerType = PFD_MAIN_PLANE;
	pfd.iPixelType = PFD_TYPE_RGBA;
	pfd.nSize = sizeof(PIXELFORMATDESCRIPTOR);
	pfd.nVersion = 1;

	// choose pixel format returns the number most similar pixel format available
	int n = ChoosePixelFormat(hdc, &pfd);

	// set pixel format returns whether it sucessfully set the pixel format
	if (SetPixelFormat(hdc, n, &pfd))
	{
		return true;
	}
	else
	{
		return false;
	}
}
//--------------------------------------------------------------------

float getRandomFloat(float min, float max)
{
	return min + (static_cast<float>(rand()) / RAND_MAX) * (max - min);
}

void InitScene()
{
	
	for(int i = 0; i < pCount; i++)
	{
		auto entityID = currentScene.entity_manager.CreateEntity();
		auto& entity = currentScene.entity_manager.entityList[entityID];
	
		entity.rigidbody2DComponentID = currentScene.component_manager.CreateRigidbody2DComponent(entityID);
		auto& rgb2d = currentScene.component_manager.GetRigidbody2DComponent(entity.rigidbody2DComponentID);
		rgb2d.mass = getRandomFloat(2.5/6,10/6);
		rgb2d.forceApplied = rgb2d.forceApplied +  Vector2f(getRandomFloat(-.125/128/1.5,.125/128/1.5),getRandomFloat(-.125/128/1.5,.125/128/1.5));
	
		entity.transformComponentID = currentScene.component_manager.CreateTransformComponent(entityID);
		auto& transform = currentScene.component_manager.GetTransformComponent(entity.transformComponentID);
		transform.scale.x = sqrtf(rgb2d.mass) / 100;
		transform.position = Vector2f(getRandomFloat(-.75,.75),getRandomFloat(-.75,.75));
		
		entity.circleRendererComponentID = currentScene.component_manager.CreateCircleRendererComponent(entityID);
		auto& circleRenderer = currentScene.component_manager.GetCircleRendererComponent(entity.circleRendererComponentID);
		circleRenderer.color = RGBColor(getRandomFloat(.1,1),getRandomFloat(.1,1),getRandomFloat(.1,1));
	
		entity.circleColliderComponentID = currentScene.component_manager.CreateCircleColliderComponent(entityID, Vector2f(0,0), transform.scale.x);
		auto& collider = currentScene.component_manager.GetCircleColliderComponent(entity.circleColliderComponentID);
	}

}	

std::vector<std::string> SplitString(const std::string& str) {
	std::vector<std::string> tokens;
	std::stringstream ss(str);
	std::string token;
	while (std::getline(ss, token, ' ')) {
		if (!token.empty()) {
			tokens.push_back(token);
		}
	}
	return tokens;
}


//--------------------------------------------------------------------

int WINAPI WinMain(HINSTANCE hInst, HINSTANCE, LPSTR lpCmdLine, int nCmdShow)
{
	if (lpCmdLine != nullptr && strlen(lpCmdLine) > 0)
	{
		std::istringstream iss(lpCmdLine);
		std::string pCountStr;
		std::string methodStr;

		iss >> pCountStr >> methodStr; // first word = pCount, second word = method

		if (!pCountStr.empty())
			pCount = std::stoi(pCountStr); // convert pCount to integer

		if (!methodStr.empty())
		{
			// Make sure the method is valid
			if (methodStr == "SERIAL")
				Physics::bvhType = Physics::SERIAL;
			else if (methodStr == "OMP")
				Physics::bvhType = Physics::OMP;
			else if (methodStr == "CUDA")
				Physics::bvhType = Physics::CUDA;
			else
				MessageBoxA(NULL, "Invalid method specified!", "Error", MB_ICONERROR);
		}
	}
	
	WNDCLASSEX wc;
	ZeroMemory(&wc, sizeof(WNDCLASSEX));
	
	wc.cbSize = sizeof(WNDCLASSEX);
	wc.hInstance = GetModuleHandle(NULL);
	wc.lpfnWndProc = WindowProcedure;
	wc.lpszClassName = WINDOW_TITLE;
	wc.style = CS_HREDRAW | CS_VREDRAW;
	
	if (!RegisterClassEx(&wc)) return false;
	
	HWND hWnd = CreateWindow(WINDOW_TITLE, WINDOW_TITLE, WS_OVERLAPPEDWINDOW,
		0, 0, 800, 800,
		NULL, NULL, wc.hInstance, NULL);
	
	//--------------------------------
	//	Initialize window for OpenGL
	//--------------------------------
	
	HDC hdc = GetDC(hWnd);
	
	//	initialize pixel format for the window
	initPixelFormat(hdc);
	
	//	get an openGL context
	HGLRC hglrc = wglCreateContext(hdc);
	
	//	make context current
	if (!wglMakeCurrent(hdc, hglrc)) return false;
	
	FrameTimer gameTimer = FrameTimer();
	gameTimer.Init(60);
	
	InitScene();
	
	//--------------------------------
	//	End initialization
	//--------------------------------
	
	ShowWindow(hWnd, nCmdShow);
	
	MSG msg;
	ZeroMemory(&msg, sizeof(msg));
	
	while (true)
	{
		if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
		{
			if (msg.message == WM_QUIT) break;
	
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		int framesToUpdate = gameTimer.GetFramesToUpdate();
		
		//Physics
		Physics::DoScenePhysics(currentScene, framesToUpdate);
		//Draw
		Graphics::Render(currentScene);
	
		SwapBuffers(hdc);
	}
	//Clean up
	UnregisterClass(WINDOW_TITLE, wc.hInstance);
	
	
	return true;
}
//--------------------------------------------------------------------