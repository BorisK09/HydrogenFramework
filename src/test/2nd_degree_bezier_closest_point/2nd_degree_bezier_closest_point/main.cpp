#include <iostream>
#include "windows.h"

#include "..\..\..\HydrogenFramework.h"

#include "SDL.h"

#include "functions.h"

#pragma comment (lib, "SDL2.lib")
#pragma comment (lib, "SDL2main.lib")

#pragma once


void DrawQBezier2D(SDL_Renderer* sdl_renderer, h2::Vector2f p0, h2::Vector2f p1, h2::Vector2f p2)
{
	
	int bez_div = 16;

	float bez_step = 1.0f/bez_div;

	h2::Vector2f pointMin, pointMax;

	float t;

	for (int i = 0; i < bez_div; i++)
	{
		t = bez_step*i;;

		pointMin = (1.0f-t)*(1.0f-t)*p0 + 2.0f*t*(1.0f-t)*p1 + t*t*p2;

		t = bez_step*(i+1);

		pointMax = (1.0f-t)*(1.0f-t)*p0 + 2.0f*t*(1.0f-t)*p1 + t*t*p2; 
		
		SDL_RenderDrawLine(sdl_renderer, (int)pointMin.x, (int)pointMin.y, (int)pointMax.x, (int)pointMax.y);
	}
}

template <class T>
int projectPointToQuadraticBezier(const T& p0, const T& p1, const T& p2, const T& inPoint, T* outPoint)
{
	T A, B;

	A = p1 - p0;
	B = p2 - 2.0f*p1 + p0;

	float a, b, c, d;

	a = h2::dot(B, B);
	b = 3.0f*h2::dot(A, B);

	T _M = p0 - inPoint;

	c = 2.0f*h2::dot(A, A) + h2::dot(_M, B);
	d = h2::dot(_M, A);

	float roots[3];

	int nRoots = h2::solveThirdDegreeEquation(a, b, c, d, roots);

	T point;

	//float minDistance;

	for (int i = 0; i < nRoots; i++)
	{
		std::cout << roots[i] << " ";

		if (roots[i] >= 0 && roots[i] <= 1)
		{
			//std::cout << roots[i];
			//std::cout << "   ";

			float t = roots[i];

			point = (1.0f-t)*(1.0f-t)*p0 + 2.0f*t*(1.0f-t)*p1 + t*t*p2;
		}

		*outPoint = point;

		/*minDist = h2::distanance(inPoint, points[0]);

		for (int j = 0; j < nPoints; j++)
		{
			if (h2::distanance(inPoint, points[j]) < 100.0)
			{
				nearestPoint = points[j];
			}
		}*/
	}



	//std::cout << "  point: " << point.x << "   " << point.y;

	std::cout << std::endl;

	return 1;
}


int main(int argc, char* argv[])
{
	UNREFERENCED_PARAMETER(argc);
	UNREFERENCED_PARAMETER(argv);

    SDL_Init(SDL_INIT_EVERYTHING);

	SDL_Window* app_window = SDL_CreateWindow("2nd degree bezier cosest point",
                                              SDL_WINDOWPOS_UNDEFINED,
                                              SDL_WINDOWPOS_UNDEFINED,
                                              640, 480,
                                              SDL_WINDOW_OPENGL);

	SDL_Renderer* renderer = SDL_CreateRenderer(app_window, -1, SDL_RENDERER_ACCELERATED);


	h2::Vector2f	bezP0(100.0f, 400.0f),
					bezP1(100.0f, 100.0f),
					bezP2(400.0f, 400.0f);

	int done = 0;

	SDL_Event ievent;

	int mouseX, mouseY;

	while (done == 0)
	{
		SDL_GetMouseState(&mouseX, &mouseY);

		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
		SDL_RenderClear(renderer);

		while (SDL_PollEvent(&ievent))
		{
			if (ievent.type == SDL_QUIT) {done = 1;}

			if (ievent.type == SDL_KEYDOWN)
			{
				if (ievent.key.keysym.sym == SDLK_ESCAPE)
				{ 
					done = 1; 
				} 
			}
		}

		SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
		DrawQBezier2D(renderer, bezP0, bezP1, bezP2);

		h2::Vector2f proj;

		projectPointToQuadraticBezier(bezP0, bezP1, bezP2, 
								      h2::Vector2f((float)mouseX, (float)mouseY), 
								      &proj);

		SDL_Rect proj_rect = {(int)proj.x - 2, (int)proj.y - 2, 4, 4};

		SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
		SDL_RenderDrawRect(renderer, &proj_rect);



		SDL_Rect cursor_rect = {mouseX - 2, mouseY - 2, 4, 4};

		SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
		SDL_RenderDrawRect(renderer, &cursor_rect);


		SDL_RenderPresent(renderer);
	}

    SDL_Quit();
    
    return 0;    
}