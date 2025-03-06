#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom *samplers;
	std::thread **threads;
	int numProcs;
	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread*[numProcs];
		samplers = new MTRandom[numProcs];
		clear();
	}
	void clear()
	{
		film->clear();
	}
	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		// Is surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Compute direct lighting here
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler)
	{
		// Add pathtracer code here
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour direct(Ray& r, Sampler* sampler)
	{
		// Compute direct lighting for an image sampler here
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour albedo(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return scene->background->evaluate(shadingData, r.dir);
	}
	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	void render()
	{
		film->incrementSPP();

		const int tileSize = 16;

		int Nx = (film->width + tileSize - 1) / tileSize;
		int Ny = (film->height + tileSize - 1) / tileSize;
		int totalTiles = Nx * Ny;

		std::atomic<int> nextTileindex(0);

		// One thread per processor
		for (int i = 0; i < numProcs; i++) {
			threads[i] = new std::thread([&, i]()
				{
					// Loop each thread until no more tiles
					while (true) {
						// Grab the next tile index
						int tileIndex = nextTileindex.fetch_add(1);
						if (tileIndex >= totalTiles) break;

						// Convert that 1D tile index into (tileX, tileY)
						int tileX = tileIndex % Nx;
						int tileY = tileIndex / Nx;

						// Compute the pixel range for this tile
						int startX = tileX * tileSize;
						int startY = tileY * tileSize;
						int endX = min(startX + tileSize, (int)film->width);
						int endY = min(startY + tileSize, (int)film->height);

						// Get the random sampler for this thread
						MTRandom& sampler = samplers[i];

						// Render all pixels in [startX,endX) x [startY,endY)
						for (int y = startY; y < endY; y++)
						{
							for (int x = startX; x < endX; x++)
							{
								float px = x + 0.5f;
								float py = y + 0.5f;
								Ray ray = scene->camera.generateRay(px, py);
								Colour col = viewNormals(ray);
								//Colour col = albedo(ray);
								film->splat(px, py, col);
								unsigned char r = (unsigned char)(col.r * 255);
								unsigned char g = (unsigned char)(col.g * 255);
								unsigned char b = (unsigned char)(col.b * 255);
								canvas->draw(x, y, r, g, b);
							}
						}
					}
				});
		}
		for (int i = 0; i < numProcs; i++)
		{
			threads[i]->join();
			delete threads[i];
		}
	}
	int getSPP()
	{
		return film->SPP;
	}
	void saveHDR(std::string filename)
	{
		film->save(filename);
	}
	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}
};