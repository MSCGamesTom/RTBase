#pragma once

#include "Core.h"
#include <random>
#include <algorithm>

class Sampler
{
public:
	virtual float next() = 0;
};

class MTRandom : public Sampler
{
public:
	std::mt19937 generator;
	std::uniform_real_distribution<float> dist;
	MTRandom(unsigned int seed = 1) : dist(0.0f, 1.0f)
	{
		generator.seed(seed);
	}
	float next()
	{
		return dist(generator);
	}
};

// Note all of these distributions assume z-up coordinate system
class SamplingDistributions
{
public:
	static Vec3 uniformSampleHemisphere(float r1, float r2)
	{
		float theta = acosf(1 - r1);
		float phi = 2 * M_PI * r2;
		return Vec3(sinf(theta) * cosf(phi), sinf(theta) * sinf(phi), r1);
	}
	static float uniformHemispherePDF(const Vec3 wi)
	{
		return 1.0f / (2 * M_PI);
	}
	static Vec3 cosineSampleHemisphere(float r1, float r2)
	{
		float theta = acosf(sqrtf(r1));
		float phi = 2 * M_PI * r2;
		return Vec3(sinf(theta) * cosf(phi), sinf(theta) * sinf(phi), r1);
	}
	static float cosineHemispherePDF(const Vec3 wi)
	{
		return wi.z / M_PI;
	}
	static Vec3 uniformSampleSphere(float r1, float r2)
	{
		float theta = acosf(1 - 2 * r1);
		float phi = 2 * M_PI * r2;
		return Vec3(sinf(theta) * cosf(phi), sinf(theta) * sinf(phi), r1);
	}
	static float uniformSpherePDF(const Vec3& wi)
	{
		return 1.0f / (4 * M_PI);
	}
};