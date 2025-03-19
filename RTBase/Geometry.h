#pragma once

#include "Core.h"
#include "Sampling.h"

class Ray
{
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	Ray()
	{
	}
	Ray(Vec3 _o, Vec3 _d)
	{
		init(_o, _d);
	}
	void init(Vec3 _o, Vec3 _d)
	{
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}
};

class Plane
{
public:
	Vec3 n;
	float d;
	void init(Vec3& _n, float _d)
	{
		n = _n;
		d = _d;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		t = (d - n.dot(r.o)) / n.dot(r.dir);
		return t >= 0;
	}
};

#define EPSILON 1e-7f

class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal
	float area; // Triangle area
	float d; // For ray triangle if needed
	Vec3 maxP, minP;
	Vec3 center;
	unsigned int materialIndex;
	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;

		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;

		e1 = vertices[0].p - vertices[2].p;
		e2 = vertices[1].p - vertices[2].p;

		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p);

		maxP = Max(vertices[0].p, Max(vertices[1].p, vertices[2].p));
		minP = Min(vertices[0].p, Min(vertices[1].p, vertices[2].p));

		center = minP + (maxP - minP) * 0.5f;
	}
	Vec3 centre() const
	{
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}

	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
		Vec3 p = Cross(r.dir, e2);
		float det = p.dot(e1);

		if (std::abs(det) < EPSILON)
			return false;

		float invDet = 1.0f / det;
		Vec3 T = r.o - vertices[2].p;

		u = T.dot(p) * invDet;

		if ((u < 0 && abs(u) > EPSILON) || (u > 1 && abs(u - 1) > EPSILON))
			return false;

		p = Cross(T, e1);
		v = r.dir.dot(p) * invDet;

		if ((v < 0 && abs(v) > EPSILON) || (u + v > 1 && abs(u + v - 1) > EPSILON))
			return false;

		t = e2.dot(p) * invDet;

		if (t < EPSILON)
			return false;

		return true;
	}
	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const
	{
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}
	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		float r1 = sampler->next();
		float r2 = sampler->next();
		
		float alpha = 1 - sqrt(r1);
		float beta = r2 * sqrt(r1);
		float gamma = 1 - (alpha + beta);
		
		pdf = 1 / area;
		
		Vec3 p = vertices[0].p * alpha + vertices[1].p * beta + vertices[2].p * gamma;

		return p;
	}
	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

class AABB
{
public:
	Vec3 max;
	Vec3 min;
	Vec3 center;
	AABB()
	{
		reset();
	}
	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
		center = (min + max) * 0.5f;
	}
	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
	}
	void extend(const AABB& other)
	{
		extend(other.min);
		extend(other.max);
	}
	void extend(Triangle triangle) {
		extend(triangle.vertices[0].p);
		extend(triangle.vertices[1].p);
		extend(triangle.vertices[2].p);
	}
	// Add code here
	bool rayAABB(const Ray& r, float& t)
	{
		Vec3 tMin = (min - r.o) * r.invDir;
		Vec3 tMax = (max - r.o) * r.invDir;

		Vec3 tEnter = Min(tMin, tMax);
		Vec3 tExit = Max(tMin, tMax);

		float t_entry = std::max(tEnter.x, std::max(tEnter.y, tEnter.z));
		float t_exit = std::min(tExit.x, std::min(tExit.y, tExit.z));

		if (t_entry > t_exit || t_exit < 0) return false; // No intersection
		t = (t_entry < 0) ? t_exit : t_entry; // If inside the box, use t_exit
		return true;
	}

	bool rayAABB(const Ray& r)
	{
		float t;
		return rayAABB(r, t);
	}

	float area()
	{
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}
};

class Sphere
{
public:
	Vec3 centre;
	float radius;
	void init(Vec3& _centre, float _radius)
	{
		centre = _centre;
		radius = _radius;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		return false;
	}
};

struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

#define MAXNODE_TRIANGLES 16
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32
#define MAX_DEPTH 16 

#ifdef DEBUG_BVH
static int totalLeafNodes = 0;
static int totalInternalNodes = 0;
#endif

class BVHNode
{
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	std::vector<int> triangleIndices; // For leaf nodes

	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	// unsigned int offset;
	// unsigned char num;
	BVHNode()
	{
		r = NULL;
		l = NULL;
	}

	~BVHNode()
	{
		delete l;
		delete r;
	}
	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& triangles, std::vector<int>& indices, int depth = 0)
	{
		bounds.reset();
		for (int i : indices)
			bounds.extend(triangles[i]);

		if (indices.size() <= MAXNODE_TRIANGLES)
		{
			triangleIndices = indices;
			return;
		}

		// Compute centroid bounds
		AABB centroidBounds;
		for (int i : indices)
			centroidBounds.extend(triangles[i].centre());

		Vec3 extent = centroidBounds.max - centroidBounds.min;

		// Choose the axis with the largest extent
		int bestAxis = (extent.x > extent.y && extent.x > extent.z) ? 0 : (extent.y > extent.z) ? 1 : 2;
		float parentArea = bounds.area();

		// SAH Binning
		struct Bin
		{
			AABB bounds;
			int count = 0;
		};
		const int BINS = 16;
		Bin bins[BINS];

		float invExtent = 1.0f / extent[bestAxis];

		// Assign triangles to bins
		for (int i : indices)
		{
			int binIdx = BINS * (triangles[i].centre()[bestAxis] - centroidBounds.min[bestAxis]) * invExtent;
			binIdx = std::min(std::max(binIdx, 0), BINS - 1);
			bins[binIdx].count++;
			bins[binIdx].bounds.extend(triangles[i]);
		}

		// Compute SAH cost for each split
		float bestCost = FLT_MAX;
		int bestSplit = -1;
		AABB leftBounds, rightBounds;
		int leftCount = 0, rightCount = indices.size();

		for (int i = 0; i < BINS - 1; i++)
		{
			leftBounds.extend(bins[i].bounds);
			leftCount += bins[i].count;
			rightCount -= bins[i].count;

			float cost = TRAVERSE_COST + (leftBounds.area() / parentArea) * leftCount * TRIANGLE_COST +
				(rightBounds.area() / parentArea) * rightCount * TRIANGLE_COST;

			if (cost < bestCost)
			{
				bestCost = cost;
				bestSplit = i;
			}
		}

		// If SAH cost is not beneficial, create a leaf
		if (bestSplit == -1 || bestCost >= indices.size() * TRIANGLE_COST)
		{
			triangleIndices = indices;
			return;
		}

		// Partition the indices into left and right
		std::vector<int> leftIndices, rightIndices;
		float splitPos = centroidBounds.min[bestAxis] + (bestSplit + 1) * (extent[bestAxis] / BINS);

		for (int i : indices)
		{
			if (triangles[i].centre()[bestAxis] < splitPos)
				leftIndices.push_back(i);
			else
				rightIndices.push_back(i);
		}

		if (leftIndices.empty() || rightIndices.empty())
		{
			triangleIndices = indices;
			return;
		}

		// Create child nodes
		l = new BVHNode();
		r = new BVHNode();
		l->build(triangles, leftIndices, depth + 1);
		r->build(triangles, rightIndices, depth + 1);
	}

	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		float tHit;
		if (!bounds.rayAABB(ray, tHit) || tHit > intersection.t)
			return;

		if (!l && !r) {
			for (int idx : triangleIndices) {
				float t, u, v;
				if (triangles[idx].rayIntersect(ray, t, u, v) && t < intersection.t) {
					intersection.t = t;
					intersection.ID = idx;
					intersection.alpha = 1.0f - u - v;
					intersection.beta = u;
					intersection.gamma = v;
				}
			}
		}
		else {
			if (l) l->traverse(ray, triangles, intersection);
			if (r) r->traverse(ray, triangles, intersection);
		}
	}
	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, intersection);
		return intersection;
	}
	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT)
	{
		float tHit;
		if (!bounds.rayAABB(ray, tHit) || tHit > maxT)
			return false;

		if (!l && !r)
		{
			for (int idx : triangleIndices)
			{
				float t, u, v;
				if (triangles[idx].rayIntersect(ray, t, u, v) && t < maxT)
					return false; // Occluded
			}
			return true;
		}

		return (l && l->traverseVisible(ray, triangles, maxT)) &&
			(r && r->traverseVisible(ray, triangles, maxT));
	}
};
