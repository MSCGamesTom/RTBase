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

#define EPSILON 0.001f

class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal
	float area; // Triangle area
	float d; // For ray triangle if needed
	unsigned int materialIndex;
	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e1 = vertices[2].p - vertices[1].p;
		e2 = vertices[0].p - vertices[2].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p);
	}
	Vec3 centre() const
	{
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}

	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
		float denom = Dot(n, r.dir);
		if (denom == 0) { return false; }
		t = (d - Dot(n, r.o)) / denom;
		if (t < 0) { return false; }
		Vec3 p = r.at(t);
		float invArea = 1.0f / Dot(e1.cross(e2), n);
		u = Dot(e1.cross(p - vertices[1].p), n) * invArea;
		if (u < 0 || u > 1.0f) { return false; }
		v = Dot(e2.cross(p - vertices[2].p), n) * invArea;
		if (v < 0 || (u + v) > 1.0f) { return false; }
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
		return Vec3(0, 0, 0);
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
	AABB()
	{
		reset();
	}
	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}
	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
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
	// Add code here
	bool rayAABB(const Ray& r)
	{
		float t;
		rayAABB(r, t);
		return true;
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

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32

class BVHNode
{
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	std::vector<int> triangleIndices;

	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	// unsigned int offset;
	// unsigned char num;
	BVHNode()
	{
		r = NULL;
		l = NULL;
	}
	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles)
	{
		// Create an index list for all triangles.
		std::vector<int> indices(inputTriangles.size());
		for (unsigned int i = 0; i < inputTriangles.size(); i++)
		{
			indices[i] = i;
		}
		// Build recursively.
		recursiveBuild(inputTriangles, indices, 0, indices.size());
	}
	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		float tHit;
		// If the ray misses this node’s bounding box or the hit is farther than a recorded hit, return.
		if (!bounds.rayAABB(ray, tHit) || tHit > intersection.t)
			return;
		// If this is a leaf node, test all triangles.
		if (!l && !r)
		{
			for (int idx : triangleIndices)
			{
				float t, u, v;
				if (triangles[idx].rayIntersect(ray, t, u, v))
				{
					if (t < intersection.t)
					{
						intersection.t = t;
						intersection.ID = idx;
						// Convert barycentrics: alpha = 1 - u - v, beta = u, gamma = v.
						intersection.alpha = 1.0f - u - v;
						intersection.beta = u;
						intersection.gamma = v;
					}
				}
			}
		}
		else
		{
			// Otherwise, traverse both children.
			if (l)
				l->traverse(ray, triangles, intersection);
			if (r)
				r->traverse(ray, triangles, intersection);
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
		// Add visibility code here
		return true;
	}

private:
	// Recursive BVH build helper using binned SAH.
    void recursiveBuild(const std::vector<Triangle>& triangles, std::vector<int>& indices, int start, int end)
    {
        // Compute bounds for triangles in [start, end).
        bounds.reset();
        for (int i = start; i < end; i++)
        {
            int idx = indices[i];
            bounds.extend(triangles[idx].vertices[0].p);
            bounds.extend(triangles[idx].vertices[1].p);
            bounds.extend(triangles[idx].vertices[2].p);
        }
        int count = end - start;
        // Create a leaf if the number of triangles is small.
        if (count <= MAXNODE_TRIANGLES)
        {
            for (int i = start; i < end; i++)
                triangleIndices.push_back(indices[i]);
            return;
        }
        // Compute centroid bounds.
        AABB centroidBounds;
        centroidBounds.reset();
        for (int i = start; i < end; i++)
        {
            int idx = indices[i];
            Vec3 centroid = triangles[idx].centre();
            centroidBounds.extend(centroid);
        }
        Vec3 extent = centroidBounds.max - centroidBounds.min;
        // Choose the axis with maximum extent.
        int axis = 0;
        if (extent.y > extent.x && extent.y > extent.z)
            axis = 1;
        else if (extent.z > extent.x)
            axis = 2;
        // If the extent is too small, make a leaf.
        if (extent[axis] < EPSILON)
        {
            for (int i = start; i < end; i++)
                triangleIndices.push_back(indices[i]);
            return;
        }
        // Set up bins for SAH.
        const int numBins = BUILD_BINS;
        struct Bin {
            AABB bounds;
            int count = 0;
        };
        std::vector<Bin> bins(numBins);
        // Place triangles into bins.
        for (int i = start; i < end; i++)
        {
            int idx = indices[i];
            Vec3 centroid = triangles[idx].centre();
            float binPos = (centroid[axis] - centroidBounds.min[axis]) /
                           (centroidBounds.max[axis] - centroidBounds.min[axis]);
            int binIndex = std::min(numBins - 1, int(binPos * numBins));
            bins[binIndex].count++;
            bins[binIndex].bounds.extend(triangles[idx].vertices[0].p);
            bins[binIndex].bounds.extend(triangles[idx].vertices[1].p);
            bins[binIndex].bounds.extend(triangles[idx].vertices[2].p);
        }
        // Compute SAH cost for splitting between bins.
        std::vector<AABB> leftBounds(numBins);
        std::vector<int> leftCounts(numBins, 0);
        AABB current;
        current.reset();
        int countLeft = 0;
        for (int i = 0; i < numBins; i++)
        {
            current.extend(bins[i].bounds.min);
            current.extend(bins[i].bounds.max);
            countLeft += bins[i].count;
            leftBounds[i] = current;
            leftCounts[i] = countLeft;
        }
        std::vector<AABB> rightBounds(numBins);
        std::vector<int> rightCounts(numBins, 0);
        AABB currentRight;
        currentRight.reset();
        int countRight = 0;
        for (int i = numBins - 1; i >= 0; i--)
        {
            currentRight.extend(bins[i].bounds.min);
            currentRight.extend(bins[i].bounds.max);
            countRight += bins[i].count;
            rightBounds[i] = currentRight;
            rightCounts[i] = countRight;
        }
        // Find the best split position.
        float bestCost = FLT_MAX;
        int bestSplit = -1;
        float parentArea = bounds.area();
        for (int i = 0; i < numBins - 1; i++)
        {
            float cost = TRAVERSE_COST +
                         (leftBounds[i].area() * leftCounts[i] + rightBounds[i + 1].area() * rightCounts[i + 1]) /
                         parentArea * TRIANGLE_COST;
            if (cost < bestCost)
            {
                bestCost = cost;
                bestSplit = i;
            }
        }
        // If splitting is not beneficial, make a leaf.
        if (bestCost > count * TRIANGLE_COST)
        {
            for (int i = start; i < end; i++)
                triangleIndices.push_back(indices[i]);
            return;
        }
        // Partition the indices according to the chosen split.
        auto midIt = std::partition(indices.begin() + start, indices.begin() + end,
            [&](int idx) {
                Vec3 centroid = triangles[idx].centre();
                float binPos = (centroid[axis] - centroidBounds.min[axis]) /
                               (centroidBounds.max[axis] - centroidBounds.min[axis]);
                int binIndex = std::min(numBins - 1, int(binPos * numBins));
                return binIndex <= bestSplit;
            });
        int mid = midIt - indices.begin();
        // Create child nodes.
        l = new BVHNode();
        r = new BVHNode();
        l->recursiveBuild(triangles, indices, start, mid);
        r->recursiveBuild(triangles, indices, mid, end);
    }
};
