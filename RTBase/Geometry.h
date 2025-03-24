#pragma once

#include "Core.h"
#include "Sampling.h"
#include <stack>

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

#define MollEPSILON 1e-7f
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

		if (std::abs(det) < MollEPSILON)
			return false;

		float invDet = 1.0f / det;
		Vec3 T = r.o - vertices[2].p;

		u = T.dot(p) * invDet;

		if ((u < 0 && abs(u) > MollEPSILON) || (u > 1 && abs(u - 1) > MollEPSILON))
			return false;

		p = Cross(T, e1);
		v = r.dir.dot(p) * invDet;

		if ((v < 0 && abs(v) > MollEPSILON) || (u + v > 1 && abs(u + v - 1) > MollEPSILON))
			return false;

		t = e2.dot(p) * invDet;

		if (t < MollEPSILON)
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
		updateCenter();
	}

	void updateCenter()
	{
		center = (min + max) * 0.5f;
	}

	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
		updateCenter();
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

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32
#define MAX_DEPTH 16 

class BVHNode
{
public:
	AABB bounds;
	BVHNode* right;
	BVHNode* left;
	std::vector<int> triangleIndices; // For leaf nodes

#ifdef DEBUG_BVH
	static int totalLeafNodes;
	static int totalInternalNodes;
#endif

	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	// unsigned int offset;
	// unsigned char num;
	BVHNode()
	{
		right = NULL;
		left = NULL;
	}

	~BVHNode()
	{
		delete left;
		delete right;
	}
	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& triangles, std::vector<int>& indices, int depth = 0)
	{
#ifdef DEBUG_BVH
		if (depth == 0) {
			totalLeafNodes = 0;
			totalInternalNodes = 0;
			std::cout << "[BVH] Building BVH..." << std::endl;
		}
#endif

		// Compute bounding box
		bounds.reset();
		for (int i : indices)
			bounds.extend(triangles[i]);

		// Stop if max depth is reached or few triangles remain
		if (indices.size() <= MAXNODE_TRIANGLES || depth >= MAX_DEPTH)
		{
			triangleIndices = indices;

#ifdef DEBUG_BVH
			totalLeafNodes++;
			std::cout << "[BVH] Leaf Node | Depth: " << depth
				<< " | Triangles: " << triangleIndices.size()
				<< " | Total Leaves: " << totalLeafNodes << std::endl;
#endif
			return;
		}

		// Compute the longest axis
		Vec3 extent = bounds.max - bounds.min;
		int axis = (extent.x > extent.y && extent.x > extent.z) ? 0 : (extent.y > extent.z) ? 1 : 2;

		// Sort triangles along the chosen axis
		std::sort(indices.begin(), indices.end(), [&](int a, int b) {
			return triangles[a].centre()[axis] < triangles[b].centre()[axis];
			});

		// Split at the median
		size_t mid = indices.size() / 2;
		std::vector<int> leftIndices(indices.begin(), indices.begin() + mid);
		std::vector<int> rightIndices(indices.begin() + mid, indices.end());

		if (leftIndices.empty() || rightIndices.empty()) {
			// If split failed, force leaf node
			triangleIndices = indices;
#ifdef DEBUG_BVH
			std::cout << "[BVH] Split Failed - Creating Leaf | Depth: " << depth
				<< " | Triangles: " << triangleIndices.size() << std::endl;
			totalLeafNodes++;
#endif
			return;
		}

		// Create child nodes
		left = new BVHNode();
		right = new BVHNode();
		left->build(triangles, leftIndices, depth + 1);
		right->build(triangles, rightIndices, depth + 1);

#ifdef DEBUG_BVH
		totalInternalNodes++;
		if (depth < 5) // Only print deeper nodes for major splits
		{
			std::cout << "[BVH] Internal Node | Depth: " << depth
				<< " | Split Axis: " << axis
				<< " | Left: " << leftIndices.size()
				<< " | Right: " << rightIndices.size()
				<< " | Total Internal Nodes: " << totalInternalNodes << std::endl;
		}

		if (depth == 0) { // Print final statistics after BVH root finishes
			std::cout << "[BVH Stats] Total Internal Nodes: " << totalInternalNodes
				<< " | Total Leaf Nodes: " << totalLeafNodes << std::endl;

			// Validate total triangle count
			int totalTriangles = 0;
			countTriangles(this, totalTriangles);
			std::cout << "[BVH] Total Triangles in Leaves: " << totalTriangles
				<< " (Expected: " << triangles.size() << ")" << std::endl;
		}
#endif
	}

	// Helper function to count triangles in leaf nodes
	void countTriangles(BVHNode* node, int& total)
	{
		if (!node) return;
		if (!node->left && !node->right)
			total += node->triangleIndices.size();
		else
		{
			countTriangles(node->left, total);
			countTriangles(node->right, total);
		}
	}

	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		if (!bounds.rayAABB(ray))
			return;

		if (!left && !right) {
			for (int idx : triangleIndices) {
				float t, u, v;
				if (triangles[idx].rayIntersect(ray, t, u, v) && t < intersection.t) {
					intersection.t = t;
					intersection.ID = idx;
					intersection.alpha = u;
					intersection.beta = v;
					intersection.gamma = 1.0f - (u + v);
				}
			}
		}
		else {
			if (left) left->traverse(ray, triangles, intersection);
			if (right) right->traverse(ray, triangles, intersection);
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
		if (!bounds.rayAABB(ray))
			return false;

		if (!left && !right)
		{
			for (int idx : triangleIndices)
			{
				float t, u, v;
				if (triangles[idx].rayIntersect(ray, t, u, v)) 
				{
					if (t < maxT)
					{
						return false;
					}
				}
			}
		}

		else
		{
			// recursive traversal call to child nodes
			if (left->bounds.rayAABB(ray))
				left->traverseVisible(ray, triangles, maxT);
			if (right->bounds.rayAABB(ray))
				right->traverseVisible(ray, triangles, maxT);
		}


		return true;
	}
};

#ifdef DEBUG_BVH
int BVHNode::totalLeafNodes = 0;
int BVHNode::totalInternalNodes = 0;
#endif

































//class BVHTree
//{
//private:
//	struct Node
//	{
//		AABB bounds;
//		unsigned int start, end;
//		unsigned int child_l, child_r;
//
//		Node(unsigned int start, unsigned int end) : start(start), end(end), child_l(0), child_r(0) {}
//	};
//
//	Triangle* triangles;		// list of triangles;
//	std::vector<Node> nodes;	// list of nodes
//	unsigned int* indices;		// list of triangle indices for nodes
//
//	const int maxDepth = 50;	// maximum depth of BVH
//	const float invBuildBins = 1.0f / (float)BUILD_BINS;	// saves inverse BUILD_BINS for optimization
//
//	float calcCost(float pArea, float lArea, float rArea, unsigned int lNum, unsigned int rNum)
//	{
//		return TRAVERSE_COST + TRIANGLE_COST * (lArea * lNum + rArea * rNum) / pArea;
//	}
//
//	float evaluateSplit(unsigned int node, unsigned int axis, float splitPos)
//	{
//		AABB boundA, boundB;				// bound of left and right child
//		unsigned int numA = 0, numB = 0;	// number of triangles in left and right child
//
//		// calculate bound of left and right child
//		for (unsigned int i = nodes[node].start; i < nodes[node].end; i++)
//		{
//			unsigned int index = indices[i];
//
//			float triCenter = triangles[index].center.coords[axis];
//
//			if (triCenter < splitPos)
//			{
//				boundA.extend(triangles[index].maxP);
//				boundA.extend(triangles[index].minP);
//				numA++;
//			}
//			else
//			{
//				boundB.extend(triangles[index].maxP);
//				boundB.extend(triangles[index].minP);
//				numB++;
//			}
//		}
//
//		// calculate and return cost
//		return calcCost(nodes[node].bounds.area(), boundA.area(), boundB.area(), numA, numB);
//	}
//
//	bool createSplit(unsigned int node, unsigned int axis, float splitPos)
//	{
//		unsigned int rStart = nodes[node].start;
//		// split the triangles
//		for (unsigned int i = nodes[node].start; i < nodes[node].end; i++)
//		{
//			float triCenter = triangles[indices[i]].center.coords[axis];
//			bool rChild = triCenter > splitPos;
//			if (rChild)
//			{
//				std::swap(indices[i], indices[rStart]);
//				rStart++;
//			}
//		}
//
//		// check for no split
//		if (rStart == nodes[node].start || rStart == nodes[node].end)
//			return false;
//
//		Node child_l(nodes[node].start, rStart);
//		Node child_r(rStart, nodes[node].end);
//
//		// calculate bounds of child nodes
//		for (unsigned int i = child_l.start; i < child_l.end; i++)
//		{
//			child_l.bounds.extend(triangles[indices[i]].maxP);
//			child_l.bounds.extend(triangles[indices[i]].minP);
//		}
//		for (unsigned int i = child_r.start; i < child_r.end; i++)
//		{
//			child_r.bounds.extend(triangles[indices[i]].maxP);
//			child_r.bounds.extend(triangles[indices[i]].minP);
//		}
//
//		// add child nodes to node list
//		nodes.emplace_back(child_l);
//		nodes.emplace_back(child_r);
//
//		// update parent nodes child indices
//		nodes[node].child_l = nodes.size() - 2;
//		nodes[node].child_r = nodes.size() - 1;
//
//		return true;
//	}
//	bool splitNode(unsigned int node)
//	{
//		const unsigned int numTest = 5;
//		unsigned int bestAxis = 0;
//		float bestPos = 0;
//		float bestCost = FLT_MAX;
//
//		// Avoid division inside the loop
//		float invNumTest = 1.0f / (float)numTest;
//
//		// Test splits along all 3 axes
//		for (int axis = 0; axis < 3; axis++)
//		{
//			float boundsStart = nodes[node].bounds.min.coords[axis];
//			float boundsEnd = nodes[node].bounds.max.coords[axis];
//
//			// Try multiple split positions
//			for (int i = 0; i < numTest; i++)
//			{
//				float splitT = (i + 1) * invNumTest;
//				float pos = boundsStart + (boundsEnd - boundsStart) * splitT;
//
//				float cost = evaluateSplit(node, axis, pos);
//
//				if (cost < bestCost)
//				{
//					bestCost = cost;
//					bestPos = pos;
//					bestAxis = axis;
//				}
//			}
//		}
//
//		// If the split cost is higher than the leaf cost, return false
//		if (calcCost(nodes[node].bounds.area(), nodes[node].end - nodes[node].start, 0, 0, 0) <= bestCost)
//			return false;
//
//		return createSplit(node, bestAxis, bestPos);
//	}
//	unsigned int split(unsigned int node, int depth = 0)
//	{
//		// check for max depth reached (leaf node)
//		if (depth >= maxDepth)
//			return depth;
//
//		// if no split is possible return (leaf node)
//		if (!splitNode(node))
//			return depth;
//
//		// recursive call to child spliting
//		depth++;
//		unsigned int depth1 = split(nodes[node].child_l, depth);
//		unsigned int depth2 = split(nodes[node].child_r, depth);
//
//		// return maximum depth (for debugging)
//		return std::max(depth1, depth2);
//	}
//
//public:
//
//	void build(std::vector<Triangle>& inputTriangles, AABB bounds)
//	{
//		// set triangles
//		triangles = &inputTriangles[0];
//
//		std::cout << "Total Triangles in scene is " << inputTriangles.size() << std::endl;
//
//		// clear data if any
//		nodes.clear();
//
//		// add root node to the data
//		indices = new unsigned int[inputTriangles.size()];
//		for (unsigned int i = 0; i < inputTriangles.size(); i++)
//			indices[i] = i;
//
//		nodes.emplace_back(Node(0, inputTriangles.size()));
//		nodes[0].bounds = bounds;
//
//		// split the root node (recursive call)
//		unsigned int depth = split(0);
//
//		//calculate statistics
//		unsigned int trices = 0;
//		unsigned int totalNodes = 0;
//		unsigned int maxTrice = 0;
//
//		for (auto& node : nodes)
//		{
//			// check for leaf node
//			if (node.child_l == 0 && node.child_r == 0)
//			{
//				unsigned int tri = node.end - node.start;
//				trices += tri;
//				if (tri > maxTrice)
//					maxTrice = tri;
//				totalNodes++;
//			}
//		}
//
//		std::cout << "\n------: BVH Info :------\n";
//		std::cout << "Total depth - " << depth << "/" << maxDepth << std::endl;
//		std::cout << "Total nodes - " << nodes.size() << std::endl;
//		std::cout << "Total Triangles - " << trices << std::endl;
//		std::cout << "Average triangle per node - " << float(trices) << std::endl;
//		std::cout << "Maximum triangles in a node - " << maxTrice << "\n\n";
//	}
//
//	void traverse(const Ray& ray, IntersectionData intersection)
//	{
//		// check for intersection with root node for early exit
//		if (!nodes[0].bounds.rayAABB(ray))
//			return;
//
//		std::stack<unsigned int> stack; //stack for node traversal
//		stack.push(0);					// push root node to stack
//
//		//traverse tree
//		while (!stack.empty()) {
//
//			// pop top node from stack
//			const Node& node = nodes[stack.top()];
//			stack.pop();
//
//			// check for leaf node to terminate recursion
//			if (node.child_l == 0 && node.child_r == 0)
//			{
//				// check for intersection with triangles
//				for (unsigned int i = node.start; i < node.end; i++)
//				{
//					float t, u, v;
//					unsigned int index = indices[i];
//
//					// check for intersections with triangle
//					if (triangles[index].rayIntersect(ray, t, u, v))
//					{
//						// update intersection data
//						if (t < intersection.t)
//						{
//							intersection.t = t;
//							intersection.ID = index;
//							intersection.alpha = u;
//							intersection.beta = v;
//							intersection.gamma = 1.0f - (u + v);
//						}
//					}
//				}
//			}
//			else
//			{
//				// recursive traversal call to child nodes
//				if (nodes[node.child_l].bounds.rayAABB(ray))
//					stack.push(node.child_l);
//				if (nodes[node.child_r].bounds.rayAABB(ray))
//					stack.push(node.child_r);
//			}
//		}
//	}
//	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
//	{
//		IntersectionData intersection;
//		intersection.t = FLT_MAX;
//		traverse(ray, intersection);
//		return intersection;
//	}
//	bool traverseVisible(const Ray& ray, const float maxT) {
//		//check for intersection with root node for early exit
//		if (!nodes[0].bounds.rayAABB(ray))
//			return false;
//
//		std::stack<unsigned int> stack; //stack for node traversal
//		stack.push(0);					// push root node to stack
//
//		//traverse tree
//		while (!stack.empty()) {
//
//			//pop top node from stack
//			const Node& node = nodes[stack.top()];
//			stack.pop();
//
//			// check for leaf node to terminate recursion
//			if (node.child_l == 0 && node.child_r == 0)
//			{
//				for (unsigned int i = node.start; i < node.end; i++)
//				{
//					float t, u, v;
//					unsigned int index = indices[i];
//
//					// check for intersections with triangle
//					if (triangles[index].rayIntersect(ray, t, u, v))
//						if (t <= maxT)
//							return false;
//				}
//			}
//			else
//			{
//				// recursive traversal call to child nodes
//				if (nodes[node.child_l].bounds.rayAABB(ray))
//					stack.push(node.child_l);
//				if (nodes[node.child_r].bounds.rayAABB(ray))
//					stack.push(node.child_r);
//			}
//		}
//
//		return true;
//	}
//};