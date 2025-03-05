#include "pch.h"
#include "Geometry.h"

TEST(PlaneRayIntersect, BasicIntersection) {
    Plane p;
    p.init(Vec3(0, 1, 0), 0);

    Ray r;
    r.init(Vec3(0, 10, 0), Vec3(0, -1, 0));

    float t = 0.0f;
    bool intersects = p.rayIntersect(r, t);

    EXPECT_TRUE(intersects);

    // Since the ray starts at y = 10 and hits the plane at y = 0,
    // the intersection should occur at t = 10.
    EXPECT_FLOAT_EQ(t, 10.0f);
}

TEST(TriangleRayIntersect, BasicIntersection) {
    // Define a simple triangle in the XY plane
    Vertex v0 = { Vec3(0, 0, 0) };
    Vertex v1 = { Vec3(1, 0, 0) };
    Vertex v2 = { Vec3(0, 1, 0) };

    Triangle tri;
    tri.init(v0, v1, v2, 0);

    // Define a ray that intersects the triangle
    Ray r;
    r.init(Vec3(0.25f, 0.25f, 1.0f), Vec3(0, 0, -1));

    float t, u, v;
    bool intersects = tri.rayIntersect(r, t, u, v);

    EXPECT_TRUE(intersects);
    EXPECT_GT(t, 0.0f);
    EXPECT_GE(u, 0.0f);
    EXPECT_LE(u, 1.0f);
    EXPECT_GE(v, 0.0f);
    EXPECT_LE(v, 1.0f);
    EXPECT_LE(u + v, 1.0f); // Ensure (u,v) are inside the triangle
}

TEST(TriangleRayIntersect, NoIntersection) {
    // Define a triangle
    Vertex v0 = { Vec3(0, 0, 0) };
    Vertex v1 = { Vec3(1, 0, 0) };
    Vertex v2 = { Vec3(0, 1, 0) };

    Triangle tri;
    tri.init(v0, v1, v2, 0);

    // Define a ray that misses the triangle
    Ray r;
    r.init(Vec3(2, 2, 1), Vec3(0, 0, -1));

    float t, u, v;
    bool intersects = tri.rayIntersect(r, t, u, v);

    EXPECT_FALSE(intersects);
}

TEST(TriangleRayIntersect, ParallelRay) {
    // Define a triangle
    Vertex v0 = { Vec3(0, 0, 0) };
    Vertex v1 = { Vec3(1, 0, 0) };
    Vertex v2 = { Vec3(0, 1, 0) };

    Triangle tri;
    tri.init(v0, v1, v2, 0);

    // Define a ray parallel to the triangle's plane
    Ray r;
    r.init(Vec3(0.5f, 0.5f, 1.0f), Vec3(1, 1, 0));

    float t, u, v;
    bool intersects = tri.rayIntersect(r, t, u, v);

    EXPECT_FALSE(intersects);
}

TEST(TriangleRayIntersect, BehindTriangle) {
    // Define a triangle
    Vertex v0 = { Vec3(0, 0, 0) };
    Vertex v1 = { Vec3(1, 0, 0) };
    Vertex v2 = { Vec3(0, 1, 0) };

    Triangle tri;
    tri.init(v0, v1, v2, 0);

    // Define a ray that points away from the triangle
    Ray r;
    r.init(Vec3(0.25f, 0.25f, -1.0f), Vec3(0, 0, -1));

    float t, u, v;
    bool intersects = tri.rayIntersect(r, t, u, v);

    EXPECT_FALSE(intersects);
}

// AABB Ray Intersection Tests
TEST(AABBRayIntersect, BasicIntersection) {
    AABB box;
    box.min = Vec3(-1, -1, -1);
    box.max = Vec3(1, 1, 1);

    Ray r;
    r.init(Vec3(0, 0, 2), Vec3(0, 0, -1));

    float t;
    bool intersects = box.rayAABB(r, t);

    EXPECT_TRUE(intersects);
    EXPECT_GT(t, 0.0f);
}

TEST(AABBRayIntersect, NoIntersection) {
    AABB box;
    box.min = Vec3(-1, -1, -1);
    box.max = Vec3(1, 1, 1);

    Ray r;
    r.init(Vec3(2, 2, 2), Vec3(1, 1, 1));

    bool intersects = box.rayAABB(r);

    EXPECT_FALSE(intersects);
}

TEST(AABBRayIntersect, InsideBox) {
    AABB box;
    box.min = Vec3(-1, -1, -1);
    box.max = Vec3(1, 1, 1);

    Ray r;
    r.init(Vec3(0, 0, 0), Vec3(1, 0, 0));

    float t;
    bool intersects = box.rayAABB(r, t);

    EXPECT_TRUE(intersects);
    EXPECT_LT(t, 0.0f); // Indicates the ray starts inside the box
}

TEST(AABBRayIntersect, ParallelToBoxFace) {
    AABB box;
    box.min = Vec3(-1, -1, -1);
    box.max = Vec3(1, 1, 1);

    Ray r;
    r.init(Vec3(0, 2, 0), Vec3(1, 0, 0));

    bool intersects = box.rayAABB(r);

    EXPECT_FALSE(intersects);
}