using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using Unity.Burst;



public class AABB
{
    public Vector3 Min { get; private set; }
    public Vector3 Max { get; private set; }

    public Mesh mesh;
    public Transform transform;

    public AABB(Mesh mesh, Transform transform)
    {
        // Initialize bounds
        Min = Vector3.positiveInfinity;
        Max = Vector3.negativeInfinity;
        this.mesh = mesh;
        this.transform = transform;

        // Transform all vertices to world space and update bounds
        foreach (Vector3 vertex in mesh.vertices)
        {
            Vector3 worldVertex = transform.TransformPoint(vertex);
            Min = Vector3.Min(Min, worldVertex);
            Max = Vector3.Max(Max, worldVertex);
        }
    }
    public void UpdateBounds()
    {
        Min = Vector3.positiveInfinity;
        Max = Vector3.negativeInfinity;

        foreach (Vector3 vertex in mesh.vertices)
        {
            Vector3 worldVertex = transform.TransformPoint(vertex);
            Min = Vector3.Min(Min, worldVertex);
            Max = Vector3.Max(Max, worldVertex);
        }
    }
    // Check if a point is inside the AABB
    public bool Contains(Vector3 point)
    {
        return point.x >= Min.x && point.x <= Max.x &&
               point.y >= Min.y && point.y <= Max.y &&
               point.z >= Min.z && point.z <= Max.z;
    }

    // Find the closest point on the AABB to a given point
    public Vector3 ClosestPoint(Vector3 point)
    {
        return new Vector3(
            Mathf.Clamp(point.x, Min.x, Max.x),
            Mathf.Clamp(point.y, Min.y, Max.y),
            Mathf.Clamp(point.z, Min.z, Max.z)
        );
    }
}
public class PointInMesh : MonoBehaviour
{

    public Mesh mesh;
    protected AABB aabb;
    private Vector3[] transformedVertices;

    void Start()
    {
        mesh = GetComponent<MeshFilter>().mesh;
        aabb = new AABB(mesh, transform);
        UpdateTransformedVertices();
    }

    void UpdateTransformedVertices()
    {
        Vector3[] vertices = mesh.vertices;
        transformedVertices = new Vector3[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
        {
            transformedVertices[i] = transform.TransformPoint(vertices[i]);
        }
    }

    void Update()
    {
        aabb.UpdateBounds();
        UpdateTransformedVertices(); // Only call if transform has changed
    }



    /// <summary>
    /// Determines if a point is inside the mesh and calculates the closest point on the surface and the distance.
    /// </summary>
    public virtual bool IsPointInsideMesh(Vector3 point, out Vector3 closestPointOnSurface, out float distanceToSurface)
    {
        closestPointOnSurface = Vector3.zero;
        distanceToSurface = float.MaxValue;

        // AABB check
        if (!aabb.Contains(point))
        {
            closestPointOnSurface = aabb.ClosestPoint(point);
            distanceToSurface = Vector3.Distance(point, closestPointOnSurface);
            return false;
        }

        // Prepare data for Jobs
        Vector3[] localVertices = mesh.vertices; // Local space vertices
        int[] triangles = mesh.triangles;

        // Transform vertices to world space (copy to a new array)
        Vector3[] worldVertices = new Vector3[localVertices.Length];
        for (int i = 0; i < localVertices.Length; i++)
        {
            worldVertices[i] = transform.TransformPoint(localVertices[i]); // Convert to world space
        }

        // Use NativeArrays for the job system
        NativeArray<Vector3> nativeVertices = new NativeArray<Vector3>(worldVertices, Allocator.TempJob);
        NativeArray<int> nativeTriangles = new NativeArray<int>(triangles, Allocator.TempJob);
        NativeArray<bool> intersectionResults = new NativeArray<bool>(triangles.Length / 3, Allocator.TempJob);

        Vector3 rayDirection = Vector3.right; // Arbitrary ray direction

        // Schedule the job
        RayTriangleIntersectionJob job = new RayTriangleIntersectionJob
        {
            Vertices = nativeVertices,
            Triangles = nativeTriangles,
            RayOrigin = point,
            RayDirection = rayDirection,
            Results = intersectionResults
        };

        JobHandle handle = job.Schedule(intersectionResults.Length, 64);
        handle.Complete();

        // Calculate intersection count and closest point
        int intersectionCount = 0;
        float minDistance = float.MaxValue;

        for (int i = 0; i < intersectionResults.Length; i++)
        {
            if (intersectionResults[i])
            {
                intersectionCount++;
            }

            // Calculate closest point on triangles in the main thread
            int index = i * 3;
            Vector3 v0 = worldVertices[triangles[index]];       // Use world-space vertices
            Vector3 v1 = worldVertices[triangles[index + 1]];
            Vector3 v2 = worldVertices[triangles[index + 2]];

            Vector3 currentClosestPoint = ClosestPointOnTriangle(point, v0, v1, v2);
            float currentDistance = Vector3.Distance(point, currentClosestPoint);

            if (currentDistance < minDistance)
            {
                minDistance = currentDistance;
                closestPointOnSurface = currentClosestPoint;
            }
        }

        distanceToSurface = minDistance;
        Vector3 direction = closestPointOnSurface - point;
        closestPointOnSurface = point + direction.normalized * distanceToSurface*1.1f;

        // Cleanup
        nativeVertices.Dispose();
        nativeTriangles.Dispose();
        intersectionResults.Dispose();

        // Odd number of intersections means the point is inside
        return (intersectionCount % 2) == 1;
    }


    private Vector3 ClosestPointOnTriangle(Vector3 point, Vector3 a, Vector3 b, Vector3 c)
    {
        Vector3 ab = b - a;
        Vector3 ac = c - a;
        Vector3 ap = point - a;

        float d1 = Vector3.Dot(ab, ap);
        float d2 = Vector3.Dot(ac, ap);
        if (d1 <= 0f && d2 <= 0f) return a;

        Vector3 bp = point - b;
        float d3 = Vector3.Dot(ab, bp);
        float d4 = Vector3.Dot(ac, bp);
        if (d3 >= 0f && d4 <= d3) return b;

        Vector3 cp = point - c;
        float d5 = Vector3.Dot(ab, cp);
        float d6 = Vector3.Dot(ac, cp);
        if (d6 >= 0f && d5 <= d6) return c;

        float vc = d1 * d4 - d3 * d2;
        if (vc <= 0f && d1 >= 0f && d3 <= 0f) return a + ab * (d1 / (d1 - d3));

        float vb = d5 * d2 - d1 * d6;
        if (vb <= 0f && d2 >= 0f && d6 <= 0f) return a + ac * (d2 / (d2 - d6));

        float va = d3 * d6 - d5 * d4;
        if (va <= 0f && (d4 - d3) >= 0f && (d5 - d6) >= 0f) return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));

        float denom = 1f / (va + vb + vc);
        float v = vb * denom;
        float w = vc * denom;
        return a + ab * v + ac * w;
    }
}

/// <summary>
/// Burst-compiled job for parallel ray-triangle intersection tests.
/// </summary>
[BurstCompile]
public struct RayTriangleIntersectionJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<Vector3> Vertices;
    [ReadOnly] public NativeArray<int> Triangles;
    [ReadOnly] public Vector3 RayOrigin;
    [ReadOnly] public Vector3 RayDirection;

    [WriteOnly] public NativeArray<bool> Results;

    public void Execute(int index)
    {
        int i = index * 3;
        Vector3 v0 = Vertices[Triangles[i]];
        Vector3 v1 = Vertices[Triangles[i + 1]];
        Vector3 v2 = Vertices[Triangles[i + 2]];

        Results[index] = RayIntersectsTriangle(RayOrigin, RayDirection, v0, v1, v2);
    }

    private bool RayIntersectsTriangle(Vector3 rayOrigin, Vector3 rayDirection, Vector3 v0, Vector3 v1, Vector3 v2)
    {
        const float EPSILON = 0.000001f;
        Vector3 edge1 = v1 - v0;
        Vector3 edge2 = v2 - v0;

        Vector3 h = Vector3.Cross(rayDirection, edge2);
        float a = Vector3.Dot(edge1, h);
        if (a > -EPSILON && a < EPSILON) return false;

        float f = 1.0f / a;
        Vector3 s = rayOrigin - v0;
        float u = f * Vector3.Dot(s, h);
        if (u < 0.0f || u > 1.0f) return false;

        Vector3 q = Vector3.Cross(s, edge1);
        float v = f * Vector3.Dot(rayDirection, q);
        if (v < 0.0f || u + v > 1.0f) return false;

        float t = f * Vector3.Dot(edge2, q);
        return t > EPSILON;
    }
}
