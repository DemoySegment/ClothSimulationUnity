using System.Collections;
using System.Collections.Generic;
using UnityEngine;


// BVH Node
public class BVHNode
{
    public Bounds Bounds;       // Bounding box for this node
    public List<int> Triangles; // Triangles in this node (leaf)
    public BVHNode Left;        // Left child
    public BVHNode Right;       // Right child
}


public class BVHPointInMesh
{
    private BVHNode root;
    private Vector3[] vertices;
    private int[] triangles;

    public BVHPointInMesh(Mesh mesh, Transform transform)
    {
        vertices = new Vector3[mesh.vertexCount];
        for (int i = 0; i < mesh.vertexCount; i++)
        {
            vertices[i] = transform.TransformPoint(mesh.vertices[i]); // Transform to world space
        }
        triangles = mesh.triangles;
        root = BuildBVH(0, triangles.Length / 3);
    }

    // Build the BVH recursively
    private BVHNode BuildBVH(int start, int end)
    {
        BVHNode node = new BVHNode();

        // Compute bounding box for the range of triangles
        node.Bounds = ComputeBounds(start, end);

        // If few triangles, make this a leaf node
        if (end - start <= 10)
        {
            node.Triangles = new List<int>();
            for (int i = start; i < end; i++)
            {
                node.Triangles.Add(i * 3); // Store the start index of each triangle
            }
            return node;
        }

        // Otherwise, split into two child nodes
        int mid = (start + end) / 2;
        node.Left = BuildBVH(start, mid);
        node.Right = BuildBVH(mid, end);

        return node;
    }

    private Bounds ComputeBounds(int start, int end)
    {
        Bounds bounds = new Bounds(vertices[triangles[start * 3]], Vector3.zero);
        for (int i = start * 3; i < end * 3; i++)
        {
            bounds.Encapsulate(vertices[triangles[i]]);
        }
        return bounds;
    }

    // Combined PointInMesh logic using BVH
    public bool IsPointInsideMesh(Vector3 point, out Vector3 closestPoint, out float distance)
    {
        closestPoint = Vector3.zero;
        distance = float.MaxValue;

        // Traverse BVH to test relevant triangles
        return QueryBVH(root, point, ref closestPoint, ref distance, Vector3.up);
    }

    private bool QueryBVH(BVHNode node, Vector3 point, ref Vector3 closestPoint, ref float distance, Vector3 rayDirection)
    {
        // Check bounding box containment and proximity
        if (!node.Bounds.Contains(point) && Vector3.Distance(node.Bounds.ClosestPoint(point), point) > distance)
        {
            return false;
        }

        bool isInside = false;

        // If this is a leaf node, test all triangles
        if (node.Triangles != null)
        {
            foreach (int triangleStartIndex in node.Triangles)
            {
                Vector3 v0 = vertices[triangles[triangleStartIndex]];
                Vector3 v1 = vertices[triangles[triangleStartIndex + 1]];
                Vector3 v2 = vertices[triangles[triangleStartIndex + 2]];

                // Test for ray intersection (PointInMesh)
                if (RayIntersectsTriangle(point, rayDirection, v0, v1, v2))
                {
                    isInside = !isInside; // Toggle for each intersection
                }

                // Compute the closest point on the triangle
                Vector3 projectedPoint = ClosestPointOnTriangle(point, v0, v1, v2);
                float dist = Vector3.Distance(point, projectedPoint);

                if (dist < distance)
                {
                    distance = dist;
                    closestPoint = projectedPoint;
                }
            }
            return isInside;
        }

        // Otherwise, recurse into child nodes
        bool leftInside = node.Left != null && QueryBVH(node.Left, point, ref closestPoint, ref distance, rayDirection);
        bool rightInside = node.Right != null && QueryBVH(node.Right, point, ref closestPoint, ref distance, rayDirection);

        return leftInside || rightInside;
    }


    // Ray-Triangle Intersection using Möller–Trumbore
    private bool RayIntersectsTriangle(Vector3 rayOrigin, Vector3 rayDirection, Vector3 v0, Vector3 v1, Vector3 v2)
    {
        Vector3 edge1 = v1 - v0;
        Vector3 edge2 = v2 - v0;
        Vector3 h = Vector3.Cross(rayDirection, edge2);
        float a = Vector3.Dot(edge1, h);

        if (a > -0.0001f && a < 0.0001f) return false;

        float f = 1.0f / a;
        Vector3 s = rayOrigin - v0;
        float u = f * Vector3.Dot(s, h);

        if (u < 0.0f || u > 1.0f) return false;

        Vector3 q = Vector3.Cross(s, edge1);
        float v = f * Vector3.Dot(rayDirection, q);

        if (v < 0.0f || u + v > 1.0f) return false;

        float t = f * Vector3.Dot(edge2, q);
        return t > 0.0001f; // Intersection exists
    }

    // Compute the closest point on a triangle to a given point
    private Vector3 ClosestPointOnTriangle(Vector3 point, Vector3 a, Vector3 b, Vector3 c)
    {
        Vector3 ab = b - a;
        Vector3 ac = c - a;
        Vector3 ap = point - a;

        float d1 = Vector3.Dot(ab, ap);
        float d2 = Vector3.Dot(ac, ap);

        if (d1 <= 0.0f && d2 <= 0.0f) return a;

        Vector3 bp = point - b;
        float d3 = Vector3.Dot(ab, bp);
        float d4 = Vector3.Dot(ac, bp);

        if (d3 >= 0.0f && d4 <= d3) return b;

        Vector3 cp = point - c;
        float d5 = Vector3.Dot(ab, cp);
        float d6 = Vector3.Dot(ac, cp);

        if (d6 >= 0.0f && d5 <= d6) return c;

        float vc = d1 * d4 - d3 * d2;

        if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
        {
            float v = d1 / (d1 - d3);
            return a + v * ab;
        }

        float vb = d5 * d2 - d1 * d6;

        if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
        {
            float w = d2 / (d2 - d6);
            return a + w * ac;
        }

        float va = d3 * d6 - d5 * d4;

        if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
        {
            float u = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return b + u * (c - b);
        }

        return a;
    }
}

