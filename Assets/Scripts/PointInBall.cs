using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PointInBall : PointInMesh
{
    public override bool IsPointInsideMesh(Vector3 point, out Vector3 closestPointOnSurface, out float distanceToSurface)
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
        Vector3 center = transform .position;
        Vector3 dir = point - center;
        float radius = (mesh.vertices[0]).magnitude* transform.lossyScale.x *1.1f;
        if ((point - center).magnitude > radius)
        {
            return false;
        }
        closestPointOnSurface = center + dir.normalized * radius;
        distanceToSurface = (closestPointOnSurface - point).magnitude;
        return true;
    }
}
