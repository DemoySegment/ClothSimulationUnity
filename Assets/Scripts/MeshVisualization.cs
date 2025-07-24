using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using UnityEngine;

[ExecuteAlways] // Ensures the script runs in both Play and Edit modes
public class MeshVisualization : MonoBehaviour
{
    public Color vertexColor = Color.red; // Color for vertices
    public Color edgeColor = Color.green; // Color for edges
    public float vertexSize = 0.05f; // Size of the vertex spheres

    private void OnDrawGizmos()
    {
        // Get the MeshFilter component
        MeshFilter meshFilter = GetComponent<MeshFilter>();
        if (meshFilter == null || meshFilter.sharedMesh == null)
        {
            return; // Exit if no mesh is found
        }

        // Get mesh data
        Mesh mesh = meshFilter.sharedMesh;
        Vector3[] vertices = mesh.vertices; // Vertex positions
        int[] triangles = mesh.triangles;   // Triangle indices

        Gizmos.matrix = transform.localToWorldMatrix; // Use object's local-to-world transformation

        // Draw vertices
        Gizmos.color = vertexColor;
        foreach (Vector3 vertex in vertices)
        {
            Gizmos.DrawSphere(vertex, vertexSize); // Draw a sphere at each vertex position
        }

        // Draw edges
        Gizmos.color = edgeColor;
        for (int i = 0; i < triangles.Length; i += 3)
        {
            // Each triangle consists of three vertices
            Vector3 v1 = vertices[triangles[i]];
            Vector3 v2 = vertices[triangles[i + 1]];
            Vector3 v3 = vertices[triangles[i + 2]];

            // Draw lines for the edges of the triangle
            Gizmos.DrawLine(v1, v2);
            Gizmos.DrawLine(v2, v3);
            Gizmos.DrawLine(v3, v1);
        }
    }
}

