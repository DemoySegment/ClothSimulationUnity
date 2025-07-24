using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting.FullSerializer;
using UnityEngine;
using Unity.Mathematics;
using System.Linq;
using UnityEngine.UI;

public static class MeshHelper
{
    public static void Quick_Sort(ref int[] a, int l, int r)
    {
        int j;
        if (l < r)
        {
            j = Quick_Sort_Partition(ref a, l, r);
            Quick_Sort(ref a, l, j - 1);
            Quick_Sort(ref a, j + 1, r);
        }
    }

    private static int Quick_Sort_Partition(ref int[] a, int l, int r)
    {
        int pivot_0, pivot_1, i, j;
        pivot_0 = a[l * 2 + 0];
        pivot_1 = a[l * 2 + 1];
        i = l;
        j = r + 1;
        while (true)
        {
            do ++i; while (i <= r && (a[i * 2] < pivot_0 || a[i * 2] == pivot_0 && a[i * 2 + 1] <= pivot_1));
            do --j; while (a[j * 2] > pivot_0 || a[j * 2] == pivot_0 && a[j * 2 + 1] > pivot_1);
            if (i >= j) break;
            Swap(ref a[i * 2], ref a[j * 2]);
            Swap(ref a[i * 2 + 1], ref a[j * 2 + 1]);
        }
        Swap(ref a[l * 2 + 0], ref a[j * 2 + 0]);
        Swap(ref a[l * 2 + 1], ref a[j * 2 + 1]);
        return j;
    }

    private static void Swap(ref int a, ref int b)
    {
        int temp = a;
        a = b;
        b = temp;
    }


    public static int GetTriangleVertexIndexExcept(int[]vertices, int triangleIndex, int exceptVertex0, int exceptVertex1)
    {
        for (var i = 0; i < 3; i++)
        {
            var vIndex = vertices[triangleIndex * 3 + i];
            if (vIndex != exceptVertex0 && vIndex != exceptVertex1)
            {
                return vIndex;
            }
        }
        return -1;
    }


    /// <summary>
    /// initialize the quad mesh and calculate the topology information stored in edgelist, lengthlist and velocity. Velocity is initialized to 0
    /// </summary>
    /// <param name="mesh"></param>
    /// <param name="edgeList"></param>
    /// <param name="lengthList"></param>
    /// <param name="V"></param>
    /// <param name="n"></param>
    public static void InitializeQuadMesh(Mesh mesh, out int[] edgeList,  out Dictionary<Tuple<int, int>, List<int>> edgeTriangleMap, out Vector3[] V, int n = 21)
    {
        //-----------
        //topological construction
        //-----------

        //Resize the mesh.
        Vector3[] positions = new Vector3[n * n];
        Vector2[] UV = new Vector2[n * n];
        // num triangle = 2* (n-1)^2
        int[] triangleList = new int[(n - 1) * (n - 1) * 2 * 3];

        // positions: x-> [-5,5], y -> 0, z -> [-5, 5]
        // UV: [0,1]
        for (int j = 0; j < n; j++)
        {
            for (int i = 0; i < n; i++)
            {
                positions[j * n + i] = new Vector3(5 - 10.0f * i / (n - 1), 0, 5 - 10.0f * j / (n - 1));
                UV[j * n + i] = new Vector3(i / (n - 1.0f), j / (n - 1.0f));
            }
        }

        // trianglize the mesh grid
        int t = 0;
        for (int j = 0; j < n - 1; j++)
        {
            for (int i = 0; i < n - 1; i++)
            {
                triangleList[t * 6 + 0] = j * n + i;
                triangleList[t * 6 + 1] = j * n + i + 1;
                triangleList[t * 6 + 2] = (j + 1) * n + i + 1;
                triangleList[t * 6 + 3] = j * n + i;
                triangleList[t * 6 + 4] = (j + 1) * n + i + 1;
                triangleList[t * 6 + 5] = (j + 1) * n + i;
                t++;
            }
        }

        mesh.vertices = positions;
        mesh.triangles = triangleList;
        mesh.uv = UV;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();

        // Construct the original edge list
        // triangleList.Length = num triangle * 3
        // num edge = num triangle * 3
        // edgeList.Length = num edge * 2
        int[] originalEdgeList = new int[triangleList.Length * 2];
        for (int i = 0; i < triangleList.Length; i += 3)
        {
            // first edge
            originalEdgeList[i * 2 + 0] = triangleList[i + 0];
            originalEdgeList[i * 2 + 1] = triangleList[i + 1];

            // second edge
            originalEdgeList[i * 2 + 2] = triangleList[i + 1];
            originalEdgeList[i * 2 + 3] = triangleList[i + 2];

            // third edge
            originalEdgeList[i * 2 + 4] = triangleList[i + 2];
            originalEdgeList[i * 2 + 5] = triangleList[i + 0];
        }
        
        

        // reorder the original edge list so that index0 < index1
        for (int i = 0; i < originalEdgeList.Length; i += 2)
        {
            if (originalEdgeList[i] > originalEdgeList[i + 1])
                Swap(ref originalEdgeList[i], ref originalEdgeList[i + 1]);
        }

        // update: before sorting, record edge's two adjacent triangles for bend constraint

        // two vertices of an edge -> triangle index
        edgeTriangleMap = new Dictionary<Tuple<int, int>, List<int>>();

        
        for (int i = 0; i < originalEdgeList.Length; i += 2)
        {
            Tuple<int,int> edgeTuple = new Tuple<int, int>(originalEdgeList[i], originalEdgeList[i+1]);
            if (!edgeTriangleMap.TryGetValue(edgeTuple, out List<int> triangleIndexList))
            {
                triangleIndexList = new List<int>(); 
                edgeTriangleMap[edgeTuple] = triangleIndexList;   
            }

            triangleIndexList.Add(triangleList[i/6*3]);
            triangleIndexList.Add(triangleList[i / 6*3 + 1]);
            triangleIndexList.Add(triangleList[i / 6*3 + 2]);

        }

        // remove edges that don't have two triangles
        var keysToRemove = edgeTriangleMap
            .Where(kvp => kvp.Value.Count == 3)
            .Select(kvp => kvp.Key)
            .ToList();

        foreach (var key in keysToRemove)
        {
            edgeTriangleMap.Remove(key);
        }




        // sort the original edge list using quicksort
        Quick_Sort(ref originalEdgeList, 0, originalEdgeList.Length / 2 - 1);

        // count unique edges
        int e_number = 0;
        for (int i = 0; i < originalEdgeList.Length; i += 2)
        {
            if (i == 0 || originalEdgeList[i + 0] != originalEdgeList[i - 2] || originalEdgeList[i + 1] != originalEdgeList[i - 1])
                e_number++;
        }

        



        edgeList = new int[e_number * 2];
        for (int i = 0, e = 0; i < originalEdgeList.Length; i += 2)
        {
            if (i == 0 || originalEdgeList[i + 0] != originalEdgeList[i - 2] || originalEdgeList[i + 1] != originalEdgeList[i - 1])
            {
                edgeList[e * 2 + 0] = originalEdgeList[i + 0];
                edgeList[e * 2 + 1] = originalEdgeList[i + 1];
                e++;
            }
        }




        V = new Vector3[positions.Length];
        for (int i = 0; i < positions.Length; i++)
            V[i] = new Vector3(0, 0, 0);
    }

    public static void BuildLengthConstraints(Vector3[] positions, int[] edgeList, out float[] lengthConstraints)
    {
        lengthConstraints = new float[edgeList.Length / 2];
        for (int e = 0; e < edgeList.Length / 2; e++)
        {
            int i = edgeList[e * 2 + 0];
            int j = edgeList[e * 2 + 1];
            lengthConstraints[e] = (positions[i] - positions[j]).magnitude;
        }
    }

    public static void BuildBendConstraints(Vector3[] positions, Dictionary<Tuple<int, int>, List<int>> edgeTriangleMap, out float[] bendConstraints)
    {
        bendConstraints = new float[edgeTriangleMap.Count];
        for(int j = 0; j < edgeTriangleMap.Count;j++)
        {
            var edge = edgeTriangleMap.ElementAt(j);
            var p0 = positions[edge.Key.Item1];
            var p1 = positions[edge.Key.Item2] - p0;

            int index2 = -1;
            int index3 = -1;
            List<int> PositionInTriangleIndex = edge.Value;
            for(int i = 0; i < PositionInTriangleIndex.Count; i++)
            {
                if (PositionInTriangleIndex[i] != edge.Key.Item1 && PositionInTriangleIndex[i] != edge.Key.Item2)
                {
                    if (index2 == -1)
                    {
                        index2 = i;
                    }
                    else
                    {
                        index3 = i;
                    }
                }
            }
            
            var p2 = positions[index2] - p0;
            var p3 = positions[index3] - p0;

            var n1 = math.normalize(math.cross(p1, p2));
            var n2 = math.normalize(math.cross(p1, p3));

            bendConstraints[j] = math.acos(math.dot(n1, n2));
        }
    }
}

/// <summary>
/// edge infor maintaining two index and two triangles
/// </summary>
public class Edge
{
    public int vIndex0;
    public int vIndex1;
    public List<int> triangleIndexes = new List<int>(2);

    public Edge(int vIndex0, int vIndex1)
    {
        this.vIndex0 = math.min(vIndex0, vIndex1);
        this.vIndex1 = math.max(vIndex0, vIndex1);
    }

    public override int GetHashCode()
    {
        return (vIndex0 << 16 | vIndex1);
    }

    public override bool Equals(object obj)
    {
        if (!(obj is Edge))
        {
            return false;
        }
        var edge2 = (Edge)obj;

        return vIndex0 == edge2.vIndex0 && vIndex1 == edge2.vIndex1;
    }

    public static bool operator ==(Edge e1, Edge e2)
    {
        var n1 = object.ReferenceEquals(e1, null);
        var n2 = object.ReferenceEquals(e2, null);
        if (n1 && n2)
        {
            return true;
        }
        if (n1 != n2)
        {
            return false;
        }
        return e1.vIndex0 == e2.vIndex0 && e1.vIndex1 == e2.vIndex1;
    }

    public static bool operator !=(Edge e1, Edge e2)
    {
        var n1 = object.ReferenceEquals(e1, null);
        var n2 = object.ReferenceEquals(e2, null);
        if (n1 && n2)
        {
            return false;
        }
        if (n1 != n2)
        {
            return true;
        }
        return e1.vIndex0 != e2.vIndex0 || e1.vIndex1 != e2.vIndex1;
    }

    public static int GetEdgeHash(int vIndex0, int vIndex1)
    {
        if (vIndex0 < vIndex1)
        {
            return (vIndex0 << 16 | vIndex1);
        }
        else
        {
            return (vIndex1 << 16 | vIndex0);
        }
    }

}
