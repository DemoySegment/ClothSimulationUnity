using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;
using UnityEngine;

public class PBDSolver : MonoBehaviour
{
    // number of vertices
    public int n;
    // time step
    public float timeStep = 0.0333f;

    // damping
    float damping = 0.99f;
    // mass of the particles
    public float mass;

    public float alpha;

    public int iteration = 32;

    public List<int> fixedVertices = new List<int>();

    public enum SolverType {
        GaussSeidel,
        Jaccobi
    }
    public SolverType type = SolverType.Jaccobi;


    public List<PointInMesh> meshToCollide;

    public Vector3 wind = new Vector3(0,0,0);

    public bool dynamicWind = false;

    public bool ballCollision = false;

    // edge information: num edge * 2
    int[] edgeList;
    // the original length: num edge
    // idea: in PBD, the length serves for length constraint
    float[] distanceConstraint;

    // the angle constraints
    float[] bendConstraints;

    Dictionary<Tuple<int, int>, List<int>> edgeTriangleMap;


    // velocity of each particle
    Vector3[] velocity;

    // force applied to each particle
    Vector3[] force;

    // reference to the Mesh of the cloth
    Mesh mesh;
    // Start is called before the first frame update
    void Start()
    {
        // initialize mesh info and construct topology
        mesh = GetComponent<MeshFilter>().mesh;

        MeshHelper.InitializeQuadMesh(mesh, out edgeList,out edgeTriangleMap, out velocity, n);

        MeshHelper.BuildLengthConstraints(mesh.vertices, edgeList, out distanceConstraint);

        MeshHelper.BuildBendConstraints(mesh.vertices, edgeTriangleMap, out bendConstraints);

        // initialize force matrix
        force = new Vector3[velocity.Length];
        for (int i = 0; i < force.Length; i++)
        {
            force[i] = Vector3.zero;
        }

    }

    void UpdateWind()
    {
        float minValue = -10;
        float maxValue = 10;
        wind = new Vector3(
                UnityEngine.Random.Range(minValue, maxValue), // Random x value
                UnityEngine.Random.Range(minValue, maxValue), // Random y value
                UnityEngine.Random.Range(minValue, maxValue)  // Random z value
            );
        
    }
    void Update()
    {
        // in a PBD solver, 
        Vector3[] X = mesh.vertices;

        // apply force
        for (int i = 0; i < X.Length; i++)
        {
            // skip fixed vertices
            if (fixedVertices.Contains(i))
            {
                continue;
            }

            // apply gravity
            velocity[i] += timeStep * new Vector3(0, -9.8f, 0);

            // apply wind
            velocity[i] += timeStep *  wind / mass;
            velocity[i] *= damping;
            X[i] += timeStep * velocity[i];
        }
        mesh.vertices = X;


        for (int i = 0; i < iteration; i++)
        {
            switch (type)
            {
                case SolverType.Jaccobi:
                    JacobiSolver();
                    break;
                case SolverType.GaussSeidel:
                    GaussSeidelSolver();
                    break;
            }
            
        }

        if (ballCollision)
        {
            BallCollision(); 
        }
        else
        {
            MeshCollision();
        }

        

        mesh.RecalculateNormals();

        if (dynamicWind)
        {
            UpdateWind();
        }
    }

    void JacobiSolver()
    {
        Vector3[] vertices = mesh.vertices;

        // store vertex position updates
        Vector3[] sumX = new Vector3[vertices.Length];
        // store vertex count updates
        float[] sumN = new float[vertices.Length];

        // initialize arrays to 0
        for (int i = 0; i < vertices.Length; i++)
        {
            sumX[i] = Vector3.zero;
            sumN[i] = 0;
        }

        // for each edge calculate solution to the projection function
        for (int i = 0; i < edgeList.Length / 2; i++)
        {
            int v0 = edgeList[i * 2 + 0];
            int v1 = edgeList[i * 2 + 1];

            // note that 1 / 2 is actually mass_j / (mass_i + mass_j)
            Vector3 distance = vertices[v0] - vertices[v1];
            Vector3 update = (distance.magnitude - distanceConstraint[i]) * distance.normalized / 2;

            // save projection results and accumulate
            sumX[v0] += vertices[v0] - update;
            sumX[v1] += vertices[v1] + update;
            sumN[v0] += 1;
            sumN[v1] += 1;
        }

        for (int i = 0; i < vertices.Length; i++)
        {
            if (fixedVertices.Contains(i))
            {
                continue;
            }

            Vector3 value = (sumX[i] + alpha * vertices[i]) / (sumN[i] + alpha);

            velocity[i] += (value - vertices[i]) / timeStep;

            vertices[i] = value;

        }

        mesh.vertices = vertices;
    }

    void GaussSeidelSolver()
    {
        Vector3[] vertices = mesh.vertices;
        Vector3[] verticesNew = mesh.vertices;

        // distance constraint projection solver
        for (int i = 0; i < edgeList.Length / 2; i++)
        {
            int v0 = edgeList[i * 2 + 0];
            int v1 = edgeList[i * 2 + 1];

            // note that 1 / 2 is actually mass_j / (mass_i + mass_j)
            Vector3 distance = verticesNew[v0] - verticesNew[v1];
            Vector3 update = (distance.magnitude - distanceConstraint[i]) * distance.normalized / 2;


            // directly update projection results
            if (!fixedVertices.Contains(v0))
            {
                verticesNew[v0] = verticesNew[v0] - update;
            }
            if (!fixedVertices.Contains(v1))
            {
                verticesNew[v1] = verticesNew[v1] + update;
            }


        }


        for (int i = 0; i < vertices.Length; i++)
        {
            velocity[i] += (verticesNew[i] - vertices[i]) / timeStep;
        }
        mesh.vertices = verticesNew;
    }

    void BallCollision()
    {
        // Get the mesh and its vertices in local space
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] localVertices = mesh.vertices;
        Vector3[] worldVertices = new Vector3[localVertices.Length];

        // Convert the vertices to world space
        for (int i = 0; i < localVertices.Length; i++)
        {
            worldVertices[i] = transform.TransformPoint(localVertices[i]);
        }

        // Get the sphere's center in world space
        GameObject sphere = GameObject.Find("Sphere");
        Vector3 sphereCenter = sphere.transform.TransformPoint(Vector3.zero);

        // Get the sphere's radius (assumes uniform scaling)
        Mesh sphereMesh = sphere.GetComponent<MeshFilter>().mesh;
        float sphereRadius = (sphereMesh.vertices[0]).magnitude * sphere.transform.lossyScale.x * 1.1f;
        Debug.Log(sphereRadius);
        for (int i = 0; i < worldVertices.Length; i++)
        {
            if (fixedVertices.Contains(i))
            {
                continue;
            }

            Vector3 d = worldVertices[i] - sphereCenter;
            if (d.magnitude < sphereRadius)
            {
                // Resolve collision
                Vector3 correctedPosition = sphereCenter + sphereRadius * d.normalized;
                velocity[i] += (correctedPosition - worldVertices[i]) / timeStep;
                worldVertices[i] = correctedPosition;
            }
        }

        // Convert vertices back to local space and update the mesh
        for (int i = 0; i < worldVertices.Length; i++)
        {
            localVertices[i] = transform.InverseTransformPoint(worldVertices[i]);
        }

        mesh.vertices = localVertices;

        // Recalculate bounds and normals for proper rendering
        mesh.RecalculateBounds();
        mesh.RecalculateNormals();
    }

    void MeshCollision()
    {
        // Get mesh vertices in local space
        Vector3[] localVertices = mesh.vertices;
        Vector3[] worldVertices = new Vector3[localVertices.Length];
        float collisionThreshold = 0.01f; // Threshold to detect proximity collisions

        // Convert vertices to world space
        for (int i = 0; i < localVertices.Length; i++)
        {
            worldVertices[i] = transform.TransformPoint(localVertices[i]);
        }

        foreach (PointInMesh mesh in meshToCollide)
        {
            for (int i = 0; i < worldVertices.Length; i++)
            {
                if (fixedVertices.Contains(i))
                {
                    // Skip fixed vertices
                    continue;
                }

                Vector3 closestPointOnSurface;
                float distanceToSurface;

                // Check for collision or proximity
                if (!mesh.IsPointInsideMesh(worldVertices[i], out closestPointOnSurface, out distanceToSurface) &&
                    distanceToSurface > collisionThreshold)
                {
                    continue;
                }

                // Resolve collision
                Vector3 d = closestPointOnSurface - worldVertices[i];

                velocity[i] += (closestPointOnSurface - worldVertices[i]) / timeStep;

                // Correct position to prevent penetration
                worldVertices[i] = closestPointOnSurface;

                // Update velocity with damping to prevent oscillations
                Vector3 normal = d.normalized;
                //velocity[i] = Vector3.Reflect(velocity[i], normal) * 0.5f;
                

                // Optional: Zero out small velocity to stabilize
                if (velocity[i].magnitude < 0.001f)
                {
                    velocity[i] = Vector3.zero;
                }
            }
        }

        // Convert vertices back to local space and update the mesh
        for (int i = 0; i < worldVertices.Length; i++)
        {
            localVertices[i] = transform.InverseTransformPoint(worldVertices[i]);
        }

        mesh.vertices = localVertices;

        // Recalculate bounds and normals for proper rendering
        mesh.RecalculateBounds();
        mesh.RecalculateNormals();
    }






}
