using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static TMPro.SpriteAssetUtilities.TexturePacker_JsonArray;

public class ImplicitSpringSolver : MonoBehaviour
{
    // number of vertices
    public int n;
    // time step
    public float timeStep = 0.0333f;
    // damping
    public float damping = 0.99f;

    // mass of the particles
    public float mass;

    public float rho = 0.995f;
    public float spring_k = 8000;
    public int iteration = 32;

    public List<int> fixedVertices = new List<int>();

    public Vector3 wind = new Vector3(0, 0, 0);

    public bool dynamicWind = false;

    // edge information: num edge * 2
    int[] edgeList;
    // the original length of springs: num edge
    float[] lengthList;

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

        MeshHelper.InitializeQuadMesh(mesh, out edgeList, out edgeTriangleMap, out velocity, n);

        MeshHelper.BuildLengthConstraints(mesh.vertices, edgeList, out lengthList);

        // initialize force matrix
        force = new Vector3[velocity.Length];
        for (int i = 0; i < force.Length; i++)
        {
            force[i] = Vector3.zero; 
        }

    }

    // Update is called once per frame
    void Update()
    {

        // in each update, we use Newton's method to implicitly integrate for both X and V
        Vector3[] X = mesh.vertices;
        Vector3[] last_X = new Vector3[X.Length];
        Vector3[] X_hat = new Vector3[X.Length];
        Vector3[] gradient = new Vector3[X.Length];

        // first: damping the velocity V and initialize X_hat as X + timestep*V
        for (int i = 0; i < X.Length; i++)
        {
            velocity[i] *= damping;
            X[i] = X_hat[i] = X[i] + timeStep * velocity[i];
        }



        // Newton iterations to solve the optimization problem

        float omega = 1.0f;
        for (int i = 0; i < iteration; i++)
        {
            if (i == 0) omega = 1.0f;
            else if (i == 1) omega = 2.0f / (2.0f - rho * rho);
            else omega = 4.0f / (4.0f - rho * rho * omega);

            // calculate gradient matrix for each vertice
            Get_Gradient(X, X_hat, timeStep, ref gradient);

            // update x position: xi <- xi - (1/timeStep^2 * mass + 4* spring_k)^-1 * gradient
            for (int j = 0; j < gradient.Length; j++)
            {
                if (fixedVertices.Contains(j))
                {
                    continue;
                }

                Vector3 new_x = omega * (X[j] - 1 / (mass / (timeStep * timeStep) + spring_k * 4) * gradient[j]) + (1 - omega) * last_X[j];
                last_X[j] = X[j];
                X[j] = new_x;
            }
        }
        // update velocity
        for (int i = 0; i < X.Length; i++)
        {
            velocity[i] += (X[i] - X_hat[i]) / timeStep;
        }

        // apply new vertices
        mesh.vertices = X;

        if (dynamicWind)
        {
            UpdateWind();
        }

        //Collision_Handling();


        mesh.RecalculateNormals();
        

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

    void Get_Gradient(Vector3[] X, Vector3[] X_hat, float t, ref Vector3[] G)
    {

        // gradient = 1 / timestep ^ 2 * mass matrix * (X - X_hat) - force matrix
        Get_Force(X);

        for (int i = 0; i < G.Length; i++)
        {
            G[i] = 1 / (timeStep * timeStep) * mass * (X[i] - X_hat[i]) - force[i];
        }        

    }
    

    void Collision_Handling()
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

    void Get_Force(Vector3[] X)
    {
        // there should be at least two kinds of force

        // first: gravity
        for (int i = 0; i < force.Length; i++)
        {
            force[i] = new Vector3(0, -9.8f, 0);
        }

        // as each of the edges is viewed as spring, iterate through edges and apply spring force
        for (int i = 0; i < edgeList.Length / 2; i++)
        {
            int index1 = edgeList[i * 2 + 0];
            int index2 = edgeList[i * 2 + 1];

            Vector3 f = spring_k * (1 - lengthList[i] / (X[index1] - X[index2]).magnitude) * (X[index1] - X[index2]);
            force[index1] -= f;
            force[index2] += f;
        }

        // update: add wind force
        for (int i = 0; i < force.Length; i++)
        {
            force[i] += wind/mass;
        }
    }
}
