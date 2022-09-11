using System;
using UnityEngine;

public class CPU_Fluid : MonoBehaviour
{
    [Header("Required")]
    public GameObject planeToTexture;
    public Material material;

    [Header("Simulation Parameters")]
    public int n = 64;
    [Range(0, 0.001f)] public float diff = 0f;
    [Range(0, 0.001f)] public float visc = 0f;
    [Range(0, 0.1f)] public float gravityMult = 0f;
    [Range(0, 1f)] public float timeModifier = 1f;
    public bool drawVector;
    public bool enableBoundary;
    float gravity = 9.81f;
    public float force = 75f;
    public float source = 100f;

    const int diffuseSolveIter = 20;

    // Texture to visualize the density field.
    Texture2D texture;


    // Arrays to hold the field values.
    int size;
    float[] u, u_prev, v, v_prev, dens, dens_prev;
    //v =  the kinematic viscosity of the
    //u =  the velocity field

    float[] sourceForce;
    float[] gravityForce;

    public Vector3 mousePosition;
    public Vector3 mouseDelta;

    float dt;
    // Start is called before the first frame update
    void Start()
    {
        //Define grid Size
        size = (n + 2) * (n + 2);

        // Create the empty arrays.
        u = new float[size];
        u_prev = new float[size];

        v = new float[size];
        v_prev = new float[size];

        dens = new float[size];
        dens_prev = new float[size];

        //Set plan material
        planeToTexture.GetComponent<MeshRenderer>().material = material;

        // Create a texture to display the density field and assign it to the material.
        texture = new Texture2D(n + 2, n + 2, TextureFormat.RGBAHalf, false);
        material.SetTexture("_MainTex", texture);

        //Test source force placed in the middle
        sourceForce = new float[size];
        gravityForce = new float[size];
        sourceForce[To1D(n / 2, n / 2)] = 100;
        Array.Fill<float>(gravityForce, gravity);

        //Add_Source(n, ref dens, ref sourceForce, dt);
        //Plane _plane = planeToTexture.GetComponent<MeshRenderer>().probeAnchor
    }

    // Update is called once per frame
    void Update()
    {
        dt = Time.deltaTime * timeModifier;
        GetFromUI(ref dens_prev, ref u_prev, ref v_prev);
        AddGravity(gravityMult);
        Vel_Step(n, ref u, ref v, ref u_prev, ref v_prev, visc, dt);
        Dens_Step(n, ref dens, ref dens_prev, ref u, ref v, diff, dt);
        DrawDensity();
        DrawVector();

    }

    private void AddGravity(float gravityMult)
    {
        Add_Source(n, ref v, ref gravityForce, gravityMult, dt);
    }

    Vector3 GetMousePosOnPlaneNormalized()
    {
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        RaycastHit hit;
        if (Physics.Raycast(ray, out hit))
        {
        }
        return new Vector3(hit.textureCoord.x, 0, hit.textureCoord.y);
    }

    Vector2Int GetIdFromPositionOnPlaneNormalized(Vector3 p)
    {
        p *= n;
        return new Vector2Int((int)p.x, (int)p.z);
    }


    void GetFromUI(ref float[] d, ref float[] u, ref float[] v)
    {
        for (int i = 0; i < size; i++)
        {
            d[i] = u[i] = v[i] = 0f;
        }
        mouseDelta = GetMousePosOnPlaneNormalized() - mousePosition;
        mousePosition = GetMousePosOnPlaneNormalized();

        if (!Input.GetMouseButton(0) && !Input.GetMouseButton(1) && !Input.GetMouseButton(2)) return;

        int x = GetIdFromPositionOnPlaneNormalized(mousePosition).x;
        int y = GetIdFromPositionOnPlaneNormalized(mousePosition).y;

        //Debug.Log("X : " + x + " ; Y : " + y);

        //Debug.Log("\n Density @:" +x+" ; " + y + " is : " + dens[To1D(x, y)]);
        //Debug.Log("\n Velocity @:" +x+" ; " + y + " is : " + v[To1D(x, y)]);

        if (x < 1 || x > n || y < 1 || y > n)
        {
            return;
        }

        if (Input.GetMouseButton(0))
        {
            u[To1D(x, y)] = force * mouseDelta.x * (n + 2);  // scale by resolution.
            v[To1D(x, y)] = force * mouseDelta.z * (n + 2);  // scale by resolution.
        }

        if (Input.GetMouseButton(1))
        {
            d[To1D(x, y)] = source;
        }

        if (Input.GetMouseButton(2))
        {
            ClearFields();
        }
    }
    void Add_Source(int n, ref float[] x, ref float[] s, float dt) //Add source forces to the density
    {
        int size = (n + 2) * (n + 2);
        for (int i = 0; i < size; i++)
        {
            x[i] += dt * s[i];
        }
    }
    void Add_Source(int n, ref float[] x, ref float[] s, float mult, float dt) //Add source forces to the density
    {
        int size = (n + 2) * (n + 2);
        for (int i = 0; i < size; i++)
        {
            x[i] += dt * s[i] * mult;
        }
    }

    void Diffuse(int n, int b, ref float[] x, ref float[] x0, float diff, float dt)
    {
        float a = dt * diff * n * n;

        LinSolve(n, b, ref x, ref x0, a, 1f + 4f * a);
    }

    void Advect(int n, int b, ref float[] d, ref float[] d0, ref float[] u, ref float[] v, float dt)
    {
        int i0, j0, i1, j1;
        float x, y, s0, t0, s1, t1, dt0;
        dt0 = dt * n;

        for (int i = 1; i <= n; i++)
        {
            for (int j = 1; j <= n; j++)
            {
                x = i - dt0 * u[To1D(i, j)];
                y = j - dt0 * v[To1D(i, j)];

                if (x < 0.5f) x = 0.5f;
                if (x > n + 0.5f) x = n + 0.5f;
                if (y < 0.5f) y = 0.5f;
                if (y > n + 0.5f) y = n + 0.5f;

                i0 = (int)x;
                i1 = i0 + 1;
                j0 = (int)y;
                j1 = j0 + 1;

                s1 = x - i0;
                s0 = 1 - s1;
                t1 = y - j0;
                t0 = 1 - t1;

                d[To1D(i, j)] = s0 * (t0 * d0[To1D(i0, j0)] + t1 * d0[To1D(i0, j1)])
                              + s1 * (t0 * d0[To1D(i1, j0)] + t1 * d0[To1D(i1, j1)]);
            }
        }
        SetBoundary(n, b, ref d);
    }

    void Dens_Step(int n, ref float[] x, ref float[] x0, ref float[] u, ref float[] v, float diff, float dt)
    {
        Add_Source(n, ref x, ref x0, dt);
        Swap(ref x0, ref x);
        Diffuse(n, 0, ref x, ref x0, diff, dt);
        Swap(ref x0, ref x);
        Advect(n, 0, ref x, ref x0, ref u, ref v, dt);
    }

    void Vel_Step(int n, ref float[] u, ref float[] v, ref float[] u0, ref float[] v0, float visc, float dt)
    {
        Add_Source(n, ref u, ref u0, dt);
        Add_Source(n, ref v, ref v0, dt);
        Swap(ref u0, ref u);
        Diffuse(n, 0, ref u, ref u0, visc, dt);
        Swap(ref v0, ref v);
        Diffuse(n, 0, ref v, ref v0, visc, dt);
        Project(n, ref u, ref v, ref u0, ref v0);
        Swap(ref u0, ref u);
        Swap(ref v0, ref v);
        Advect(n, 1, ref u, ref u0, ref u0, ref v0, dt);
        Advect(n, 2, ref v, ref v0, ref u0, ref v0, dt);
        Project(n, ref u, ref v, ref u0, ref v0);

    }

    void Project(int n, ref float[] u, ref float[] v, ref float[] p, ref float[] div)
    {
        float h = 1f / n;
        for (int i = 1; i <= n; i++)
        {
            for (int j = 1; j <= n; j++)
            {
                div[To1D(i, j)] = -.5f * h * (u[To1D(i + 1, j)] - u[To1D(i - 1, j)]
                                            + v[To1D(i, j + 1)] - v[To1D(i, j - 1)]);
                p[To1D(i, j)] = 0;
            }
        }
        SetBoundary(n, 0, ref div);
        SetBoundary(n, 0, ref p);

        LinSolve(n, 0, ref p, ref div, 1, 4);

        for (int i = 1; i <= n; i++)
        {
            for (int j = 1; j <= n; j++)
            {
                u[To1D(i, j)] -= 0.5f * n * (p[To1D(i + 1, j)] - p[To1D(i - 1, j)]);
                v[To1D(i, j)] -= 0.5f * n * (p[To1D(i, j + 1)] - p[To1D(i, j - 1)]);
            }
        }
        SetBoundary(n, 1, ref u);
        SetBoundary(n, 2, ref v);
    }
    private void LinSolve(int n, int b, ref float[] x, ref float[] x0, float a, float c)
    {
        for (int k = 0; k < diffuseSolveIter; k++)
        {
            for (int i = 1; i <= n; i++)
            {
                for (int j = 1; j <= n; j++)
                {
                    x[To1D(i, j)] = (x0[To1D(i, j)] + a * (x[To1D(i + 1, j)] + x[To1D(i - 1, j)] + x[To1D(i, j + 1)] + x[To1D(i, j - 1)])) / (c);
                }
            }
            SetBoundary(n, b, ref x);
        }
    }

    int To1D(int i, int j) //To convert 1 dimmension array to 2D 
    {
        return i + (n + 2) * j;
    }
    void Swap(ref float[] a, ref float[] b)
    {
        var tmp = a;
        a = b;
        b = tmp;
    }
    void ClearFields()
    {
        u = new float[size];
        v = new float[size];
        dens = new float[size];
    }
    void SetBoundary(int n, int b, ref float[] x)
    {
        if (enableBoundary)
        {

            for (int i = 1; i <= n; i++)
            {

                //b == 0 quand c'est u0 & v0 && Diffuse

                //IF b == 1 [u] => Reflect horizontal
                x[To1D(0, i)] = b == 1 ? -x[To1D(1, i)] : x[To1D(1, i)];
                x[To1D(n + 1, i)] = b == 1 ? -x[To1D(n, i)] : x[To1D(n, i)];

                //IF b == 2  [v] => Reflect vertical
                x[To1D(i, 0)] = b == 2 ? -x[To1D(i, 1)] : x[To1D(i, 1)];
                x[To1D(i, n + 1)] = b == 2 ? -x[To1D(i, n)] : x[To1D(i, n)];

            }


            //Corners
            x[To1D(0, 0)] = 0.5f * (x[To1D(1, 0)] + x[To1D(0, 1)]);
            x[To1D(0, n + 1)] = 0.5f * (x[To1D(1, n + 1)] + x[To1D(0, n)]);
            x[To1D(n + 1, 0)] = 0.5f * (x[To1D(n, 0)] + x[To1D(n + 1, 1)]);
            x[To1D(n + 1, n + 1)] = 0.5f * (x[To1D(n, n + 1)] + x[To1D(n + 1, n)]);
        }
    }
    void DrawDensity()
    {
        Color[] tmp = texture.GetPixels(0);

        for (int i = 0; i < tmp.Length; i++)
        {
            tmp[i] = new Color(dens[i], dens[i], dens[i], 1f);
        }

        texture.SetPixels(tmp, 0);
        texture.Apply();

        //Debug.Log("\n Velocity @:" + n/2 + " ; " + n / 2 + " is : " + v[To1D(n / 2, n / 2)]);
    }
    private void DrawVector()
    {
        if (drawVector)
        {
            for (int i = 1; i <= n; i++)
            {
                for (int j = 1; j <= n; j++)
                {

                    Vector3 startLine = new Vector3(1 - (i - n / 2), 0, -(j - n / 2)) / (n);
                    Vector3 endLine = new Vector3(-u[To1D(i, j)], 0, -v[To1D(i, j)]);

                    Vector3 direction = endLine;


                    //Vector3 startLineScaled = (startLine / n) + Vector3.one * 0.5f;


                        Debug.DrawLine(startLine, startLine + direction/10, Color.red);

                    /*
                            Debug.Log("\n start line : " + startLine);
                            Debug.Log("\n endLine :  " + endLine);
                            Debug.Log("\n direction :  " + (endLine - startLine));
                    */
                }
            }
        }
    }

}
