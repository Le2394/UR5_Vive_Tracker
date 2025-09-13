using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class GripperControl : MonoBehaviour
{
    private const float s_min = 0.0f;
    private const float s_max = 100.0f;

    private const float v_min = 55.0f;
    private const float v_max = 180.0f;

    private readonly float[] sensorCoefficients = new float[] { -3.27240413e-19f, 2.99419163e-17f,
                                                                -9.06959980e-16f, 2.22222222e+00f,
                                                                -2.03054500e-14f };

    private readonly float[] coefficients = new float[] { 2.858763157442875e-09f, -1.443536999119206e-07f,
                                                          1.0835088633169088e-05f, 0.008994451797350918f,
                                                          2.6805554267921243e-07f };

    public bool in_position;

    private float speed;

    private static GripperControl instance;
    public static GripperControl Instance { get =>  instance; }

    private float stroke;
    public float Stroke { get => stroke; }

    

    private float theta;
    public float Theta { get => theta; }

    private float theta_i;

    private GameObject R_Arm_ID_0; private GameObject R_Arm_ID_1;
    private GameObject R_Arm_ID_2;
    private GameObject L_Arm_ID_0; private GameObject L_Arm_ID_1;
    private GameObject L_Arm_ID_2;

    private int ctrl_state;

    public bool start_movemet;


    // Start is called before the first frame update
    void Start()
    {
        // Initialization of the end-effector movable parts.
        //  Right arm.
        R_Arm_ID_0 = transform.Find("R_Arm_ID_0").gameObject; R_Arm_ID_1 = transform.Find("R_Arm_ID_1").gameObject;
        R_Arm_ID_2 = R_Arm_ID_0.transform.Find("R_Arm_ID_2").gameObject;
        //  Left arm.
        L_Arm_ID_0 = transform.Find("L_Arm_ID_0").gameObject; L_Arm_ID_1 = transform.Find("L_Arm_ID_1").gameObject;
        L_Arm_ID_2 = L_Arm_ID_0.transform.Find("L_Arm_ID_2").gameObject;


        // Reset variables.
        ctrl_state = 0;
        //  Reset the read-only variables to null.
        in_position = false;
    }

    private void Awake()
    {
        if (GripperControl.instance != null) { Debug.LogError("Singleton Error!"); }
        GripperControl.instance = this;
    }

    // Update is called once per frame
    /*
        void FixedUpdate()
        {
            switch (ctrl_state)
            {
                case 0:
                    {
                        // If the values are out of range, clamp them.
                        __stroke = Mathf.Clamp(stroke, s_min, s_max);
                        __speed = Mathf.Clamp(speed, v_min, v_max);

                        if (start_movemet == true)
                        {
                            ctrl_state = 1;
                        }
                    }
                    break;

                case 1:
                    {
                        // Reset variables.
                        in_position = false;

                        // Convert the stroke to the angle in degrees.

                        //__theta = Polyval(coefficients, __stroke) * Mathf.Rad2Deg;

                        ctrl_state = 2;
                    }
                    break;

                case 2:
                    {
                        // Interpolate the orientation between the current position and the target position.
                        __theta_i = Mathf.MoveTowards(__theta_i, __theta, __speed * Time.deltaTime);

                        // Change the orientation of the end-effector arm.
                        //  Right arm.
                        R_Arm_ID_0.transform.localEulerAngles = new Vector3(0.0f, -__theta_i, 0.0f);
                        R_Arm_ID_1.transform.localEulerAngles = new Vector3(0.0f, -__theta_i, 0.0f);
                        R_Arm_ID_2.transform.localEulerAngles = new Vector3(0.0f, __theta_i, 0.0f);
                        //  Left arm.
                        L_Arm_ID_0.transform.localEulerAngles = new Vector3(0.0f, __theta_i, 0.0f);
                        L_Arm_ID_1.transform.localEulerAngles = new Vector3(0.0f, __theta_i, 0.0f);
                        L_Arm_ID_2.transform.localEulerAngles = new Vector3(0.0f, -__theta_i, 0.0f);

                        if (__theta_i == __theta)
                        {
                            in_position = true; start_movemet = false;
                            ctrl_state = 0;
                        }
                    }
                    break;

            }
        }
    */

    private void FixedUpdate()
    {
        float sensorAngle = TcpController.Instance.Angle;

        stroke = Mathf.Clamp(stroke, s_min, s_max);
        speed = Mathf.Clamp(speed, v_min, v_max);

        stroke = SensorPolyval(sensorCoefficients, sensorAngle);
        theta = Polyval(coefficients, stroke) * Mathf.Rad2Deg;
        //Debug.Log(stroke);
        //Debug.Log(sensorAngle);
        theta_i = Mathf.MoveTowards(theta_i, theta, speed * Time.deltaTime);

        R_Arm_ID_0.transform.localEulerAngles = new Vector3(0.0f, -theta_i, 0.0f);
        R_Arm_ID_1.transform.localEulerAngles = new Vector3(0.0f, -theta_i, 0.0f);
        R_Arm_ID_2.transform.localEulerAngles = new Vector3(0.0f, theta_i, 0.0f);
        
        L_Arm_ID_0.transform.localEulerAngles = new Vector3(0.0f, theta_i, 0.0f);
        L_Arm_ID_1.transform.localEulerAngles = new Vector3(0.0f, theta_i, 0.0f);
        L_Arm_ID_2.transform.localEulerAngles = new Vector3(0.0f, -theta_i, 0.0f);

    }

    public float SensorPolyval(float[] coefficients, float x)
    {

        float y = 0.0f; int n = coefficients.Length - 1;
        foreach (var (coeff_i, i) in coefficients.Select((coeff_i, i) => (coeff_i, i)))
        {

            y += coeff_i * Mathf.Pow(x, (n - i));
        }

        return y;
    }

    public float Polyval(float[] coefficients, float x)
    {
        float y = 0.0f; int n = coefficients.Length - 1;
        foreach (var (coeff_i, i) in coefficients.Select((coeff_i, i) => (coeff_i, i)))
        {

            y += coeff_i * Mathf.Pow(x, (n - i));
        }

        return y;
    }
}
