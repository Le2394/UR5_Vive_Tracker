using System.Collections;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;

public class Controller : MonoBehaviour
{
    public Transform end_point;
    public Transform target;
    public float rotationSpeed = 80.0f;

    private Vector3 controllerOffset;
    private Vector3 endEffectorOffset;


    private bool offsetCaptured = false;


    void Update()
    {
/*
        float joystickX = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick).x;
        float joystickY = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick).y;

        float r2 = OVRInput.Get(OVRInput.Axis1D.SecondaryIndexTrigger, OVRInput.Controller.Touch);
        bool r2Pressed = OVRInput.Get(OVRInput.Button.SecondaryIndexTrigger, OVRInput.Controller.Touch);

        float r1 = OVRInput.Get(OVRInput.Axis1D.SecondaryHandTrigger, OVRInput.Controller.Touch);
        bool r1Pressed = OVRInput.GetDown(OVRInput.Button.SecondaryHandTrigger, OVRInput.Controller.Touch);

        if (r1Pressed)
        {
            axis += 1f;
            Debug.Log(axis);
        }

        if (axis % 2 == 0) 
        {
            if (OVRInput.Get(OVRInput.Button.One))
            {
                end_point.Rotate(Vector3.right, rotationSpeed * Time.deltaTime);
            }
            else if (OVRInput.Get(OVRInput.Button.Two))
            {
                end_point.Rotate(Vector3.left, rotationSpeed * Time.deltaTime);
            }
        }
        else
        {
            if (OVRInput.Get(OVRInput.Button.One))
            {
                end_point.Rotate(Vector3.up, rotationSpeed * Time.deltaTime);
            }
            else if (OVRInput.Get(OVRInput.Button.Two))
            {
                end_point.Rotate(Vector3.down, rotationSpeed * Time.deltaTime);
            }
        }

        //end_point.position = Vector3.Lerp(end_point.position, target.position, Time.deltaTime * 5.0f);
        
        if(r2Pressed && !offsetCaptured)
        {
            offsetCaptured = true;
            controllerOffset = target.position;
            endEffectorOffset = end_point.position;
        }

        if (r2Pressed && offsetCaptured)
        {
            end_point.position = (target.position - controllerOffset) + endEffectorOffset;
        }

        if(!r2Pressed)
        {
            offsetCaptured = false;
        }
*/

        //end_point.rotation = Quaternion.Slerp(end_point.rotation, target.rotation, Time.deltaTime * 5.0f);

        /*Vector3 targetEuler = target.rotation.eulerAngles;
        Vector3 endEuler = end_point.rotation.eulerAngles;
        
        endEuler.x = -targetEuler.z + 90;
        endEuler.y = -targetEuler.x + 90;
        endEuler.z = targetEuler.y + 450.0f;

        end_point.rotation = Quaternion.Euler(endEuler);*/
        //Debug.Log(end_point.eulerAngles);
    }
}
