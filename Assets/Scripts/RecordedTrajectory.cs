using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Valve.Newtonsoft.Json;

public class RecordedTrajectory : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        SaveData saveData = new SaveData
        {
            joints = IK_control.joints,
        };
        string json = JsonConvert.SerializeObject(saveData);
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    private class SaveData
    {
        public float[] joints;
    }
}
