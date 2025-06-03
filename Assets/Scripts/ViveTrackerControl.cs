using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NativeWebSocket;
using System.Text;
using Unity.VisualScripting;

public class ViveTrackerControl : MonoBehaviour
{
    public Transform endPoint;
    public Transform target;
    public float rotationSpeed = 80.0f;

    private Vector3 controllerPosOffset;
    private Vector3 endEffectorPosOffset;
    private Vector3 controllerRotOffset;
    private Vector3 endEffectorRotOffset;

    public string inputRemote;
    public WebSocket remoteWebSocket;

    public static string buttonStatus;
    private bool posOffsetCaptured = false;
    private bool rotOffsetCaptured = false;
    private bool posEnable = false;
    private bool rotEnable = false;

    private void Start()
    {
        InitializeSockets();
        ConnectToWebSocket();
    }

    private void InitializeSockets()
    {
        remoteWebSocket = new WebSocket($"ws://{inputRemote}:1111");
        Debug.Log("Initialize completely");
    }

    public async void ConnectToWebSocket()
    {
        string urlRemote = $"ws://{inputRemote}:1111";

        if (remoteWebSocket == null || remoteWebSocket.State != WebSocketState.Open)
        {
            remoteWebSocket = new WebSocket(urlRemote);
            SetupWebSocket(remoteWebSocket, "remote");

            try
            {
                await remoteWebSocket.Connect();
                Debug.Log("WebSocket connected successfully!");
            }
            catch (System.Exception ex)
            {
                Debug.LogError("WebSocket connection failed: " + ex.Message);
            }
        }
    }

    void SetupWebSocket(WebSocket socket, string part)
    {
        socket.OnOpen += () =>
        {
            Debug.Log($"{part} Connected!");

        };
        socket.OnMessage += (bytes) =>
        {
            string msg = Encoding.UTF8.GetString(bytes);
            if (msg == "Pos")
            {
                posEnable = true;
            }
            if (msg == "Rot")
            {
                rotEnable = true;
            }
        };
        socket.OnError += (e) => Debug.LogError($"{part} Error: {e}");
        socket.OnClose += (e) => Debug.LogWarning($"{part} Closed!");
    }

    void Update()
    {
        //end_point.position = Vector3.Lerp(end_point.position, target.position, Time.deltaTime * 5.0f);

        if (posEnable && !posOffsetCaptured)
        {
            posOffsetCaptured = true;
            Debug.Log("posEnable");
            controllerPosOffset = target.position;
            endEffectorPosOffset = endPoint.position;
        }

        if (rotEnable && !rotOffsetCaptured)
        {
            rotOffsetCaptured = true;
            Debug.Log("rotEnable");
            controllerRotOffset = target.rotation.eulerAngles;
            endEffectorRotOffset = endPoint.rotation.eulerAngles;
        }

        if (posEnable && posOffsetCaptured)
        {
            endPoint.position = (target.position - controllerPosOffset) + endEffectorPosOffset;

        }
        if (rotEnable && rotOffsetCaptured)
        {
            Vector3 rotation = target.rotation.eulerAngles;
            rotation = new Vector3(rotation.x, rotation.y, rotation.z);
            endPoint.rotation = Quaternion.Euler(rotation);
        }

        if (!rotEnable)
        { rotOffsetCaptured = false; }
        if(!posEnable)
        { posOffsetCaptured = false; }

        if (remoteWebSocket != null)
            remoteWebSocket.DispatchMessageQueue();
    }
}
