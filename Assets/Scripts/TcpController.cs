using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEngine.UIElements;

public class TcpController : MonoBehaviour
{
    private TcpListener listener;
    private TcpClient client;
    private NetworkStream stream;

    public Transform endPoint;
    public Transform target;

    private Vector3 controllerPosOffset;
    private Vector3 endEffectorPosOffset;
    private Vector3 controllerRotOffset;
    private Vector3 endEffectorRotOffset;

    public int port = 1111;
    private Thread listenThread;

    private bool posOffsetCaptured = false;
    private bool rotOffsetCaptured = false;
    private bool posEnable = false;
    private bool rotEnable = false;

    private static TcpController instance;
    public static TcpController Instance {  get => instance; }
    
    private float angle;
    public float Angle { get => angle; }

    public float speed = 50f;
    private void Start()
    {
        listenThread = new Thread(StartListening);
        listenThread.IsBackground = true;
        listenThread.Start();
    }

    private void Awake()
    {
        if(TcpController.instance != null) { Debug.LogError("Singleton Error!"); }
        TcpController.instance = this;
    }
    void StartListening()
    {
        try
        {
            listener = new TcpListener(IPAddress.Any, port);
            listener.Start();

            client = listener.AcceptTcpClient();
            Debug.Log("ESP32 connected!");
            stream = client.GetStream();

            byte[] buffer = new byte[1024];
            using (var reader = new System.IO.StreamReader(stream, Encoding.ASCII))
            {
                while (client.Connected)
                {
                    string message = reader.ReadLine();
                    if (!string.IsNullOrEmpty(message))
                    {
                        Debug.Log(message);
                        if (message.StartsWith("Angle:"))
                        {
                            string angleStr = message.Substring(6);
                            angle = float.Parse(angleStr);
                        }
                        if (message == "Pos") posEnable = true;
                        if (message == "Rot") rotEnable = true;
                        if (message == "stopPos") posEnable = false;
                        if (message == "stopRot") rotEnable = false;

                    }
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogError("Server error: " + e.Message);
        }
    }

    private void Update()
    {
        if (posEnable && !posOffsetCaptured)
        {
            posOffsetCaptured = true;

            controllerPosOffset = target.position;
            endEffectorPosOffset = endPoint.position;
        }

        if (rotEnable && !rotOffsetCaptured)
        {
            rotOffsetCaptured = true;

            controllerRotOffset = target.rotation.eulerAngles;
            endEffectorRotOffset = endPoint.rotation.eulerAngles;
        }

        if (posEnable && posOffsetCaptured)
        {
            Vector3 targetPosition = new Vector3(-(target.position.x - controllerPosOffset.x) + endEffectorPosOffset.x,
                                                  (target.position.y - controllerPosOffset.y) + endEffectorPosOffset.y,
                                                 -(target.position.z - controllerPosOffset.z) + endEffectorPosOffset.z);
            endPoint.position = Vector3.Lerp(endPoint.position, targetPosition, speed * Time.deltaTime);
        }
        if (rotEnable && rotOffsetCaptured)
        {
            Vector3 rotation = target.rotation.eulerAngles;
            rotation = new Vector3(-rotation.x, rotation.y, -rotation.z);
            Quaternion targetRotation = Quaternion.Euler(rotation);

            endPoint.rotation = Quaternion.Lerp(endPoint.rotation, targetRotation, speed * Time.deltaTime);
        }

        if (!rotEnable)
        { rotOffsetCaptured = false; }
        if (!posEnable)
        { posOffsetCaptured = false; }
    }

    private void OnApplicationQuit()
    {
        stream?.Close();
        client?.Close();
        listener?.Stop();
        listenThread?.Abort();
    }
}
