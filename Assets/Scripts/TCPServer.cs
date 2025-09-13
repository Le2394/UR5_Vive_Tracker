using System;
using System.Collections.Concurrent;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEditor.PackageManager;
using UnityEngine;

public class TCPServer : MonoBehaviour
{
    public string ipAddress = "192.168.55.109";
    public int jointPort = 10000;
    public int strokePort = 10001;
    public bool control;

    private TcpListener jointListener;
    private TcpListener strokeListener;

    private Thread jointThread;
    private Thread strokeThread;

    private TcpClient jointClient;
    private TcpClient strokeClient;

    private TcpController endPoint;
    private bool running = false;

    private ConcurrentQueue<string> logQueue = new ConcurrentQueue<string>();
    private Vector3 currentEndPointPosition = Vector3.zero;
    private readonly object eeLock = new object();

    void Start()
    {
        StartServers();
    }

    void StartServers()
    {
        try
        {
            IPAddress localAddr = IPAddress.Parse(ipAddress);
            jointListener = new TcpListener(IPAddress.Any, jointPort);
            strokeListener = new TcpListener(IPAddress.Any, strokePort);

            jointListener.Start();
            strokeListener.Start();
            running = true;

            jointThread = new Thread(RunJointServer);
            strokeThread = new Thread(RunStrokeServer);
            jointThread.IsBackground = true;
            strokeThread.IsBackground = true;

            jointThread.Start();
            strokeThread.Start();

            Debug.Log($"Joint server started on {ipAddress}:{jointPort}");
            Debug.Log($"Stroke server started on {ipAddress}:{strokePort}");
        }
        catch (Exception ex)
        {
            Debug.LogError($"Server failed to start: {ex.Message}");
        }
    }

    void RunJointServer()
    {
        while (running)
        {
            Debug.Log("Waiting for joint client...");
            jointClient = jointListener.AcceptTcpClient();
            Debug.Log("Joint client connected!");
            Thread clientThread = new Thread(HandleJointClient);
            clientThread.IsBackground = true;
            clientThread.Start();
        }
    }
    void Update()
    {
        while (logQueue.TryDequeue(out string message))
        {
            Debug.Log(message);
        }

        // Update endpoint position safely
        if (endPoint != null)
        {
            lock (eeLock)
            {
                currentEndPointPosition = endPoint.position;
            }
        }
    }
    void RunStrokeServer()
    {
        while (running)
        {
            Debug.Log("Waiting for stroke client...");
            strokeClient = strokeListener.AcceptTcpClient();
            Debug.Log("Stroke client connected!");
            Thread clientThread = new Thread(HandleStrokeClient);
            clientThread.IsBackground = true;
            clientThread.Start();
        }
    }

    private void HandleJointClient()
    {

        try
        {
            NetworkStream stream = jointClient.GetStream();
            byte[] buffer = new byte[24];
            byte[] eeBuffer = new byte[12];
            while (jointClient.Connected && running)
            {
                float[] jointValues = control
                    ? IK_control.joints
                    : new float[] { IK_toolkit.Joint_1, IK_toolkit.Joint_2, IK_toolkit.Joint_3, IK_toolkit.Joint_4, IK_toolkit.Joint_5, IK_toolkit.Joint_6 };

                float[] jointValuesInRadians = new float[jointValues.Length];
                for (int i = 0; i < jointValues.Length; i++)
                {
                    jointValuesInRadians[i] = jointValues[i] * Mathf.Deg2Rad;
                }

                Vector3 eePos;
                lock (eeLock)
                {
                    eePos = currentEndPointPosition;
                }
                float[] ee = new float[] { eePos.x, eePos.y, eePos.z };
                Buffer.BlockCopy(jointValuesInRadians, 0, buffer, 0, buffer.Length);
                stream.Write(buffer, 0, buffer.Length);
                Buffer.BlockCopy(ee, 0, eeBuffer, 0, eeBuffer.Length);
                stream.Write(eeBuffer, 0, eeBuffer.Length);
                stream.Flush();

                logQueue.Enqueue($"Sent joint values (radians): {string.Join(", ", jointValuesInRadians)}");
                logQueue.Enqueue($"Sending endpoint coords: {string.Join(", ", ee)}");
                Thread.Sleep(5);
            }
        }
        catch (Exception ex)
        {
            logQueue.Enqueue($"Client connection lost: {ex.Message}");
        }
        finally
        {
            jointClient?.Close();
            logQueue.Enqueue("Client disconnected.");
        }
    }

    void HandleStrokeClient()
    {
        try
        {
            NetworkStream stream = strokeClient.GetStream();
            byte[] thetaBuffer = new byte[4];
            byte[] strokeBuffer = new byte[4];

            while (strokeClient.Connected && running)
            {
                float stroke = GripperControl.Instance.Stroke;
                float theta = GripperControl.Instance.Theta;

                Buffer.BlockCopy(new float[] { stroke }, 0, strokeBuffer, 0, 4);
                stream.Write(strokeBuffer, 0, 4);
                //Debug.Log($"Sent Stroke: {stroke}");
                /*
                                Buffer.BlockCopy(new float[] { theta }, 0, thetaBuffer, 0, 4);
                                stream.Write(thetaBuffer, 0, 4);
                                Debug.Log($"Sent Theta: {theta}");
                */

                stream.Flush();
                Thread.Sleep(5);
            }
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"Stroke client connection lost: {ex.Message}");
        }
        finally
        {
            strokeClient?.Close();
            Debug.Log("Stroke client disconnected.");
        }
    }

    public void StopServers()
    {
        running = false;

        jointListener?.Stop();
        strokeListener?.Stop();

        jointClient?.Close();
        strokeClient?.Close();

        jointThread?.Join();
        strokeThread?.Join();

        Debug.Log("Servers stopped.");
    }

    void OnApplicationQuit()
    {
        StopServers();
    }
}