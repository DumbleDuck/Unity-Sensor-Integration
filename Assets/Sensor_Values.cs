using UnityEngine;
using System;
using uPLibrary.Networking.M2Mqtt;
using uPLibrary.Networking.M2Mqtt.Messages;
using System.Text;
using System.Collections.Generic;

public class MQTTReceiver : MonoBehaviour
{
    public static MQTTReceiver Instance { get; private set; } // Singleton Instance: to make this class accessible to other scripts

    private MqttClient client;
    public string brokerAddress = "192.168.25.10";
    public int brokerPort = 1883;
    public string topic = "encoder/reading";

    private static readonly Queue<Action> _executionQueue = new Queue<Action>();

    private int encoderValue; 
    private bool buttonPressed;
    private float pitch;
    private float roll;
    private float temperature;
    private int ir;
    private bool fl;
    private int touchFlag;

    void Awake()
    {
        // Singleton setup
        if (Instance == null)
        {
            Instance = this;
            DontDestroyOnLoad(gameObject); // Persist across scenes
        }
        else
        {
            Destroy(gameObject); // Prevent duplicates
        }
    }

    //MQTT connection
    void Start()
    {
        client = new MqttClient(brokerAddress, brokerPort, false, null, null, MqttSslProtocols.None);  //New instance of MQTTClient class
        client.MqttMsgPublishReceived += Client_MqttMsgPublishReceived;  //Method Client_MqttMsgPublishReceived gets triggered whenever a new message is received  
        client.Connect(Guid.NewGuid().ToString());  //connects to new client and assigns it a unique ID

        if (client.IsConnected)
        {
            client.Subscribe(new string[] { topic }, new byte[] { MqttMsgBase.QOS_LEVEL_AT_LEAST_ONCE });  //If client connected, subscribes to the require topic
            Debug.Log("Subscribed to topic: " + topic);
        }
        else
        {
            Debug.LogError("MQTT connection failed.");
        }
    }
    
    //Storing and displaying values on console 
    private void Client_MqttMsgPublishReceived(object sender, MqttMsgPublishEventArgs e)
    {
        string message = Encoding.UTF8.GetString(e.Message);
        //Debug.Log("Message received from MQTT: " + message);

        // Parse JSON message
        var data = JsonUtility.FromJson<MqttData>(message);

        //Storing values
        encoderValue = data.encoderValue;
        buttonPressed = data.buttonPressed;
        pitch = data.Pitch;
        roll = data.Roll;
        temperature = data.Temp;
        ir = data.IR;
        fl = data.Flag;
        touchFlag = data.TouchFlag;

        print("Touch flag: " + touchFlag);


        // Log the received values
        //Debug.Log($"Encoder Value: {data.encoderValue}, Button Pressed: {data.buttonPressed}, Pitch: {data.Pitch}, Roll: {data.Roll}");

        // Just log the value to the console
        //Enqueue(() => PrintValues(data.encoderValue, data.buttonPressed, data.Pitch, data.Roll)); //Adds PrintValue action to the queue for execution in main thread. logging can only be performed in main thread.

    }

    void Update()
    {
        lock (_executionQueue)   // Locks the queue till all things have been executed. Prevent other threads from modifying it and prevent data corruption.
        {
            while (_executionQueue.Count > 0)       
            {
                _executionQueue.Dequeue().Invoke();  // Executes whatever is in the first priority in the queue
            }
        }
    }

    private void Enqueue(Action action) //The method that takes an action as argument and queues it (called in Client_Mqtt... method)
    {
        lock (_executionQueue)
        {
            _executionQueue.Enqueue(action);  
        }
    }

    void OnDestroy()
    {
        if (client != null && client.IsConnected)
        {
            client.Disconnect();
        }
    }

    void PrintValues(int encoderValue, bool buttonPressed, float pitch, float roll)
    {
        Debug.Log($"Encoded Value: {encoderValue}, Button Pressed: {buttonPressed}, Pitch: {pitch}, Roll: {roll}");
    }

    public int GetEncoderValue() 
    { 
        return encoderValue; 
    }
    public bool GetButtonPressed() 
    { 
        return buttonPressed; 
    }

    public (float,float) GetPR()
    {
        return (pitch, roll);
    }

    public float GetTemp()
    {
        return temperature;
    }

    public int GetIR()
    {
        return ir;
    }

    public bool GetFlag()
    {
        return fl;
    }

    public int GetTouchFlag()
    {
        return touchFlag;
    }

    [Serializable]
    public class MqttData
    {
        public int encoderValue;
        public bool buttonPressed;
        public float Pitch;
        public float Roll;
        public float Temp;
        public int IR;
        public bool Flag;
        public int TouchFlag;
    }
}
