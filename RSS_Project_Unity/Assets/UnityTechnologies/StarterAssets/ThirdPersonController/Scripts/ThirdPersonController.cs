using System.IO.Ports;
using UnityEngine;
using UnityEngine.Networking;

#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace StarterAssets
{
    public class ThirdPersonController : MonoBehaviour
    {
        [Header("MPU6050 Integration")]
        public string SerialPortName = "COM9";  // Adjust to your system's port
        public int BaudRate = 9600;

        private SerialPort _serialPort;
        
        // To handle HTTP requests
        private string _serverUrl = "http://<UNITY_SERVER_IP>:<PORT>/sensor_data"; // Replace with correct IP and port

        private void Start()
        {
            _serialPort = new SerialPort(SerialPortName, BaudRate);
            if (!_serialPort.IsOpen)
            {
                _serialPort.Open();
            }
            StartCoroutine(ListenForSensorData());
        }

        private void Update()
        {
            if (_serialPort.IsOpen)
            {
                try
                {
                    // Read data from serial port if available
                    if (_serialPort.BytesToRead > 0)
                    {
                        string data = _serialPort.ReadLine(); // Read line from serial
                        SensorData sensorData = JsonUtility.FromJson<SensorData>(data); // Parse sensor data
                        ProcessAccelerometerData(sensorData); // Process the accelerometer data
                    }
                }
                catch (System.Exception e)
                {
                    Debug.LogError("Error reading serial data: " + e.Message);
                }
            }
        }

        // Coroutine to listen for HTTP requests
        private IEnumerator ListenForSensorData()
        {
            while (true)
            {
                UnityWebRequest request = UnityWebRequest.Get(_serverUrl); // Send a GET request to receive data

                yield return request.SendWebRequest();

                if (request.isNetworkError || request.isHttpError)
                {
                    Debug.Log("Error: " + request.error);
                }
                else
                {
                    // Parse the JSON response
                    string jsonResponse = request.downloadHandler.text;
                    SensorData sensorData = JsonUtility.FromJson<SensorData>(jsonResponse);
                    ProcessAccelerometerData(sensorData);
                }

                yield return new WaitForSeconds(0.1f); // Poll every 100ms
            }
        }

        private void ProcessAccelerometerData(SensorData data)
        {
            // Implement your logic to update character movement
        }

        // Cleanup serial port when the game ends
        private void OnApplicationQuit()
        {
            if (_serialPort.IsOpen)
            {
                _serialPort.Close();
            }
        }
    }

    // Data class for sensor data
    [System.Serializable]
    public class SensorData
    {
        public float accel_x;
        public float accel_y;
        public float accel_z;
        public float gyro_x;
        public float gyro_y;
        public float gyro_z;
    }
}
