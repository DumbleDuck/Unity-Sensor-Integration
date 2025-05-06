using UnityEngine;
using TMPro;

public class TemperatureDisplay : MonoBehaviour
{
    public float temperature = 20.0f; // Default temperature value
    private TextMeshPro textMeshPro;

    void Start()
    {
        // Get the TextMeshPro component attached to this GameObject i.e the canvas. TextMeshPro is responsible for updating the text
        textMeshPro = GetComponent<TextMeshPro>();

        // Initialize the display with the default temperature
        UpdateTemperatureDisplay();
    }

    void Update()
    {
        var mqttReceiver = MQTTReceiver.Instance;
        if (mqttReceiver != null)
        {
            // Get the latest encoder value
            temperature = mqttReceiver.GetTemp();

            //Logging
            //Debug.Log($"Encoder Value: {encoderValue}, Button Pressed: {buttonPressed}, Acc_X: {accX}, Acc_Y: {accZ}, Acc_Z: {accZ}"); 
        }
        else
        {
            Debug.LogError("MQTTReceiver instance not found.");
        }
        // For demonstration, you can update the temperature here
        // For now, it just keeps showing the default value
        // Students can change this logic to update the temperature from the ESP32
        UpdateTemperatureDisplay();
    }

    public void UpdateTemperatureDisplay()
    {
        // Check if the TextMeshPro component is assigned before updating the text
        if (textMeshPro != null)
        {
            // Update the text with the current temperature value
            textMeshPro.text = temperature.ToString("F1") + " °C";
        }
    }
}
