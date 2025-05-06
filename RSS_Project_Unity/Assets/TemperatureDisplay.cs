using UnityEngine;
using TMPro;

public class TemperatureDisplay : MonoBehaviour
{
    public float temperature = 10.0f; // Default temperature value
    private TextMeshPro textMeshPro;

    void Start()
    {
        // Get the TextMeshPro component attached to this GameObject
        textMeshPro = GetComponent<TextMeshPro>();

        // Initialize the display with the default temperature
        UpdateTemperatureDisplay();
    }

    void Update()
    {
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
