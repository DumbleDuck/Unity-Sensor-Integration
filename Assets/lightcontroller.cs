using UnityEngine;

public class SensorLightController : MonoBehaviour
{
    public GameObject[] lightsToTurnOff;  // Array of lights to turn off
    public GameObject pointLight;         // The Point Light to turn on
    public AudioClip musicClip;           // The music file to play
    private AudioSource audioSource;      // The AudioSource component to play the music
    private bool isSensorActivated = false;  // Tracks whether the "sensor" is activated

    
    int lastTouchFlagState = 0; // To store the previous state of the touch flag
    void Start()
    {
        // Get or add the AudioSource component to the GameObject
        audioSource = GetComponent<AudioSource>();

        // If no AudioSource component exists, add one automatically
        if (audioSource == null)
        {
            audioSource = gameObject.AddComponent<AudioSource>();
        }

        // Set the music clip for the AudioSource
        audioSource.clip = musicClip;
        audioSource.loop = true;  // Set it to loop if desired
    }

    void Update()
    {
        var mqttReceiver = MQTTReceiver.Instance;
        // Check for "T" key press or touch sensor flag
        int currentTouchFlagState = mqttReceiver.GetTouchFlag(); 

        if (Input.GetKeyDown(KeyCode.T) || (currentTouchFlagState == 1 && lastTouchFlagState == 0)) 
        { // Toggle the sensor state
            isSensorActivated = !isSensorActivated; 
            
            // Apply the light changes based on the new
            UpdateLights(); 
        } 
        
        // Update the last touch flag state
        lastTouchFlagState = currentTouchFlagState;
    }

    // Updates the lights based on the sensor state
    private void UpdateLights()
    {
        if (isSensorActivated)
        {
            // Turn off all lights except the Point Light
            foreach (GameObject light in lightsToTurnOff)
            {
                light.SetActive(false);
            }

            // Turn on the Point Light
            pointLight.SetActive(true);

            // Play the music if it's not already playing
            if (!audioSource.isPlaying)
            {
                audioSource.Play();
            }
        }
        else
        {
            // Turn on all lights except the Point Light
            foreach (GameObject light in lightsToTurnOff)
            {
                light.SetActive(true);
            }

            // Turn off the Point Light
            pointLight.SetActive(false);

            // Stop the music if it is playing
            if (audioSource.isPlaying)
            {
                audioSource.Stop();
            }
        }
    }
}