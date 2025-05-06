using UnityEngine;

public class DiscoLight : MonoBehaviour
{
    public Light pointLight; // Reference to the Point Light
    public float transitionDuration = 5f; // Time in seconds for a full color cycle

    private float timer = 0f;

    // Define the colors to transition through
    private Color[] colors = new Color[]
    {
        Color.red,    // Red
        Color.blue,   // Blue
        Color.green,  // Green
        new Color(0.5f, 0f, 0.5f), // Purple
        Color.cyan    // Cyan
    };

    void Update()
    {
        if (colors.Length < 2) return; // Ensure there are enough colors to transition through

        // Increment timer based on time passed
        timer += Time.deltaTime / transitionDuration;

        // Determine the current and next color index based on the timer
        int currentColorIndex = Mathf.FloorToInt(timer) % colors.Length;
        int nextColorIndex = (currentColorIndex + 1) % colors.Length;

        // Calculate interpolation factor (0 to 1) between the current and next colors
        float t = timer % 1f;

        // Interpolate between the current and next colors
        pointLight.color = Color.Lerp(colors[currentColorIndex], colors[nextColorIndex], t);
    }
}
