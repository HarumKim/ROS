## Activity 1.4 – Color-Based Object Detection and Behavior Control

In this project, a differential-drive robot uses a vision sensor (camera) to detect colored spheres in the environment and respond based on their color. 

### Robot Behavior Based on Detected Color:

- 🟢 **Green Sphere**: The robot moves forward toward the green sphere.
- 🔵 **Blue Sphere**: The robot rotates to the **left** until it detects a green sphere.
- 🟠 **Orange Sphere**: The robot rotates to the **right** until it detects a green sphere.
- 🟡 **Yellow Sphere**: The robot **stops** for **5 seconds**, then resumes operation.
