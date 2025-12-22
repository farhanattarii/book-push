# Digital Twins & HRI in Unity

This section covers creating high-fidelity 3D visualization and human-robot interaction (HRI) studies using Unity. Unity provides photorealistic rendering capabilities that complement the physics simulation from Gazebo, enabling students to study spatial relationships, visual perception, and human-robot interaction patterns in a visually rich environment.

## Learning Objectives

After completing this section, you will be able to:
- Set up Unity for robotics visualization and simulation
- Create high-fidelity digital twin representations of robots
- Implement human-robot interaction scenarios in Unity
- Synchronize Unity visualization with Gazebo physics simulation
- Design and evaluate HRI studies using Unity environments
- Implement sensor visualization and overlay techniques

## Installing Unity

### Option 1: Unity Hub (Recommended)

Unity Hub is the recommended way to manage Unity installations:

1. Download Unity Hub from the [Unity website](https://unity.com/download)
2. Install Unity Hub and create a Unity account (free)
3. Use Unity Hub to install Unity 2022.3 LTS or later
4. Install the Universal Render Pipeline (URP) package for advanced rendering

### Option 2: Unity Personal Edition

For students and educational use, Unity offers a free Personal Edition:

```bash
# Visit https://unity.com/download to download Unity Personal
# Follow the installation wizard for your operating system
```

### Required Packages and Dependencies

Install these essential packages for robotics visualization:

1. **Universal Render Pipeline (URP)**: For high-quality rendering
2. **ProBuilder**: For rapid environment prototyping
3. **Cinemachine**: For advanced camera systems
4. **Unity Robotics Hub**: Contains robotics-specific tools and samples

## Unity Robotics Setup

### Unity Robotics Package

Unity provides the Unity Robotics package for integrating with ROS 2:

```bash
# In Unity Package Manager:
# Window → Package Manager → Add package from git URL
# Add: com.unity.robotics.ros-tcp-connector
```

### ROS 2 Communication Bridge

The Unity ROS TCP Connector enables communication between Unity and ROS 2:

```csharp
// Example Unity C# script for ROS communication
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class RobotController : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<UInt64Msg>("/unity_robot_ticks");
    }

    void Update()
    {
        // Send robot tick to ROS
        ros.Publish("/unity_robot_ticks", new UInt64Msg((ulong)Time.frameCount));
    }
}
```

## Creating Digital Twin Representations

### Importing Robot Models

To create accurate digital twins, you'll need to import your robot models into Unity:

1. **URDF Import**: Use the Unity URDF Importer package
2. **FBX/Obj Models**: Import visual meshes from CAD models
3. **Joint Configuration**: Map Unity joints to robot kinematics

```csharp
// Example: Loading robot configuration
using UnityEngine;
using System.Collections.Generic;

[System.Serializable]
public class RobotJoint
{
    public string name;
    public Transform jointTransform;
    public Joint jointComponent;
    public float minAngle;
    public float maxAngle;
}

public class RobotDigitalTwin : MonoBehaviour
{
    public List<RobotJoint> joints = new List<RobotJoint>();

    public void SetJointPositions(Dictionary<string, float> jointPositions)
    {
        foreach (var joint in joints)
        {
            if (jointPositions.ContainsKey(joint.name))
            {
                // Update joint position in Unity
                JointMotor motor = joint.jointComponent.motor;
                motor.targetVelocity = jointPositions[joint.name];
                joint.jointComponent.motor = motor;
            }
        }
    }
}
```

### Visual Fidelity Enhancement

For high-fidelity visualization, implement these techniques:

1. **PBR Materials**: Use Physically Based Rendering for realistic surfaces
2. **Lighting Setup**: Configure realistic lighting conditions
3. **Post-Processing**: Add bloom, ambient occlusion, and color grading
4. **LOD Systems**: Implement Level of Detail for performance

```csharp
// Example: Material configuration for robot parts
using UnityEngine;

public class RobotMaterialManager : MonoBehaviour
{
    [Header("Material Settings")]
    public Material robotBodyMaterial;
    public Material sensorMaterial;
    public Material jointMaterial;

    [Header("Visual Effects")]
    public bool enableSpecularHighlights = true;
    public bool enableReflections = true;

    void Start()
    {
        ConfigureMaterials();
    }

    void ConfigureMaterials()
    {
        // Configure robot body material
        robotBodyMaterial.SetFloat("_Metallic", 0.7f);
        robotBodyMaterial.SetFloat("_Smoothness", 0.8f);

        // Configure sensor material (for visualization)
        sensorMaterial.SetColor("_EmissionColor", Color.blue);
        sensorMaterial.EnableKeyword("_EMISSION");
    }
}
```

## Human-Robot Interaction (HRI) Scenarios

### Interaction Design Principles

When designing HRI scenarios in Unity, consider these principles:

1. **Natural Interaction**: Use intuitive gestures and movements
2. **Feedback Systems**: Provide visual, auditory, and haptic feedback
3. **Safety Boundaries**: Implement safe interaction zones
4. **User Comfort**: Ensure comfortable interaction distances

### VR/AR Integration for HRI

Unity supports VR and AR platforms for immersive HRI studies:

```csharp
// Example: VR interaction handler
using UnityEngine;
using UnityEngine.XR;

public class VRInteractionHandler : MonoBehaviour
{
    [Header("VR Interaction Settings")]
    public Transform leftController;
    public Transform rightController;
    public LayerMask interactionLayer;

    void Update()
    {
        HandleVRInteraction();
    }

    void HandleVRInteraction()
    {
        // Check for controller collisions with interactable objects
        if (leftController != null && rightController != null)
        {
            // Implement interaction logic
            CheckControllerInteractions(leftController);
            CheckControllerInteractions(rightController);
        }
    }

    void CheckControllerInteractions(Transform controller)
    {
        Collider[] hits = Physics.OverlapSphere(controller.position, 0.1f, interactionLayer);
        foreach (Collider hit in hits)
        {
            IInteractable interactable = hit.GetComponent<IInteractable>();
            if (interactable != null)
            {
                interactable.OnHover(controller);

                // Check for trigger press
                if (IsTriggerPressed(controller))
                {
                    interactable.OnInteract(controller);
                }
            }
        }
    }

    bool IsTriggerPressed(Transform controller)
    {
        // Check VR input for trigger press
        return Input.GetAxis("Trigger_" + controller.name) > 0.5f;
    }
}
```

### Social Interaction Protocols

Implement social interaction protocols for studying human-robot social dynamics:

```csharp
// Example: Social interaction manager
using UnityEngine;
using System.Collections;

public enum SocialBehavior
{
    Approach,
    Greet,
    Follow,
    Avoid,
    Wait
}

public class SocialInteractionManager : MonoBehaviour
{
    [Header("Social Parameters")]
    public float personalSpaceRadius = 1.0f;
    public float socialSpaceRadius = 2.0f;
    public float publicSpaceRadius = 4.0f;

    [Header("Behavior Settings")]
    public SocialBehavior currentBehavior = SocialBehavior.Wait;

    public void UpdateSocialBehavior(Transform human)
    {
        float distance = Vector3.Distance(transform.position, human.position);

        if (distance < personalSpaceRadius)
        {
            // Too close - trigger avoidance
            SetBehavior(SocialBehavior.Avoid);
        }
        else if (distance < socialSpaceRadius)
        {
            // Social distance - can interact
            SetBehavior(SocialBehavior.Greet);
        }
        else if (distance < publicSpaceRadius)
        {
            // Public distance - can approach
            SetBehavior(SocialBehavior.Approach);
        }
        else
        {
            // Out of range - wait
            SetBehavior(SocialBehavior.Wait);
        }
    }

    void SetBehavior(SocialBehavior behavior)
    {
        if (currentBehavior != behavior)
        {
            currentBehavior = behavior;
            OnBehaviorChange(behavior);
        }
    }

    void OnBehaviorChange(SocialBehavior behavior)
    {
        // Implement behavior-specific actions
        switch (behavior)
        {
            case SocialBehavior.Approach:
                StartCoroutine(ApproachBehavior());
                break;
            case SocialBehavior.Greet:
                StartCoroutine(GreetBehavior());
                break;
            case SocialBehavior.Avoid:
                StartCoroutine(AvoidBehavior());
                break;
            case SocialBehavior.Wait:
                StopAllCoroutines();
                break;
        }
    }

    IEnumerator ApproachBehavior()
    {
        // Implement approach logic
        yield return null;
    }

    IEnumerator GreetBehavior()
    {
        // Implement greeting logic
        yield return null;
    }

    IEnumerator AvoidBehavior()
    {
        // Implement avoidance logic
        yield return null;
    }
}
```

## Synchronization with Physics Simulation

### Real-time Data Synchronization

To maintain synchronization between Gazebo physics and Unity visualization:

```csharp
// Example: Synchronization manager
using UnityEngine;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class SimulationSynchronizer : MonoBehaviour
{
    [Header("Synchronization Settings")]
    public float syncInterval = 0.033f; // ~30 FPS
    public float maxSyncDelay = 0.1f;   // 100ms max delay

    private ROSConnection ros;
    private Dictionary<string, Transform> robotParts = new Dictionary<string, Transform>();

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<OdometryMsg>("/robot_odometry", OnOdometryReceived);

        // Initialize robot parts mapping
        InitializeRobotParts();

        // Start synchronization loop
        InvokeRepeating("SendUnityState", syncInterval, syncInterval);
    }

    void InitializeRobotParts()
    {
        // Map robot parts from Unity hierarchy
        Transform[] allChildren = GetComponentsInChildren<Transform>();
        foreach (Transform child in allChildren)
        {
            if (child.name.StartsWith("joint_") || child.name.StartsWith("link_"))
            {
                robotParts[child.name] = child;
            }
        }
    }

    void OnOdometryReceived(OdometryMsg odometry)
    {
        // Update Unity representation based on Gazebo data
        Vector3 position = new Vector3(
            (float)odometry.pose.pose.position.x,
            (float)odometry.pose.pose.position.z, // Note: Unity Y-up vs ROS Z-up
            (float)odometry.pose.pose.position.y
        );

        Quaternion rotation = new Quaternion(
            (float)odometry.pose.pose.orientation.x,
            (float)odometry.pose.pose.orientation.z,
            (float)odometry.pose.pose.orientation.y,
            (float)odometry.pose.pose.orientation.w
        );

        transform.position = position;
        transform.rotation = rotation;
    }

    void SendUnityState()
    {
        // Send Unity state back to ROS for visualization
        OdometryMsg unityState = new OdometryMsg();
        unityState.header.stamp = new TimeStamp();
        unityState.header.frame_id = "unity_world";

        unityState.pose.pose.position = new PointMsg(
            transform.position.x,
            transform.position.z,
            transform.position.y
        );

        unityState.pose.pose.orientation = new QuaternionMsg(
            transform.rotation.x,
            transform.rotation.z,
            transform.rotation.y,
            transform.rotation.w
        );

        ros.Publish("/unity_state", unityState);
    }
}
```

### Latency Management

Implement techniques to minimize synchronization latency:

1. **Interpolation**: Smooth transitions between received states
2. **Prediction**: Predict future states based on current motion
3. **Buffer Management**: Optimize data buffer sizes
4. **Network Optimization**: Use efficient message formats

## Sensor Visualization and Overlay

### LiDAR Visualization

Create visualizations for LiDAR sensor data in Unity:

```csharp
// Example: LiDAR point cloud visualization
using UnityEngine;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class LidarVisualizer : MonoBehaviour
{
    [Header("LiDAR Settings")]
    public GameObject pointPrefab;
    public int maxPoints = 10000;
    public float pointSize = 0.01f;

    private List<GameObject> pointObjects = new List<GameObject>();
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<LaserScanMsg>("/laser_scan", OnLaserScanReceived);

        // Initialize point objects
        InitializePointObjects();
    }

    void InitializePointObjects()
    {
        for (int i = 0; i < maxPoints; i++)
        {
            GameObject point = Instantiate(pointPrefab);
            point.SetActive(false);
            pointObjects.Add(point);
        }
    }

    void OnLaserScanReceived(LaserScanMsg scan)
    {
        // Process LiDAR scan data
        for (int i = 0; i < Mathf.Min(scan.ranges.Length, maxPoints); i++)
        {
            float range = scan.ranges[i];
            if (range >= scan.range_min && range <= scan.range_max)
            {
                float angle = scan.angle_min + i * scan.angle_increment;

                Vector3 pointPos = new Vector3(
                    range * Mathf.Cos(angle),
                    0,
                    range * Mathf.Sin(angle)
                );

                if (i < pointObjects.Count)
                {
                    GameObject point = pointObjects[i];
                    point.transform.position = transform.TransformPoint(pointPos);
                    point.SetActive(true);
                }
            }
            else if (i < pointObjects.Count)
            {
                pointObjects[i].SetActive(false);
            }
        }
    }
}
```

### Camera Overlay Systems

Implement camera overlay systems for sensor data visualization:

```csharp
// Example: Camera overlay manager
using UnityEngine;
using UnityEngine.UI;

public class CameraOverlayManager : MonoBehaviour
{
    [Header("Overlay Settings")]
    public RawImage overlayImage;
    public bool showDepthOverlay = true;
    public bool showLidarOverlay = true;
    public Color overlayColor = Color.red;

    [Header("Sensor Data")]
    public Texture2D depthTexture;
    public Texture2D lidarTexture;

    void Update()
    {
        UpdateOverlays();
    }

    void UpdateOverlays()
    {
        if (showDepthOverlay && depthTexture != null)
        {
            overlayImage.texture = depthTexture;
            overlayImage.color = overlayColor;
        }
        else if (showLidarOverlay && lidarTexture != null)
        {
            overlayImage.texture = lidarTexture;
            overlayImage.color = overlayColor;
        }
        else
        {
            overlayImage.texture = null;
        }
    }

    public void ToggleDepthOverlay()
    {
        showDepthOverlay = !showDepthOverlay;
        if (showDepthOverlay) showLidarOverlay = false;
    }

    public void ToggleLidarOverlay()
    {
        showLidarOverlay = !showLidarOverlay;
        if (showLidarOverlay) showDepthOverlay = false;
    }
}
```

## Hands-on Exercises

### Exercise 1: Digital Twin Creation

**Objective**: Create a digital twin of a simple robot in Unity and synchronize it with Gazebo physics.

**Steps**:
1. Import a basic robot model into Unity
2. Set up the Unity ROS TCP connector
3. Create a script to receive joint states from Gazebo
4. Update the Unity robot model based on received states
5. Verify synchronization by moving the robot in Gazebo and observing Unity

### Exercise 2: HRI Scenario Development

**Objective**: Develop a simple HRI scenario where the robot responds to human proximity.

**Steps**:
1. Create a simple human avatar in Unity
2. Implement distance-based behavior switching
3. Add visual feedback when the robot detects the human
4. Test different interaction zones (personal, social, public)
5. Evaluate the naturalness of the interaction

### Exercise 3: Sensor Data Visualization

**Objective**: Visualize LiDAR and camera data overlaid on the Unity scene.

**Steps**:
1. Set up LiDAR point cloud visualization
2. Create camera overlay system for sensor data
3. Implement real-time updates of sensor visualization
4. Test with different environments in Gazebo
5. Evaluate the effectiveness of the visualization

## Common Issues and Troubleshooting

### Issue: Synchronization Lag
- **Symptom**: Unity visualization lags behind Gazebo physics
- **Solution**: Increase sync frequency, optimize network settings, implement interpolation

### Issue: Model Import Problems
- **Symptom**: Robot model appears distorted or joints don't align
- **Solution**: Check coordinate system conversions, verify joint limits, validate URDF

### Issue: Performance Degradation
- **Symptom**: Unity frame rate drops with complex scenes
- **Solution**: Implement LOD systems, optimize materials, reduce point cloud resolution

### Issue: ROS Communication Failures
- **Symptom**: No data exchange between Unity and ROS 2
- **Solution**: Verify ROS master connection, check topic names, validate message types