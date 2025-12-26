# Sensor Simulation & Validation

This section covers simulating various sensors (LiDAR, depth cameras, IMU) on robots in simulation environments. Sensor simulation is crucial for developing and testing perception algorithms without requiring physical hardware, allowing students to experiment safely and cost-effectively while producing data that closely matches real-world sensor characteristics and noise patterns.

## Learning Objectives

After completing this section, you will be able to:
- Configure and simulate LiDAR sensors with realistic characteristics
- Set up depth camera simulation with appropriate noise models
- Implement IMU sensor simulation with accurate measurements
- Validate simulated sensor data against real-world characteristics
- Create sensor fusion scenarios combining multiple sensor types
- Design and execute sensor validation experiments

## LiDAR Simulation

### Gazebo LiDAR Configuration

LiDAR sensors in Gazebo are configured through SDF files with specific parameters:

```xml
<!-- Example LiDAR sensor configuration in SDF -->
<model name="lidar_sensor">
  <link name="lidar_link">
    <sensor name="lidar" type="ray">
      <ray>
        <scan>
          <horizontal>
            <samples>1080</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>  <!-- -π radians -->
            <max_angle>3.14159</max_angle>   <!-- π radians -->
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>lidar</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
      </plugin>
      <always_on>true</always_on>
      <update_rate>10</update_rate>
    </sensor>
  </link>
</model>
```

### LiDAR Parameters and Characteristics

Key parameters that affect LiDAR simulation quality:

- **Range**: Minimum and maximum detection distance
- **Resolution**: Angular resolution of the sensor
- **Update Rate**: How frequently the sensor publishes data
- **Noise Model**: Realistic noise characteristics
- **Ray Count**: Number of rays in horizontal/vertical directions

### ROS 2 LiDAR Integration

LiDAR data is typically published as `sensor_msgs/LaserScan` messages:

```python
# Example Python code for processing LiDAR data
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar/scan',
            self.lidar_callback,
            10)
        self.subscription  # prevent unused variable warning

    def lidar_callback(self, msg):
        # Convert to numpy array for processing
        ranges = np.array(msg.ranges)

        # Filter out invalid ranges
        valid_ranges = ranges[(ranges >= msg.range_min) & (ranges <= msg.range_max)]

        # Calculate statistics
        if len(valid_ranges) > 0:
            avg_distance = np.mean(valid_ranges)
            min_distance = np.min(valid_ranges)

            self.get_logger().info(f'Avg: {avg_distance:.2f}m, Min: {min_distance:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    lidar_processor = LidarProcessor()
    rclpy.spin(lidar_processor)
    lidar_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Depth Camera Simulation

### Depth Camera Configuration in Gazebo

Depth cameras in Gazebo provide RGB, depth, and point cloud data:

```xml
<!-- Example depth camera configuration -->
<model name="depth_camera">
  <link name="camera_link">
    <sensor name="depth_camera" type="depth">
      <camera>
        <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees in radians -->
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>camera</namespace>
        </ros>
        <frame_name>camera_link</frame_name>
        <min_depth>0.1</min_depth>
        <max_depth>10.0</max_depth>
      </plugin>
      <always_on>true</always_on>
      <update_rate>30</update_rate>
    </sensor>
  </link>
</model>
```

### Depth Camera Topics and Data

Depth cameras typically publish multiple topics:

- `/camera/image_raw`: RGB image data
- `/camera/depth/image_raw`: Depth image data
- `/camera/points`: Point cloud data
- `/camera/camera_info`: Camera calibration parameters

### Processing Depth Camera Data

```python
# Example depth camera data processing
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthCameraProcessor(Node):
    def __init__(self):
        super().__init__('depth_camera_processor')
        self.bridge = CvBridge()

        # Subscribe to camera topics
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10)

    def image_callback(self, msg):
        # Convert ROS image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Process the image (e.g., object detection)
        processed_image = self.process_image(cv_image)

        # Display or further process
        cv2.imshow("Camera Image", processed_image)
        cv2.waitKey(1)

    def depth_callback(self, msg):
        # Convert depth image to numpy array
        depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")

        # Calculate depth statistics
        valid_depths = depth_image[depth_image > 0]
        if len(valid_depths) > 0:
            avg_depth = np.mean(valid_depths)
            self.get_logger().info(f'Average depth: {avg_depth:.2f}m')

    def process_image(self, image):
        # Example: Simple edge detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        return cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

def main(args=None):
    rclpy.init(args=args)
    processor = DepthCameraProcessor()
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## IMU Simulation

### IMU Sensor Configuration

IMU sensors in Gazebo provide orientation, angular velocity, and linear acceleration:

```xml
<!-- Example IMU sensor configuration -->
<model name="imu_sensor">
  <link name="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
        <ros>
          <namespace>imu</namespace>
        </ros>
        <frame_name>imu_link</frame_name>
      </plugin>
    </sensor>
  </link>
</model>
```

### IMU Data Processing

IMU data is published as `sensor_msgs/Imu` messages containing orientation, angular velocity, and linear acceleration:

```python
# Example IMU data processing
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np
from scipy.spatial.transform import Rotation as R

class ImuProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)

        # Initialize variables for filtering
        self.orientation_history = []
        self.angular_velocity_history = []
        self.linear_acceleration_history = []

    def imu_callback(self, msg):
        # Extract orientation (quaternion)
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        rotation = R.from_quat(orientation_list)

        # Convert to Euler angles for easier interpretation
        euler_angles = rotation.as_euler('xyz', degrees=True)

        # Extract angular velocity
        angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        # Extract linear acceleration
        linear_acceleration = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # Calculate magnitude of acceleration (excluding gravity)
        acceleration_magnitude = np.linalg.norm(linear_acceleration)

        self.get_logger().info(
            f'Roll: {euler_angles[0]:.2f}°, '
            f'Pitch: {euler_angles[1]:.2f}°, '
            f'Yaw: {euler_angles[2]:.2f}°, '
            f'Accel Mag: {acceleration_magnitude:.2f} m/s²'
        )

def main(args=None):
    rclpy.init(args=args)
    imu_processor = ImuProcessor()
    rclpy.spin(imu_processor)
    imu_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Sensor Fusion Techniques

### Combining Multiple Sensors

Sensor fusion combines data from multiple sensors to improve accuracy and reliability:

```python
# Example sensor fusion node
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import TransformListener, Buffer
import numpy as np

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Initialize sensor data storage
        self.lidar_data = None
        self.imu_data = None
        self.odom_data = None

        # Create subscribers for different sensors
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        # Publisher for fused pose
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/fused_pose', 10)

        # Timer for fusion processing
        self.timer = self.create_timer(0.033, self.fusion_callback)  # ~30Hz

    def lidar_callback(self, msg):
        self.lidar_data = msg

    def imu_callback(self, msg):
        self.imu_data = msg

    def fusion_callback(self):
        if self.lidar_data is not None and self.imu_data is not None:
            # Perform sensor fusion (simplified example)
            fused_pose = self.perform_fusion()
            self.pose_pub.publish(fused_pose)

    def perform_fusion(self):
        # Simplified sensor fusion algorithm
        # In practice, this would use more sophisticated methods like EKF
        fused_pose = PoseWithCovarianceStamped()
        fused_pose.header.stamp = self.get_clock().now().to_msg()
        fused_pose.header.frame_id = 'map'

        # Use IMU for orientation
        fused_pose.pose.pose.orientation = self.imu_data.orientation

        # For position, you might combine odometry and LiDAR-based localization
        # This is a simplified placeholder

        return fused_pose

def main(args=None):
    rclpy.init(args=args)
    fusion_node = SensorFusionNode()
    rclpy.spin(fusion_node)
    fusion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Sensor Validation Techniques

### Comparing Simulated vs Real Data

To validate sensor simulation quality, compare simulated data with real-world characteristics:

```python
# Example sensor validation script
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats

def validate_lidar_simulation(simulated_ranges, real_ranges, tolerance=0.1):
    """
    Validate LiDAR simulation by comparing with real data
    """
    # Remove invalid ranges
    sim_valid = simulated_ranges[(simulated_ranges > 0) & (simulated_ranges < 30)]
    real_valid = real_ranges[(real_ranges > 0) & (real_ranges < 30)]

    # Statistical comparison
    sim_mean = np.mean(sim_valid)
    real_mean = np.mean(real_valid)

    sim_std = np.std(sim_valid)
    real_std = np.std(real_valid)

    print(f"Simulated - Mean: {sim_mean:.3f}, Std: {sim_std:.3f}")
    print(f"Real - Mean: {real_mean:.3f}, Std: {real_std:.3f}")

    # Check if means are within tolerance
    mean_diff = abs(sim_mean - real_mean)
    std_diff = abs(sim_std - real_std)

    is_valid = (mean_diff <= tolerance) and (std_diff <= tolerance)

    print(f"Mean difference: {mean_diff:.3f}, Std difference: {std_diff:.3f}")
    print(f"Validation result: {'PASS' if is_valid else 'FAIL'}")

    return is_valid

def plot_sensor_comparison(simulated_data, real_data, title="Sensor Validation"):
    """
    Plot comparison between simulated and real sensor data
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # Histogram comparison
    ax1.hist(simulated_data, bins=50, alpha=0.5, label='Simulated', density=True)
    ax1.hist(real_data, bins=50, alpha=0.5, label='Real', density=True)
    ax1.set_title(f'{title} - Histogram Comparison')
    ax1.legend()
    ax1.grid(True)

    # Scatter plot
    min_len = min(len(simulated_data), len(real_data))
    ax2.scatter(simulated_data[:min_len], real_data[:min_len], alpha=0.5)
    ax2.plot([0, max(simulated_data[:min_len])], [0, max(simulated_data[:min_len])], 'r--', label='Perfect Match')
    ax2.set_xlabel('Simulated')
    ax2.set_ylabel('Real')
    ax2.set_title(f'{title} - Scatter Plot')
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()

# Example usage (with dummy data)
simulated_lidar = np.random.normal(5.0, 0.5, 1000)  # Simulated LiDAR ranges
real_lidar = np.random.normal(5.1, 0.48, 1000)      # Real LiDAR ranges (slightly different)

validate_lidar_simulation(simulated_lidar, real_lidar)
```

### Performance Metrics

Key metrics for sensor validation:

1. **Accuracy**: How close simulated values are to expected real values
2. **Precision**: Consistency of simulated measurements
3. **Noise Characteristics**: Statistical properties of sensor noise
4. **Latency**: Time delay between physical change and sensor response
5. **Resolution**: Smallest detectable change in measurement

## Hands-on Exercises

### Exercise 1: LiDAR Configuration and Testing

**Objective**: Configure a LiDAR sensor in Gazebo and validate its performance.

**Steps**:
1. Create a LiDAR sensor configuration file with custom parameters
2. Launch Gazebo with your robot and LiDAR sensor
3. Collect LiDAR data in different environments
4. Analyze the data for range accuracy and angular resolution
5. Compare with theoretical specifications

### Exercise 2: Depth Camera Integration

**Objective**: Set up a depth camera and process its data for obstacle detection.

**Steps**:
1. Configure a depth camera in your robot model
2. Subscribe to camera topics in ROS 2
3. Implement a simple obstacle detection algorithm
4. Visualize detected obstacles in RViz
5. Test with different lighting conditions

### Exercise 3: IMU Data Analysis

**Objective**: Analyze IMU data for robot orientation and motion.

**Steps**:
1. Configure IMU sensor with appropriate noise models
2. Collect IMU data during robot movement
3. Convert quaternion orientation to Euler angles
4. Implement simple filtering to reduce noise
5. Compare IMU-based positioning with ground truth

### Exercise 4: Sensor Fusion Challenge

**Objective**: Combine data from multiple sensors to improve pose estimation.

**Steps**:
1. Integrate LiDAR, IMU, and odometry data
2. Implement a simple fusion algorithm (e.g., weighted average)
3. Compare fused pose with individual sensor estimates
4. Evaluate improvement in accuracy and robustness
5. Test with sensor failures to verify robustness

## Common Issues and Troubleshooting

### Issue: LiDAR Range Inconsistencies
- **Symptom**: LiDAR readings don't match expected distances
- **Solution**: Check coordinate frames, verify range parameters, validate environment

### Issue: Depth Camera Artifacts
- **Symptom**: Depth images show unrealistic values or artifacts
- **Solution**: Verify camera calibration, check for reflective surfaces, adjust clipping planes

### Issue: IMU Drift
- **Symptom**: Orientation estimates drift over time
- **Solution**: Implement sensor fusion with other sources, apply calibration, check noise models

### Issue: Sensor Timing Issues
- **Symptom**: Sensor data arrives at unexpected times or rates
- **Solution**: Check update rates in configuration, verify ROS time synchronization, use message filters