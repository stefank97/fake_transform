import rclpy
import rclpy.time
import math
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration
from geometry_msgs.msg import Vector3
from rclpy.parameter import Parameter


#AI generated functions for quaternion/euler conversions
def quaternion_from_euler(roll, pitch, yaw):
    cr = math.cos(roll/2); sr = math.sin(roll/2)
    cp = math.cos(pitch/2); sp = math.sin(pitch/2)
    cy = math.cos(yaw/2); sy = math.sin(yaw/2)

    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return [x, y, z, w]

def euler_from_quaternion(x, y, z, w):
    # returns roll, pitch, yaw
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw




class PoseAttackerNode(Node):
    def __init__(self):
        super().__init__('pose_attacker_node')

        self.last_callback_time = None                  #To calculate time difference for drift
        self.reference_transform = None                 #saving of amcl map-odom transform
        self.reference_pose = None                      #saving of amcl pose
        self.reference_scan = None                      #saving of last laserscan
        self.timer = None                               #Timer for publisher

        #Create and fill tf buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        #Declare parameters with default values
        self.declare_parameter("mode", "tf")
        self.declare_parameter("offset_x", 0.0)
        self.declare_parameter("offset_y", 0.0)
        self.declare_parameter("offset_z", 0.0)
        self.declare_parameter("offset_yaw_deg", 0.0)     #Yaw in degrees, for configuration in e.g. Yaml
        self.declare_parameter("rate_hz", 10.0)             #Rate of transform publisher

        #Drift parameters for shifting the robot over time per second
        self.declare_parameter("offset_drift_x", 0.0)
        self.declare_parameter("offset_drift_y", 0.0)
        self.declare_parameter("offset_drift_yaw_deg", 0.0)
        
        
    
        self.mode = self.get_parameter("mode").value

        self.offset_x = self.get_parameter("offset_x").value
        self.offset_y = self.get_parameter("offset_y").value
        self.offset_z = self.get_parameter("offset_z").value
        self.offset_yaw = math.radians(self.get_parameter("offset_yaw_deg").value)      #Convert degrees to radians for calculations (quaternions)
        self.rate_hz = self.get_parameter("rate_hz").value
        
        self.offset_drift_x = self.get_parameter("offset_drift_x").value
        self.offset_drift_y = self.get_parameter("offset_drift_y").value
        self.offset_drift_yaw = math.radians(self.get_parameter("offset_drift_yaw_deg").value)      #Convert degrees to radians for calculations (quaternions)

        self.broadcaster = TransformBroadcaster(self)
        self.real_amcl = None   #To store the last received real amcl message

        amcl_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.amcl_callback, amcl_qos) #before depth "10" instead of amcl_qos
        self.amcl_pub = self.create_publisher(PoseWithCovarianceStamped, 'amcl_pose', amcl_qos) #before depth "10" instead of amcl_qos
        
        #Rviz drift subscription
        self.drift_sub = self.create_subscription(Vector3, 'fake_transform/drift', self.drift_callback, 10)
        

    #Callback for drift updates (rviz plugin)
    def drift_callback(self, msg):
        self.offset_drift_x = msg.x
        self.offset_drift_y = msg.y
        self.offset_drift_yaw = msg.z
        self.get_logger().info(f"Updated drift to x: {msg.x}, y: {msg.y}, yaw(deg): {math.degrees(msg.z)}")

        self.set_parameters([rclpy.parameter.Parameter("offset_drift_x", rclpy.Parameter.Type.DOUBLE, msg.x)])
        self.set_parameters([rclpy.parameter.Parameter("offset_drift_y", rclpy.Parameter.Type.DOUBLE, msg.y)])
        self.set_parameters([rclpy.parameter.Parameter("offset_drift_yaw_deg", rclpy.Parameter.Type.DOUBLE, math.degrees(msg.z))])

        if msg.x == 0.0 and msg.y == 0.0 and msg.z == 0.0:
            self.offset_x = 0.0
            self.offset_y = 0.0
            self.offset_yaw = 0.0
            self.get_logger().info("Offsets reset to zero")



    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9         #Divide by 1billion to get seconds

        if current_time == 0:
            self.get_logger().warn("Clock not ready yet (time=0), skipping...")
            return

        if self.last_callback_time is not None:
            time_diff = current_time - self.last_callback_time
    
            self.offset_x += self.offset_drift_x * time_diff
            self.offset_y += self.offset_drift_y * time_diff
            self.offset_yaw += self.offset_drift_yaw * time_diff
        else:
            time_diff = 0.0        #No time difference on first call, so no drift

        self.last_callback_time = current_time


        #Publish fake tf if last tf is available
        if self.reference_transform is not None:    
            self.fake_tf_publish()
            #self.get_logger().info(f"Publishing fake transform")

        #Publish fake amcl_pose if last pose is available
        if self.reference_pose is not None:
            self.fake_amcl_publish()
            #self.get_logger().info(f"Publishing fake amcl pose")





    def fake_tf_publish(self):



        if self.reference_transform is None:
            self.get_logger().warn("No reference tf available yet, skipping.")
            return
        

        base = self.reference_transform
        self.get_logger().debug(f"Using base transform: \n{base}")

        #No offset, no drift?:
        nothing_set = (abs(self.offset_x) < 1e-9 and abs(self.offset_y) < 1e-9 and abs(self.offset_z) < 1e-9 and abs(self.offset_yaw) < 1e-9 and abs(self.offset_drift_x) < 1e-9 and abs(self.offset_drift_y) < 1e-9 and abs(self.offset_drift_yaw) < 1e-9)

        if nothing_set:
            base.header.stamp = self.get_clock().now().to_msg()
            self.broadcaster.sendTransform(base)
            return

        t = TransformStamped()
#        t.header.stamp = base.header.stamp   #Use the same timestamp as the real amcl tf, to avoid inconsistencies

        t.header.frame_id = base.header.frame_id
        t.child_frame_id = base.child_frame_id

        t.transform.translation.x = base.transform.translation.x + self.offset_x
        t.transform.translation.y = base.transform.translation.y + self.offset_y
        t.transform.translation.z = base.transform.translation.z + self.offset_z

        #Real rotation as base, and adding offsets
        quat = base.transform.rotation
        roll, pitch, yaw = euler_from_quaternion(quat.x, quat.y, quat.z, quat.w)


        #Add offset to yaw
        new_yaw = yaw + self.offset_yaw
        new_quat = quaternion_from_euler(roll, pitch, new_yaw)
        t.transform.rotation.x = new_quat[0]
        t.transform.rotation.y = new_quat[1]
        t.transform.rotation.z = new_quat[2]
        t.transform.rotation.w = new_quat[3]

        #Update timestamp to current time, to avoid warnings (TF_OLD_DATA)
        t.header.stamp = self.get_clock().now().to_msg()
        #t.header.stamp = base.header.stamp

        self.broadcaster.sendTransform(t)
        self.get_logger().debug(f"Published fake transform: \n{t}")



    def amcl_callback(self, msg):
        
        if self.timer is None:
            amcl_tf = self.save_tf(msg)

            if amcl_tf is not None:
                self.reference_transform = amcl_tf
                self.reference_pose = msg

                hz_to_sec = 1.0 / self.rate_hz
                self.timer = self.create_timer(hz_to_sec, self.timer_callback)
                self.get_logger().debug(f"Received real amcl message: \n{amcl_tf}")   
            
        else:

            amcl_tf = self.save_tf(msg)
            if amcl_tf is not None:
                self.reference_transform = amcl_tf      #fill the "real" transform once
            self.reference_pose = msg





    def fake_amcl_publish(self):

        if self.reference_pose is None:
            self.get_logger().info("No real amcl message received. Waiting...")
            return
        ref = self.reference_pose
            
        fake_msg = PoseWithCovarianceStamped()
        fake_msg.header = ref.header

        
        fake_msg.pose = ref.pose
        #Modify the real amcl pose message by the offsets
        fake_msg.pose.pose.position.x = ref.pose.pose.position.x + self.offset_x
        fake_msg.pose.pose.position.y = ref.pose.pose.position.y + self.offset_y
        fake_msg.pose.pose.position.z = ref.pose.pose.position.z + self.offset_z

        #Orientation from old amcl msg to euler, add offset, back to quaternion
        quat = ref.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(quat.x, quat.y, quat.z, quat.w)

        new_yaw = yaw + self.offset_yaw
        new_quat = quaternion_from_euler(roll, pitch, new_yaw)
        fake_msg.pose.pose.orientation.x = new_quat[0]
        fake_msg.pose.pose.orientation.y = new_quat[1]
        fake_msg.pose.pose.orientation.z = new_quat[2]
        fake_msg.pose.pose.orientation.w = new_quat[3]

        #Covariance not changed for now, could also be modified if needed ##"How trustworthy is the localization?"
        fake_msg.pose.covariance = ref.pose.covariance
    
        #Update timestamp to current time, to avoid warnings Testweise!!
        #rt = rclpy.time.Time()
        fake_msg.header.stamp = self.get_clock().now().to_msg()
        #fake_msg.header.stamp = ref.header.stamp

        self.amcl_pub.publish(fake_msg)
        self.get_logger().debug(f"Published fake amcl pose: \n{fake_msg}")


    def save_tf(self, msg):
        #save tf of map-odom 
        try:
            req_stamp = rclpy.time.Time.from_msg(msg.header.stamp)

            if self.tf_buffer.can_transform("map", "odom", req_stamp, timeout=Duration(seconds=0.5)):
                tf = self.tf_buffer.lookup_transform("map", "odom", req_stamp, Duration(seconds=0.5))
                self.get_logger().debug("Found map-odom tf: " + str(tf))
                return tf
            if self.tf_buffer.can_transform("map", "odom", rclpy.time.Time(), timeout=Duration(seconds=0.5)):
                tf = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time(), Duration(seconds=0.5))
                self.get_logger().debug("Found map-odom tf: " + str(tf))
                return tf

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: \n{e}")
            return None
        
        self.get_logger().warn("TF lookup failed: No transform in buffer")
        return None



def main(args=None):
    rclpy.init(args=args)
    node = PoseAttackerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()


















