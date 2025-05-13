# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-

# import rospy
# import numpy as np
# # import quaternion
# # from cv_bridge import CvBridge
# # from sensor_msgs.msg import Image
# # from sensor_msgs.point_cloud2 import PointCloud2
# from tf2_msgs.msg import TFMessage
# from tf2_ros import TransformBroadcaster
# from geometry_msgs.msg import PoseStamped,TransformStamped ,PoseWithCovarianceStamped
# from std_msgs.msg import Float64MultiArray

# # from sensor_msgs.msg import PointCloud2
# # from sensor_msgs.msg import PointCloud
# # from sensor_msgs.msg import PointField


# def pr_callback(data  ):
#     msg = PoseStamped()
#     msg.header.frame_id = 'map'
#     msg.header.stamp = rospy.Time.now()
#     msg.pose.position.x = data.data[0]
#     msg.pose.position.y = data.data[1]
#     msg.pose.position.z = data.data[2]

#     msg.pose.orientation.x = data.data[4]
#     msg.pose.orientation.y = data.data[5]
#     msg.pose.orientation.z = data.data[6]
#     msg.pose.orientation.w = data.data[3]
#     pose_pub.publish(msg)
#     global msgcov
#     msgcov = PoseWithCovarianceStamped()
#     msgcov.header.frame_id = 'map'
#     msgcov.header.stamp = rospy.Time.now()

#     msgcov.pose.pose.position.x = data.data[0]
#     msgcov.pose.pose.position.y = data.data[1]
#     msgcov.pose.pose.position.z = data.data[2]

#     msgcov.pose.pose.orientation.x = data.data[4]
#     msgcov.pose.pose.orientation.y = data.data[5]
#     msgcov.pose.pose.orientation.z = data.data[6]
#     msgcov.pose.pose.orientation.w = data.data[3]
#     posecov_pub.publish(msgcov)

#     transform_msg.header.stamp = rospy.Time.now()
#     transform_msg.transform.translation.x = data.data[0]
#     transform_msg.transform.translation.y = data.data[1]
#     transform_msg.transform.translation.z = data.data[2]

#     q = np.array([data.data[4],data.data[5],data.data[6],data.data[3]])
#     # print(q)
#     # q = np.array([-data.data[3],-data.data[4],-data.data[5],data.data[6]])/np.linalg.norm(q)
#     # q = -q
#     transform_msg.transform.rotation.x = q[0]
#     transform_msg.transform.rotation.y = q[1]
#     transform_msg.transform.rotation.z = q[2]
#     transform_msg.transform.rotation.w = q[3]
    
#     tf_broadcaster.sendTransform(transform_msg)
#     print("transform_msg:",transform_msg)



# if __name__ == "__main__":
#     rospy.init_node("pr_convert")
#     rospy.loginfo('node runing...')
#     pr_sub = rospy.Subscriber("/sim/torso/PR",Float64MultiArray,pr_callback,queue_size=10)
#     pose_pub = rospy.Publisher("/sim/torso/Pose",PoseStamped,queue_size=1)
#     posecov_pub = rospy.Publisher("/sim/torso/PoseCov",PoseWithCovarianceStamped,queue_size=1)
#     planpos_pub = rospy.Publisher("/sim/torso/PoseCov_plan",PoseWithCovarianceStamped,queue_size=1)

#     msgcov = PoseWithCovarianceStamped()
#     tf_broadcaster = TransformBroadcaster()
#     transform_msg = TransformStamped()
#     transform_msg.header.frame_id = 'map'
#     transform_msg.child_frame_id = 'torso_map'
#     rate = rospy.Rate(5)
#     cnt = 0 
#     while not rospy.is_shutdown():
#         # tf_broadcaster.sendTransform(transform_msg)
#         if cnt == 3:
#             planpos_pub.publish(msgcov)
#             cnt = 0
#         rate.sleep()  
#         cnt += 1

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
# import quaternion
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image
# from sensor_msgs.point_cloud2 import PointCloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseStamped,TransformStamped ,PoseWithCovarianceStamped
from std_msgs.msg import Float64MultiArray

# --- Global Variables ---
# Define globals explicitly at the top level for clarity
pose_pub = None
posecov_pub = None
planpos_pub = None
tf_broadcaster = None
transform_msg = None # Initialize as None initially
latest_msgcov = None # Use this instead of global msgcov inside callback

def pr_callback(data):
    # Use global variables that we intend to modify or use
    global latest_msgcov, tf_broadcaster, transform_msg, pose_pub, posecov_pub

    # --- Check if necessary components are initialized ---
    # This is crucial if the callback somehow gets triggered before main() finishes setup
    if pose_pub is None or posecov_pub is None or tf_broadcaster is None or transform_msg is None:
        rospy.logwarn_throttle(2.0, "Publishers or TF broadcaster/message not ready yet in callback.")
        return

    rospy.loginfo("pr_callback triggered!") # Add this log message

    current_time = rospy.Time.now() # Get current time once

    # --- Publish PoseStamped ---
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = current_time
    msg.pose.position.x = data.data[0]
    msg.pose.position.y = data.data[1]
    msg.pose.position.z = data.data[2]
    msg.pose.orientation.x = data.data[4]
    msg.pose.orientation.y = data.data[5]
    msg.pose.orientation.z = data.data[6]
    msg.pose.orientation.w = data.data[3] # w is index 3
    pose_pub.publish(msg)

    # --- Publish PoseWithCovarianceStamped ---
    msgcov = PoseWithCovarianceStamped()
    msgcov.header.frame_id = 'map'
    msgcov.header.stamp = current_time
    msgcov.pose.pose.position.x = data.data[0]
    msgcov.pose.pose.position.y = data.data[1]
    msgcov.pose.pose.position.z = data.data[2]
    msgcov.pose.pose.orientation.x = data.data[4]
    msgcov.pose.pose.orientation.y = data.data[5]
    msgcov.pose.pose.orientation.z = data.data[6]
    msgcov.pose.pose.orientation.w = data.data[3] # w is index 3
    # Store for the periodic planner publisher
    latest_msgcov = msgcov
    posecov_pub.publish(msgcov)


    # --- Update and Send Transform ---
    transform_msg.header.stamp = current_time # Use the same consistent timestamp
    transform_msg.transform.translation.x = data.data[0]
    transform_msg.transform.translation.y = data.data[1]
    transform_msg.transform.translation.z = data.data[2]
    transform_msg.transform.rotation.x = data.data[4]
    transform_msg.transform.rotation.y = data.data[5]
    transform_msg.transform.rotation.z = data.data[6]
    transform_msg.transform.rotation.w = data.data[3] # w is index 3

    try:
        rospy.loginfo("Attempting to send transform...") # Log before sending
        tf_broadcaster.sendTransform(transform_msg)
        rospy.loginfo("Transform sent successfully.") # Log after sending
        # print("transform_msg:",transform_msg) # Optional: keep if needed for debugging values
    except Exception as e:
        rospy.logerr(f"Error sending transform: {e}")


if __name__ == "__main__":

    rospy.init_node("pr_convert")
    rospy.loginfo('node running...')

    # --- Initialize Publishers ---
    pose_pub = rospy.Publisher("/sim/torso/Pose", PoseStamped, queue_size=1)
    posecov_pub = rospy.Publisher("/sim/torso/PoseCov", PoseWithCovarianceStamped, queue_size=1)
    planpos_pub = rospy.Publisher("/sim/torso/PoseCov_plan", PoseWithCovarianceStamped, queue_size=1)

    # --- Initialize TF Broadcaster and Message Structure ---
    tf_broadcaster = TransformBroadcaster()
    transform_msg = TransformStamped()
    transform_msg.header.frame_id = 'map'      # Parent frame
    transform_msg.child_frame_id = 'torso_map' # Child frame

    # --- Initialize variable for periodic publishing ---
    latest_msgcov = PoseWithCovarianceStamped() # Initialize with default empty message

    # --- Initialize Subscriber - IMPORTANT: Do this *after* others are initialized ---
    pr_sub = rospy.Subscriber("/sim/torso/PR", Float64MultiArray, pr_callback, queue_size=10)
    rospy.loginfo('Subscriber created for /sim/torso/PR')

    # --- Main Loop for Periodic Tasks (like planner pose) ---
    rate = rospy.Rate(5) # Loop rate 5 Hz
    publish_interval_cycles = 3 # Publish every 3 cycles
    cnt = 0
    while not rospy.is_shutdown():
        if cnt >= publish_interval_cycles:
            if latest_msgcov is not None and planpos_pub is not None:
                 # Optionally update timestamp if needed by planner
                 # latest_msgcov.header.stamp = rospy.Time.now()
                 planpos_pub.publish(latest_msgcov)
                 rospy.loginfo("Published PoseCov_plan") # Add log
            cnt = 0 # Reset counter
        else:
            cnt += 1

        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("ROSInterruptException received, shutting down.")
            break
        except Exception as e:
             rospy.logerr(f"Error in main loop: {e}")
             break

    rospy.loginfo("pr_convert node finished.")