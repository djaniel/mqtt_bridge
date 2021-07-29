#!/usr/bin/env python3

"""
ROS NODE
Receives, decodes and sends the position of a tag using TF.
Similarly, the distances [mm] to all anchors and their 
corresponding ids are published. 
Subscriptions:
distances_b64, the string in base 64 containing the ids of 
   all anchors and the distance to the tag.
position, the position of the tag as received from the mqtt
   broker.
   
Publications:

distances: The distances and ids of the anchors used in the
   trilateration process. The result of the decoding process 
   of the base 64 string sent from the MQTT broker in the 
   gateway.
pose: the pose of the tag with respect to the map_frame parameter.
   
Remark: this node has to be launched in the namespace of the tag.
"""

import rospy
import tf_conversions
import tf2_ros
from mqtt_bridge.msg import deca_data, deca_distances, decawave
from std_msgs.msg import Int8
from geometry_msgs.msg import Pose, TransformStamped
from base64 import b64decode
from rosbridge_library.internal import message_conversion

__author__ = "Daniel Soto"
__license__ = "GPL"
__version__ = "1.0"



class DecawaveDecoding():
   def __init__(self):
      # This node will subscribe to distances coded in base 64 & the position of the tag
      rospy.Subscriber( 'distances_b64', deca_data, self.distances_b64_callback)
      rospy.Subscriber( 'position', decawave, self.position_callback)
      
      # It will decode the base 64 distances to a ros message and tf
      self.distances_pub = rospy.Publisher( 'distances', deca_distances, queue_size=10)
      self.pose_pub = rospy.Publisher( 'pose', Pose, queue_size=10)
      self.tf_br = tf2_ros.TransformBroadcaster()
      
      self.tf_buffer = tf2_ros.Buffer()
      self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
      
      self.tag_name = rospy.get_namespace().strip('/')
      self.map_frame = rospy.get_param('~map_reference_frame')
      rospy.loginfo('Decoding data for ' + self.tag_name + ' with respect to ' + self.map_frame )

   def position_callback(self, msg):
      # Publish transform from the position of tag
      t = TransformStamped()
      t.header.stamp = rospy.Time.now()
      t.header.frame_id = "anchor1"
      t.child_frame_id =  self.tag_name
      t.transform.translation.x = msg.position.x
      t.transform.translation.y = msg.position.y
      t.transform.translation.z = msg.position.z
      
      q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
      t.transform.rotation.x = q[0]
      t.transform.rotation.y = q[1]
      t.transform.rotation.z = q[2]
      t.transform.rotation.w = q[3]
      
      self.tf_br.sendTransform(t)

   def distances_b64_callback(self, msg):
      # Decode, parse and store data into dictionary.
      msg_string = b64decode(msg.data)
      
      msg_dict =  dict()
      msg_dict['tags_available'] = msg_string[0]
      for index in range(0, msg_string[0]):
         anchor =  dict()
         anchor['id'] = hex (msg_string[index*6 + 2]*256 +  msg_string[index*6 + 1 ])
         anchor['distance'] = msg_string[index*6 + 4]*256 +  msg_string[index*6 + 3 ] 
         key = 'anchor' + str(index+1)
         msg_dict[key] = anchor

      # Populate instace of message with the contents of dictionary & publish
      try:
         msg_distances = deca_distances()
         message_conversion.populate_instance(msg_dict, msg_distances) 
         self.distances_pub.publish ( msg_distances )
      except Exception as e:
         rospy.logerr(e)

   def transform_to_map(self):
      pose = Pose()
      try:
         trans = self.tf_buffer.lookup_transform( self.map_frame, self.tag_name, rospy.Time())
         pose.position.x = trans.transform.translation.x
         pose.position.y = trans.transform.translation.y
         pose.position.z = 0
         pose.orientation.x = 0
         pose.orientation.y = 0
         pose.orientation.z = 0
         pose.orientation.w = 1
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
         pass
      return pose
      
   def tag_position(self):
      pose = self.transform_to_map()
      self.pose_pub.publish(pose)

if __name__ == '__main__':
   rospy.init_node('distances_decoding')
   decoder = DecawaveDecoding()
   
   # At 10Hz the pose of the tag with respect to the map_frame parameter will be published
   rate = rospy.Rate(10)
   while not rospy.is_shutdown():
      decoder.tag_position()
      
      rate.sleep()
