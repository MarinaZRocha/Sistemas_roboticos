import math
import tf2_ros
import tf2_geometry_msgs
import smach
import rospy
import threading
import statistics
import cv2 as cv
import numpy as np
from sklearn.cluster import KMeans
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge

from sklearn.cluster import DBSCAN
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Vector3Stamped, Vector3, PointStamped
from geometry_msgs.msg import Point, Quaternion
from tf2_geometry_msgs import PoseStamped
from visualization_msgs.msg import Marker
from image_geometry.cameramodels import PinholeCameraModel
from sensor_msgs.msg import LaserScan, PointCloud2, PointField, Image, CameraInfo

class FindGarbageState(smach.State):
  def __init__(self, binHeight = 0.6, timeout = 5, min_area = 30000, max_area = 250000, mode = 'max', marker_timeout = 20):
    smach.State.__init__(self, outcomes = ['succeeded', 'preempted', 'aborted'], output_keys = ['pose'])
    self.timeout = timeout
    self.binHeight = binHeight
    self.min_area = min_area
    self.max_area = max_area
    self.cv_bridge = CvBridge()
    self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)  
    self.pub_transform = tf2_ros.TransformBroadcaster()
    self.subscriber_camera_info = rospy.Subscriber("/usb_cam/camera_info", CameraInfo, self._update_camera_info, queue_size=1)
    self.camera_info = rospy.wait_for_message("/usb_cam/camera_info", CameraInfo)

  def _update_camera_info(self, msg: CameraInfo):
    self.camera_info = msg

  def execute(self, userdata):
    Posestamped = PoseStamped()
    pose = Pose()
    point = Point()
    quaternion = Quaternion()
    quaternion.x = 0
    quaternion.y = 0
    quaternion.z = 0
    quaternion.w = 1
    camera_matrix = np.array(self.camera_info.K).reshape(3,3)
    image = rospy.wait_for_message('/usb_cam/image_raw', Image)
    Posestamped.header = image.header
    pose.orientation = quaternion
    frame = cv.cvtColor(self.cv_bridge.imgmsg_to_cv2(image), cv.COLOR_BGR2GRAY)
    gauss = cv.GaussianBlur(frame, (5,5), 0)
    Z = gauss.reshape((-1,3))
    # convert to np.float32
    Z = np.float32(Z)
    # define criteria, number of clusters(K) and apply kmeans()
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    K = 2
    ret,label,center=cv.kmeans(Z,K,None,criteria,10,cv.KMEANS_RANDOM_CENTERS)        
    center = np.uint8(center)
    res = center[label.flatten()]
    res2 = res.reshape((gauss.shape))
    
    ret, thresh = cv.threshold(res2, 150, 255, cv.THRESH_BINARY)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    contour_image = cv.cvtColor(res2, cv.COLOR_GRAY2BGR)
    

    img_center_x = contour_image.shape[1] // 2
    img_center_y = contour_image.shape[0] // 2 

    valid_contours=[]

    for contour in contours:
        area = cv.contourArea(contour)
        M = cv.moments(contour)
        if self.min_area < area < self.max_area:
            cv.drawContours(contour_image, [contour], -1, (0, 0, 255), 3)
            
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv.circle(contour_image , (cx, cy), 1, (0, 255, 0), 5)
                
                (x,y),radius = cv.minEnclosingCircle(contour)
                radius = int(radius)
                if radius <= 250:
                  valid_contours.append((contour, cx, cy, radius))
                
        # Verifica o número de contornos válidos
        if len(valid_contours) > 0:
            closest_contour = None
            min_distance = float('inf')

            # Se houver mais de um contorno, encontra o mais próximo do centro
            if len(valid_contours) >= 1:
                for contour, cx, cy, radius in valid_contours:
                    #distance = np.sqrt((cx - img_center_x) * 2 + (cy - img_center_y) * 2)
                    
                    distance = np.linalg.norm(np.array([cx,cy])- np.array([img_center_x,img_center_y]))
                    
                    if distance < min_distance:
                        min_distance = distance
                        closest_contour = (contour, cx, cy, radius)

                    # Desenha o contorno mais próximo, caso tenha sido encontrado
                    if closest_contour:
                        contour, cx, cy, radius = closest_contour
                        cv.circle(contour_image, (cx, cy), radius, (0, 255, 0), 2)
                        cv.circle(contour_image, (cx, cy), 1, (255, 0, 0), 5)
                        print("---------------------------------------------")
                        print(self.camera_info)
                        print("---------------------------------------------")
                        print(camera_matrix)
                        x = (cx - camera_matrix[0,2]) * self.binHeight/camera_matrix[0,0]
                        y = (cy - camera_matrix[1,2]) * self.binHeight/camera_matrix[1,1]
                        point.x = self.binHeight
                        point.y = y
                        point.z = x
                        print("Point found", point)

                        cv.imshow('res2',contour_image)
                        if cv.waitKey(1) == 27:
                            break

                        pose.position = point
                        print('\n\n pose.position ::: \n\n', pose.position, 'pose.header ::: \n\n', Posestamped.header) 
                        Posestamped.pose = pose
                        try:    
                                print("going to transform")
                                transform = self.tf_buffer.lookup_transform('map', Posestamped.header.frame_id, rospy.Time(0), rospy.Duration(10.0))
                                # print('transformada', transform)
                                rospy.loginfo("Transform found")
                                pose2map = tf2_geometry_msgs.do_transform_pose(Posestamped, transform)
                                print('pose do objeto para o gripper ir ::: \n\n\n', pose2map)
                                userdata.pose = pose2map

                                transformStamped = TransformStamped()
                                transformStamped.header.stamp = rospy.Time.now() 
                                transformStamped.header.frame_id = 'map'  
                                transformStamped.child_frame_id = 'object2pickup'
                                transformStamped.transform.translation.x = pose2map.pose.position.x
                                transformStamped.transform.translation.y = pose2map.pose.position.y
                                transformStamped.transform.translation.z = pose2map.pose.position.z
                                transformStamped.transform.rotation.x = pose2map.pose.orientation.x
                                transformStamped.transform.rotation.y = pose2map.pose.orientation.y
                                transformStamped.transform.rotation.z = pose2map.pose.orientation.z
                                transformStamped.transform.rotation.w = pose2map.pose.orientation.w

                                self.pub_transform.sendTransform(transformStamped)
                                return 'succeeded'
                        except:
                            return 'aborted'
                else:
                    return 'aborted'       
        
    return 'aborted'
