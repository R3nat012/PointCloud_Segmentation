#!/usr/bin/env python3
import numpy as np
import rospy, sys, time, random
import message_filters
from roslib import message
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo

from yolov8_ros.msg import DetectionArray

class Segmented_pcl:
    def __init__(self):

        # Iniciamos el nodo llamado Pcl_segmentation
        rospy.init_node("Pcl_segmentation", anonymous=True)

        # Para poder transformar las coordenadas 2d de la imagen a 3d, necesitamos utilizar el paquete image_geometry
        self.camera_model = PinholeCameraModel()
        
        # Creamos los suscriptores de los topicos de yolo, slam y el topico de la informacion de la camara
        rospy.Subscriber("yolov8_node/detections", DetectionArray, self.callback_yolo)
        rospy.Subscriber("orb_slam2_mono/map_points", PointCloud2, self.callback_pcl)
        rospy.Subscriber("camera/rgb/camera_info", CameraInfo, self.callback_caminfo)

        # Creamos el publisher modified_pcl
        self.Sgment_pcl = rospy.Publisher('/modified_pcl', PointCloud2, queue_size=1)

        # Variables a almacenar
        self.point_seg = PointCloud2()

        # Verificamos que inicializamos la camara 
        self.camera_info_received = False

        # Nube de puntos almacenada
        self.modified_points = []


    def callback_yolo(self, yolo_detection_array):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", len(yolo_detection_array.detections))
        # Para ello, necesitamos las coordenadas del centro del bounding box
        
        bbox_center_x = yolo_detection_array.detections[0].bbox.center.position.x
        bbox_center_y = yolo_detection_array.detections[0].bbox.center.position.y
        # Extraemos el tamano del bounding box
        bbox_size_x = yolo_detection_array.detections[0].bbox.size.x
        bbox_size_y = yolo_detection_array.detections[0].bbox.size.y
        print(len(yolo_detection_array.detections))

        # Obtenemos el bounding box total

        limit_left = bbox_center_x - (bbox_size_x / 2)
        limit_right = bbox_center_x + (bbox_size_x / 2)

        limit_up = bbox_center_y - (bbox_size_y / 2)
        limit_down = bbox_center_y + (bbox_size_y / 2)

        # Convertimos las coordenadas
        depth = 8.0
        top_left = self.camera_model.projectPixelTo3dRay((limit_left, limit_up))
        bottom_right =  self.camera_model.projectPixelTo3dRay((limit_right, limit_down))
        x_tl, y_tl, z_tl = [depth * r for r in top_left]
        x_br, y_br, z_br = [depth * r for r in bottom_right]

        num_channels = 4; # x y z rgba
        size_cloud = self.point_seg.width * self.point_seg.height  # Total number of points

        original_pcl = self.point_seg
        if size_cloud > 0:
            points = pc2.read_points(self.point_seg, field_names=("x", "y", "z"), skip_nans=True)
            
            # Prepare a list to hold the new point data with RGBA
            
            x_tl_new = x_tl*-1
            for point in points:
                x, y, z = point
                
                if (x >= x_br) and (x <= x_tl_new) and (y <= y_br):
                    r = int(255) % 256
                    g = int(1)
                    b = int(1)  # constant blue color
                    a = 255       # full opacity
                
                else:
                    # Generate RGB colors based on coordinates or any other criteria (here it's arbitrary)
                    r = int(1)
                    g = int(255) % 256
                    b = int(1)  # constant blue color
                    a = 255       # full opacity

                # Pack RGBA as a single 32-bit integer
                rgba = (a << 24) | (b << 16) | (g << 8) | r
                
                # Append x, y, z, rgba to the new points list
                self.modified_points.append([x, y, z, rgba])

            # Define the PointFields for x, y, z, and rgba channels
            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('rgba', 12, PointField.UINT32, 1),
            ]

            # Create a new PointCloud2 message with modified points and rgba color
            modified_cloud = pc2.create_cloud(self.point_seg.header, fields, self.modified_points)

            # Publish the modified point cloud
            self.Sgment_pcl.publish(modified_cloud)

        
    def callback_pcl(self, cloud):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s",  len(list(pc2.read_points(cloud))))
        self.point_seg = cloud
    
    # La funcion callback sera la parte donde se modifica la nube de puntos
    def callback_caminfo(self, cam_info):

        # Obtenemos la transformacion de 2d a 3Dray
        if not self.camera_info_received:
            self.camera_model.fromCameraInfo(cam_info)
            self.camera_info_received = True
            rospy.loginfo("Camera info received and model initialized")
    
        
    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = Segmented_pcl()
    node.run()