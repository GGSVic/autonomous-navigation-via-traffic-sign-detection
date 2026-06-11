import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class image_resizer(Node):
    def __init__(self):
        super().__init__('imge_resizer')

        # Realizamos una subscripción al topico que publica la imágen capturada por la camara
        self.bridge = CvBridge()    
        self.subscription = self.create_subscription(Image, '/video_source/raw', self.camera_callback, 10)
        
        # Creamos un publisher para la imágen re-escalada
        self.image_resized = self.create_publisher(Image, '/image_resized', 10)

        #Mensjae de inicialización
        self.get_logger().info('Image resizer node successfully initialized!!')
        
        #Nueva dimensión de la imágen 
        self.width = 360 
        self.height = 120 

    def camera_callback(self, msg):
        try:
            #Leemos la imágen
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            #Ajustamos el tamaño del frame a 360 x 120
            self.frame = cv2.resize(self.frame, (self.width, self.height))
            #Rotamos la imágen (recomenado considerando que la cámara esta invertida)
            self.frame = cv2.rotate(self.frame, cv2.ROTATE_180)
            #Publicamos la imágen
            self.image_resized.publish(self.bridge.cv2_to_imgmsg(self.frame, encoding="bgr8"))
          
        except Exception as e:
            self.get_logger().info(f'Failed to convert image to CV2: {e}')
                              

def main(args=None):
    rclpy.init(args=args)
    c_id = image_resizer()
    rclpy.spin(c_id)
    c_id.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
