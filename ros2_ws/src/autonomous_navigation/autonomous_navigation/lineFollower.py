import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String, Float64
import math
import time
import os

class LineFollower(Node):
    def __init__(self):
        super().__init__('lineFollower')
        
        #Subscripción a la imágen redimensionada
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/image_resized', self.camera_callback, 10) #suscripcion a topico de imagen redimensionada
        self.subscription
        
        self.subscription = self.create_subscription(String, 'recalibration', self.recalibrate, 10) #uscripcion a topico de mensaje de recalibracon con tipo de dato indistinto
        self.pub_signal = self.create_publisher(String, 'signal_type', 10) #se publica el tipo de señal detectada en caso de ser interseccion
        
        #Publisher a la imágen procesada
        self.new_image_processed = self.create_publisher(Image, '/image_processed', 10) #publicacion de la imagen procesada
        self.angle_pub = self.create_publisher(Float64, '/angle_value', 10) #publciacion del alngulo obtenido a partir de la pendiente 
       
        self.signal = String()
        #Variables para el procesamiento de la imágen
        self.frame = None
        self.frame_gray = None
        self.thers = None
        self.blur = None
        self.imagen_erosionada = None
        self._ = None
        self.otsu_threshold = None
        self.otsu_binary = None
        self.kernel = None

        #Valores para el centroide
        self.cX = 0
        self.cY = 0
        self.cX2 = 180
        self.cY2 = 120
        
        #Variables para el procesamiento de los centroides
        self.centroids = []
        self.lines = []
        self.distances = []
        
        #Distancias  a la linea central
        self.distances_central = []
        
        #Contornos a dibujar
        self.contours_ax = []
        self.contours_draw = []      

        #Distancias
        self.distances_central = []
        
        #Variables para  la detección de lineas
        self.central_line = None
        self.aux_line = None
        
        #Angulo 
        self.angle = None
        self.angle = Float64()
        
        #Tiempo de calibración: 
        self.time_calibration = 5
        self.start_time = time.time()
        self.flag_calibration = True
        
        self.intersection = False
        
        self.intersection_time = None

        self.get_logger().info('Line Follower node successfully initialized!!') #mensaje de inicializacion del nodo

    def camera_callback(self, msg):
        try:
            #Obtenemos el frame redimensionado 
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            #Extraemos el segmento de frame de nuestro interés
            self.frame = self.frame[100:, :]
            self.process_image()
        except Exception as e:
            #Determinamos si se trata de una intersección o una curva
            if(abs(self.angle.data) > 0.7):
                pass
                #self.get_logger().info("Curva")
            else: #Intersección: 
                self.signal.data = "intersection"
                self.pub_signal.publish(self.signal) #se publica en el topico de la señal detectada la interseccion
                self.get_logger().info("Intersección")
            self.angle_pub.publish(self.angle) #se publica el angulo obtenido
    
    def recalibrate(self, msg): #en caso de dejar de detectar la linea, se manda vuelve a mandar la funcion de procesar imagen
        self.get_logger().info("Recalibrando")
        self.flag_calibration = True
     
        self.process_image()
    
    def get_central_line(self): #funcion para obtener los centroides y punto fijo de la imagen
        
        if self.distances:
            
            #Limpiamos la variable de los contornos a dibujar
            self.contours_draw = []
        
            #Obtenemos la linea más cercana al punto centro inferior de la imagen
            #Esta línea será la línea central A:
            
            minimo, indice_minimo = min((x,i) for i, x in enumerate(self.distances))
            self.central_line = self.centroids[indice_minimo]
            self.contours_draw.append(self.contours_aux[indice_minimo])
            
            #Eliminamos los elementos de los vectores auxiliares
            self.distances.pop(indice_minimo)
            self.centroids.pop(indice_minimo) 
            self.contours_aux.pop(indice_minimo)          

                
    def draw_line(self): #funcion para dibujar el contorno de la linea y el punto 
    
        #Dibujamos lso contornos, así como el centroide y asignamos la etiqueta del texto
        xc = self.central_line[0]
        yc = self.central_line[1]
        cv2.circle(self.frame, (xc, yc), 2, (0, 0, 0), -1)
        
        cv2.drawContours(image=self.frame, contours=self.contours_draw, contourIdx=-1, color=(0, 255, 0), thickness=1, lineType=cv2.LINE_AA)
    
    def show_central_line(self): #funcion para dibujar la linea entre los 2 puntos 
        
        #Calculamos las lineas centrales del siguiente frame: 
        new_central = None
    
        #Creamos vectores para calcular la distancia de cada contorno con las lineas centrales
        self.distances_central = []
        
        #Recorremos cada uno de los centroides
        for element in self.centroids: 

            #Obteniamos las coordenadas
            x2 = element[0]
            y2 = element[1]
        
            #Calculamos la distancia a la linea central           
            x = self.central_line[0]
            y = self.central_line[1]
   
            d = math.sqrt((x2-x)**2+(y2-y)**2)
            self.distances_central.append(d)
            
        
        #Limpiamos los contornos a dibujar
        self.contours_draw = []
            
        #Obtenemos la linea más cercana a linea central A: 
        minimo, indice_minimo = min((x,i) for i, x in enumerate(self.distances_central))
        if (minimo < 50): 
            self.central_line= self.centroids[indice_minimo]
            self.contours_draw.append(self.contours_aux[indice_minimo])
           
            self.distances_central.pop(indice_minimo)
            self.contours_aux.pop(indice_minimo)
            self.centroids.pop(indice_minimo)
                        
        self.draw_line()
        
    def pre_processing(self): 
        #Filtro blur
        self.frame = cv2.medianBlur(self.frame, 9) #se aplica filtro para eliminar ruido 
        
        #Convertimos la imágen a escala de grises
        self.frame_gray = cv2.cvtColor(self.frame, cv2.COLOR_RGB2GRAY)
        self._, self.otsu_threshold = cv2.threshold(self.frame_gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU) #binarizado otsu para ajusrtarlo automaticamente
        self.otsu_binary = np.where(self.frame_gray > self.otsu_threshold, 255, 0).astype(np.uint8)
        
        #Erosion
        kernel = np.ones((3,3),np.uint8)
        self.otsu_binary = cv2.erode(self.otsu_binary, kernel, iterations = 1)

    def process_image(self):
        #Procesamos la imágen
        self.pre_processing()
       
        #Encontramos los contornos 
        self.contours, _ = cv2.findContours(self.otsu_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
        #Filtramos los contornos bajo ciertos parametros      
        self.centroids = []
        self.distances = []
        self.contours_aux = [] 
        
        #Recorremos todos los contornos que se hayan detectado
        for contour in self.contours:
            #Caluclamos el centroide del contorno detectado
            
            self.M = cv2.moments(contour)
            
            if self.M["m00"] != 0:
                self.cX = int(self.M["m10"] / self.M["m00"])
                self.cY = int(self.M["m01"] / self.M["m00"])
            else:
                self.cX, self.cY = 0, 0
                
            #Calculamos el área del contorno                
            area = cv2.contourArea(contour)
            
            #Filtramos, seleccionando contornos que sobrepasen cierto umbral de área
            if area > 400:
                #Calculamos la distancia del centroide al centro inferiro de la imágen
                d = math.sqrt((self.cX2 - self.cX) ** 2 + (self.cY2 - self.cY) ** 2)
                #Agregamos los datos a vectores que nos servirán para calcular y seleccionar más adelante
                self.centroids.append([self.cX, self.cY])
                self.distances.append(d)
                self.contours_aux.append(contour)
                
        #Si es momento de calibración, buscamos las lienas centrales bajo cierto parametro            
        if self.flag_calibration: 
            cv2.putText(self.frame, "Calibrando", (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
            self.get_central_line()
            self.flag_calibration = False
            
        else:
            #Obtenemos las lineas basandonos en el frame anterior
            self.show_central_line()
            
            #Obtenemos las pendientes
            
            #Pendiente uno: 
            x = self.central_line[0]
            y = self.central_line[1]
            
            #Trazamos la linea: 
            cv2.line(self.frame, (self.cX2, self.cY2), (x,y), (0,255,0),2)  
        
            #Calculamos el ángulo 
            try:
                #Calcualmos el nángulo
                angle_degrees = -1*np.degrees(np.arctan2(y - self.cY2, x -self.cX2))
                #Calculamos el coseno (-1 a 1): 
                angle_radians = np.radians(angle_degrees)
                cos_angle = np.cos(angle_radians)
                           
                cv2.putText(self.frame, f"{round(cos_angle,2)}", (x - 5, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255, 0, 0), 1, cv2.LINE_AA) 
       
            except ZeroDivisionError:
                print('division by zero')
                
            self.new_image_processed.publish(self.bridge.cv2_to_imgmsg(self.frame, encoding="bgr8")) #se publica la imagen ya procesada          
            self.angle.data = cos_angle #el angulo a procesar es el coseno
            self.angle_pub.publish(self.angle) #se publica el angulo en su respectivo topico
            self.intersection = False
            
            #No hay intersección
            self.signal.data = "NOTintersection"
            self.pub_signal.publish(self.signal)
            
            
            

def main(args=None):
    rclpy.init(args=args)
    c_id = LineFollower()
    rclpy.spin(c_id)
    c_id.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()