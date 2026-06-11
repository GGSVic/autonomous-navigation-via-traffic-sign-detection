# Importamos las librerías necesarias
from ultralytics import YOLO  # Librería para el modelo YOLO
import rclpy
import cv2
from rclpy.node import Node  # Clase para nodos en ROS2
from sensor_msgs.msg import Image  # Mensaje de ROS 2 para imágenes
from cv_bridge import CvBridge
import os

from yolo_msg.msg import Yolov8Inference  # Mensaje personalizado para inferencias YOLOv8
from std_msgs.msg import String  # Mensaje de ROS2 para cadenas de texto

# Inicializamos un puente para convertir entre imágenes ROS y OpenCV
bridge = CvBridge()


# Hacemos la definición de la clase CameraSubscriber donde pondremos las suscripciones y también los publicadores
class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('signals_detector')  # Inicializamos un nodo con el nombre de signals_detector

        self.get_logger().info(
            'Initializing YOLO models...')  # Mensaje para conocer que ya se hace la inicialización de los modelos YOLO

        try:
            # Definimos una variable para poder cargar el modelo de YOLO y ponemos la ruta
            self.model1 = YOLO("/home/victor/YOLO_FinalChallenge/src/yolo_signals/yolo_signals/best.pt")
            self.get_logger().info('YOLO model1 loaded successfully.')  # Se manda un mensaje si se cargo correctamente
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model1: {e}')  # Si no se carga, se manda un mensaje de que fallo

        # Inicializamos un mensaje de inferencia YOLOv8
        self.yolov8_inference = Yolov8Inference()

        # Creamos una suscripción para la imagen redimensionada
        self.subscription = self.create_subscription(Image, '/image_resized', self.camera_callback, 10)
        self.subscription  # Esta suscripción no se usa directamente, solo se necesita crearla

        # Creamos ahora los publicadores de Yolov8_Inference que venia predeterminado en el código y también otro para poder mostrar la imagen que resulta del algoritmo
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "Yolov8_Inference", 10)
        self.img_pub = self.create_publisher(Image, "inference_result", 10)

        # Creamos un publicador para el tipo de señal detectada
        self.sign_pub = self.create_publisher(String, "signal_type", 10)
        self.signal_string = String()  # Inicializamos la cadena de señal

        self.get_logger().info(
            'Signal detector node successfully initialized !!!')  # Se manda un mensaje cuando se ha inicializado correctamente el nodo

    def camera_callback(self, data):
        self.get_logger().info('Image received')  # Tenemos un mensaje en cual sabemos que la imagen se recibio correctamente
        # Convertimos la imagen ROS a formato OpenCV
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        # Redimensionamos la imagen
        img = cv2.resize(img, (360, 120))
        img = img[:, :]

        # Realizamos la inferencia con el modelo YOLO
        results1 = self.model1(img)

        # Preparamos el mensaje de inferencia YOLOv8
        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        # Creamos una copia de la imagen para poder anotar los resultados
        annotated_frame = results1[0].plot()

        is_a_signal = False  # Creamos una variable para indicar si se detectó una señal

        # Iteramos sobre los resultados de la inferencia para poder hacer las cajas dibujadas de cada señal detectada
        for r in results1:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # Obtenemos las coordenadas de la caja
                c = box.cls  # Obtenemos la clase de la caja
                confidence = box.conf.item()  # Obtenemos la confianza de la detección para igualmente mostrarla
                class_name = self.model1.names[int(c)]  # Obtenemos el nombre de la clase
                self.signal_string.data = class_name  # Asignamos el nombre de la clase a la cadena de señal
                top, left, bottom, right = int(b[1]), int(b[0]), int(b[3]), int(b[2])  #  Se construyen las coordenadas de la caja

                # Calculamos el área de la caja
                width = abs(right - left) # ancho
                height = abs(bottom - top) # largo
                area = width * height # Se hace la multiplicación para obtenerla

                # Tenemos un ciclo i con un rango Si el área es grande o la clase es una luz de tráfico, anotamos la imagen
                if (area > 2200) or class_name in ['green_light', 'red_light', 'yellow_light']:
                    text = f'A:{area} P:{confidence:.2f}'
                    print(text) # Se imprime el texto del area y la confianza
                    is_a_signal = True  # Indicamos si es que se detectó una señal
                    (w, h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.3, 1)

                    # Se hace el dibujo de la bounding box donde se anota la señal que se esta detectando y el recuadro con el color que s ele dio a la clase de la señal en el modelo que cargamos arriba
                    cv2.rectangle(annotated_frame, (left, bottom - h - 2), (left + w, bottom), (255, 255, 255), -1)
                    cv2.putText(annotated_frame, text, (left, bottom - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 1,
                                cv2.LINE_AA)
                    self.sign_pub.publish(self.signal_string)  # Publicamos la señal que se este detectando
                else: # De otra forma
                    #  Se hace el dibujo de una cruz en la imagen para que no detecte
                    cv2.line(annotated_frame, (left, top), (right, bottom), (0, 0, 255), 1)
                    cv2.line(annotated_frame, (left, bottom), (right, top), (0, 0, 255), 1)

        if not is_a_signal: # si es que no hay señal
            self.signal_string.data = ""  # Se limpia la cada mandando un string vacío
            self.sign_pub.publish(self.signal_string)  # Publicamos el string vacío sobretodo para el caso del stop en especifico

        # Convertimos la imagen anotada a mensaje ROS y la publicamos
        img_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        self.img_pub.publish(img_msg)
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()


# Función principal
def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()  # Creamos una instancia de CameraSubscriber
    rclpy.spin(camera_subscriber)  # Mantenemos el nodo en ejecución
    rclpy.shutdown()  # Hacemos un shutdown

if __name__ == '__main__':
    main()
