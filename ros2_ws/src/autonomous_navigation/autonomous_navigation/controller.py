import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Twist

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        ''' Nos subscribimos a los tópicos /angle_value y /signal_type, necesarios para la implementación del 
        controlador '''
        self.angles_v = self.create_subscription(Float64, '/angle_value', self.angles_callback, 10)
        self.signals_subs = self.create_subscription(String, 'signal_type', self.signals_actions, 10)

        '''Creamos publishers para realizar la recalibración del seguidor de línea y para 
        publicar la velocidad calculada para la navegación autonoma del robot'''
        self.recalibration = self.create_publisher(String, 'recalibration', 10)
        self.msg_recalibration = String()
        self.msg_recalibration.data = ""

        self.speed_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speed_configuration = Twist()

        '''Mensaje de inicialización del nodo'''
        self.get_logger().info('Controller node successfully initialized!!')

        '''Declaramos las variables (constantes) para la implementación del controlador, tanto para velocidades
        altas como para velociddaes bajas'''
        self.kp_max = 0.15 #Constante del controlador para velocidad máxima
        self.kp_min = 0.1 #Constante del controlador para velocidad mínima

        self.linear_max = 0.15 #Velocidad lineal máxima
        self.linear_min = 0.08 #Velocidad lineal mínima

        self.kp = self.kp_max #Inicializamos con velocidad máxima 
        self.linear = self.linear_max #Inicializamos con velocidad máxima

        ''' Declaramos las banderas y variables auxiliares para el funcionamiento del controlador'''

        self.ignore = False #Bandera para ignorar line_follower
        self.time_sleep_follower = 3.0 #Tiempo a ignorar line_follwer

        self.delay_traffic_light = 0.0 #Tiempo para la detección del semaforo
        self.tfColor = None #Variable para el tipo de semaforo (estado actual)
        self.tf_color_last = None #Variable para el ultimo tipo de semafor (ultimo estado)
        self.valid_tf = 0 #Esta variable nos permite identificar continuidad en los frames
                
        self.signal_type = None #Varaible para el tipo de signal 
        self.signal_last = None #Variable para el último estado de la señal detectada
        self.valid_signal = 0 #Esta varaible nos permite identificar continuar en los frames
        
        #Las siguientes variables nos permiten manejar el tiempo para seguir el comportamiento acorde a las señales
        self.signal_timer = 0.0 
        self.flag_signal = True
        self.intersection_flag = False
        #Variables para el manejo de la rotonda y turn_left / turn_right
        self.rotund = False
        self.left_right = False

        #Variable para ignroar el lineFollower
        self.time_ignore_follower = 0.0

    def angles_callback(self, msg):

        if not self.ignore: #Si no se ignora el seguidor
            angle = msg.data #Dato recibido del msg
            angular_speed = -self.kp * angle #Velocidad angular, definida por el error multiplicado por la constane kp

            self.speed_configuration.linear.x = self.linear #Velocidad lineal constante
            self.speed_configuration.angular.z = angular_speed #Velocidad angular definida por el error

        elif self.tf_color == "red_light": #Se ignora el seguidor
            if time.time() - self.time_ignore_follower > 2.0:  #Esperamos un tiempo a que ignore el seguidor
                self.ignore = False
            self.speed_configuration.linear.x = self.linear #lineal constante
            self.speed_configuration.angular.z = 0.0 #Angular nula 

            # Ignorar el seguidor de línea
            self.get_logger().info("Ignoring line_follower")

        # Esperamos dos segundos para reiniciar el estado del semaforo 
        '''Esta sección sirve principalmente cuando el semaforo es detectado en amarillo y el robot 
        no llega a reconocer un cambio a verde'''
        if time.time() - self.delay_traffic_light >= 2 and self.tf_color is not None:
            self.linear = self.linear_max
            self.kp = self.kp_max
            self.tf_color = None
        
        # Mantenemos la señal identificada durante 3.5 segundos antes de reiniciar las velocidades
        if time.time() - self.signal_timer > 3.5 and not self.flag_signal:
            self.flag_signal = True
            self.linear = self.linear_max
            self.kp = self.kp_max
            

        self.speed_pub.publish(self.speed_configuration) #Publicamos la velocidad
    
    def signals_actions(self, msg):
        signal = msg.data #Leemos la señal detectada

        #Semaforo verde: Aumentamos la velocidad
        if signal == "green_light" and self.valid_tf > 5:
            self.linear = self.linear_max
            self.kp = self.kp_max
            self.tf_color = signal
            self.delay_traffic_light = time.time()

        #Semaforo amarillo: Disminuimos la velocidad
        elif signal == "yellow_light" and self.valid_tf > 5:
            self.linear = self.linear_min
            self.kp = self.kp_min
            self.tf_color = signal
            self.delay_traffic_light = time.time()

        #Semaforo rojo: únicamente asignamos el color a la variable y esperamos a la intersección
        elif signal == "red_light" and self.valid_tf > 5:
            self.tf_color = signal
            self.delay_traffic_light = time.time()
        

        #Intersección: Para este caso, la intersección unicamente detiene al robot si 
        #el semaforo esta en rojo, de otra manera, una intersección no hace nada más que 
        #ignorar momentaneamente el seguidor
        elif signal == "intersection" and self.tf_color != None:
            self.intersection_flag = True
            self.ignore = True
            if self.tf_color == "red_light":
                self.linear = 0.0
                self.kp = 0.0   
            self.time_ignore_follower = time.time()
        
        #Esta variable indica que la detección de una señal ha sido constante
        aux = self.valid_signal > 10

        # Para estas señales, únicamente disminuimos la velocidad 
        if (signal == "give_way" or signal == "roadwork_ahead") and self.flag_signal and aux:
            self.linear = self.linear_min
            self.kp = self.kp_min
            self.signal_timer = time.time()
            self.flag_signal = False
            self.signal_type = signal
            
        #Si se detecta un roundabout, realizamos un giro a la izquierda utilizando lazo abierto 
        if signal == "roundabout" and not self.rotund:
            time.sleep(0.9)
            self.signal_type = signal
            self.rotund = True
            #self.ignore = True
            #self.get_logger().info("ROUNDABOUT")
            self.speed_configuration.linear.x = 0.15
            self.speed_configuration.angular.z = 0.3
            self.speed_pub.publish(self.speed_configuration)
            time.sleep(1)
            self.speed_configuration.angular.z = 0.0
            self.speed_configuration.linear.x = 0.1
            self.speed_pub.publish(self.speed_configuration)
            self.recalibration.publish(self.msg_recalibration) #Recalibramos el lineFollower
            #self.rotund = False

        #Si se detecta una señal de turn_right o turn_left, movemos el robot por lazo abierto para 
        #realizar la trayectoria correspondiente.  En este caso, se mueve el robot a la dereca independientemente 
        # de la señal, esto debido a que nuestro modelo presenta dificultades para diferenciar entre estas

        elif (signal == "turn_right_ahead" or signal == "turn_left_ahead") and not self.left_right:
            time.sleep(0.9)
            self.signal_type = signal
            self.left_right = True
            #self.ignore = True
            #self.get_logger().info("ROUNDABOUT")
            self.speed_configuration.linear.x = 0.15
            self.speed_configuration.angular.z = -0.3
            self.speed_pub.publish(self.speed_configuration)
            time.sleep(1)
            self.speed_configuration.angular.z = 0.0
            self.speed_configuration.linear.x = 0.1
            self.speed_pub.publish(self.speed_configuration)
            self.recalibration.publish(self.msg_recalibration) #Recalibramos el lineFollower para reiniciar trayectoria

        #En caso de que la señal sea stop, detenemos   
        elif signal == "stop" and aux:
            self.signal_type = signal
            self.linear = 0.0
            self.kp = 0.0

        #Si se deja de detectar una señal y ha transcurrido el tiempo suficietne 
        #para dejar de detectar una señal, reiniciamos las velocidades
        elif signal == "" and self.flag_signal: 
            self.signal_type = signal
            self.linear = self.linear_max
            self.kp = self.kp_max
            
        else: 
            self.next_movement = signal

        #Validamos la lectura dle semaforo, con el fin de evitar
        #lecturas ruidosas
        if self.tf_color_last == self.tf_color:
            self.valid_tf += 1
        else:
            self.valid_tf = 0
            
        #Validamos la lectura de señales, con el fin de evitar
        #lecutras ruidosas
        if self.signal_last == self.signal_type: 
            self.valid_signal += 1
        else: 
            self.valid_signal = 0

        self.tf_color_last = self.tf_color
        self.signal_last = self.signal_type

        #NO SE DEFINE UNA ACCIÓN PARA AHEAD_ONLY DEBIDO A QUE NO REALIZA NADA EN ESPECÍFICO

#Inicializamos el nodo
def main(args=None):
    rclpy.init(args=args)
    c_id = Controller()
    rclpy.spin(c_id)
    c_id.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

          
