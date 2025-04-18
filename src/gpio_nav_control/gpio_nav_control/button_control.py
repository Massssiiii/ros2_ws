#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from gpiozero import Button
import time
import threading
import math

class ButtonControlNode(Node):
    def __init__(self):
        super().__init__('button_control_node')
        
        # Utilisation d'un groupe de callbacks réentrant
        cb_group = ReentrantCallbackGroup()
        
        # Configuration des GPIO avec gpiozero - Pull-up pour les boutons connectés à GND
        self.position_button = Button(23, pull_up=True, bounce_time=0.1)
        self.goal_switch = Button(24, pull_up=True, bounce_time=0.1)
        
        # Ajout des gestionnaires d'événements
        self.position_button.when_pressed = self.position_button_pressed
        self.position_button.when_released = self.position_button_released
        self.goal_switch.when_pressed = self.goal_switch_released
        self.goal_switch.when_released = self.goal_switch_pressed
        
        # Variables pour mesurer la durée d'appui
        self.position_press_start_time = 0
        self.goal_press_start_time = 0
        
        # États du système
        self.mode = "initial"  # initial, nav2 ou manu
        self.position_color = "yellow"  # Position par défaut: jaune
        self.obstacle_detected = False
        
        # Variables pour la séquence manuelle
        self.manual_sequence_active = False
        self.manual_sequence_step = 0
        self.current_action = None
        self.action_start_time = None
        
        # Positions initiales (convertie avec (0,0) au centre et en mètres)
        self.blue_positions = [
            {'x': 0.275, 'y': -0.8, 'theta': 1.57}  # Zone en Bas avec rotation positive
        ]
        self.yellow_positions = [
            {'x': -0.275, 'y': -0.8, 'theta': 1.57}  # Zone en Bas avec rotation positive
        ]
        
        # Goals d'arrivée
        self.blue_goal = self.create_pose(1.05, 0.75, 1.57)   # Zone d'arrivée bleue
        self.yellow_goal = self.create_pose(-1.05, 0.75, 1.57)  # Zone d'arrivée jaune
        
        # Publishers et clients d'action
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            'initialpose', 
            10
        )
        
        # Subscriber pour le LIDAR
        self.subscriber_lidar = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10, 
            callback_group=cb_group)
        
        # Action client pour Nav2
        self.nav_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose',
            callback_group=cb_group
        )
        
        # Timer pour les mouvements manuels
        self.timer = self.create_timer(0.1, self.timer_callback, callback_group=cb_group)
        
        self.get_logger().info('Node démarré en mode INITIAL - En attente des actions sur les boutons')
    
    def position_button_pressed(self):
        """Appelé quand le bouton de position est pressé"""
        self.position_press_start_time = time.time()
        self.get_logger().info("Bouton de position appuyé")
    
    def position_button_released(self):
        """Appelé quand le bouton de position est relâché"""
        # Calcul de la durée d'appui
        duration = time.time() - self.position_press_start_time
        self.get_logger().info(f"Bouton de position relâché après {duration:.2f} secondes")
        
        # Logique en fonction du mode actuel
        if self.mode == "initial":
            if duration < 1.0:  # Appui court en mode initial
                self.mode = "nav2"
                self.get_logger().info("Passage en mode NAV2")
            elif duration >= 1.0:  # Appui long en mode initial
                self.mode = "manu"
                self.get_logger().info("Passage en mode MANUEL")
        
        elif self.mode == "nav2":
            if duration < 1.0:  # Appui court en mode nav2
                self.position_color = "yellow"
                self.get_logger().info("Mode Nav2: Position JAUNE")
                position = self.yellow_positions[0]
                self.set_initial_pose(position['x'], position['y'], position['theta'], "yellow")
            elif 1.0 <= duration < 3.0:  # Appui moyen en mode nav2
                self.position_color = "blue"
                self.get_logger().info("Mode Nav2: Position BLEUE")
                position = self.blue_positions[0]
                self.set_initial_pose(position['x'], position['y'], position['theta'], "blue")
            else:  # Appui très long en mode nav2
                self.mode = "initial"
                self.get_logger().info("Retour au mode INITIAL")
                # Arrêter toute navigation en cours
                #self.cancel_navigation()
                self.stop_robot()
                # Réinitialiser les étapes
                self.manual_sequence_step = 0
    
        elif self.mode == "manu":
            if duration < 1.0:  # Appui court en mode manuel
                self.position_color = "yellow"
                self.get_logger().info("Mode Manuel: Position JAUNE")
                position = self.yellow_positions[0]
                self.set_initial_pose(position['x'], position['y'], position['theta'], "yellow")
            elif 1.0 <= duration < 3.0:  # Appui moyen en mode manuel
                self.position_color = "blue"
                self.get_logger().info("Mode Manuel: Position BLEUE")
                position = self.blue_positions[0]
                self.set_initial_pose(position['x'], position['y'], position['theta'], "blue")
            else:  # Appui très long en mode manuel
                self.mode = "initial"
                self.get_logger().info("Retour au mode INITIAL")
                # Arrêter toute séquence manuelle en cours
                self.stop_manual_sequence()
                self.stop_robot()
                # Réinitialiser les étapes
                self.manual_sequence_step = 0
        
    
    def goal_switch_pressed(self):
        """Appelé quand le switch de goal est pressé"""
        self.goal_press_start_time = time.time()
        self.get_logger().info("Switch de goal appuyé")
    
    def goal_switch_released(self):
        """Appelé quand le switch de goal est relâché"""
        # Traitement en fonction du mode
        self.get_logger().info("Switch de goal relâché")
        
        if self.mode == "nav2":
            # Envoyer un goal de navigation en fonction de la couleur
            if self.position_color == "yellow":
                self.send_goal_pose(self.yellow_goal)
                self.get_logger().info('Goal jaune envoyé pour la navigation')
            else:  # blue
                self.send_goal_pose(self.blue_goal)
                self.get_logger().info('Goal bleu envoyé pour la navigation')
        
        elif self.mode == "manu":
            # Démarrer la séquence manuelle en fonction de la couleur
            self.get_logger().info(f"Démarrage de la séquence manuelle {self.position_color}")
            self.start_manual_sequence(self.position_color)
    
    def lidar_callback(self, msg):
        """Détecte les obstacles via le LIDAR (uniquement en mode manuel)"""
        if self.mode == "manu":
            distances = [d for d in msg.ranges if not math.isinf(d) and not math.isnan(d)]
            if distances and min(distances) < 0.25:
                if not self.obstacle_detected:
                    self.obstacle_detected = True
                    self.get_logger().warn("Obstacle détecté à moins de 20cm ! Arrêt du robot.")
                    self.stop_robot()
            else:
                self.obstacle_detected = False
    
    def timer_callback(self):
        """Gère les actions manuelles planifiées"""
        if not self.manual_sequence_active or self.mode != "manu":
            return
            
        # Gestion des actions en cours
        if self.current_action:
            if time.time() - self.action_start_time >= self.current_action['duration']:
                self.get_logger().info(f"Action terminée: {self.current_action['type']}")
                self.current_action = None
                self.stop_robot()  # S'assurer que le robot s'arrête complètement
                # Passer à l'étape suivante après un court délai
                # pour s'assurer que le robot est bien arrêté
                threading.Timer(0.1, self.execute_next_manual_step).start()
            elif self.obstacle_detected and self.current_action['type'] != 'stop':
                self.stop_robot()
                self.current_action = None
                self.get_logger().warn("Action interrompue par obstacle")
    
    def euler_to_quaternion(self, yaw):
        """
        Convertit un angle de lacet (yaw) en quaternion
        Pour une rotation autour de l'axe Z uniquement
        """
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qx, qy, qz, qw
    
    def create_pose(self, x, y, theta):
        """Crée une pose pour le goal de navigation"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convertir theta (angle en radians) en quaternion
        qx, qy, qz, qw = self.euler_to_quaternion(theta)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        
        return pose
    
    def set_initial_pose(self, x, y, theta, color):
        """Envoie la position initiale à AMCL"""
        self.get_logger().info(f'Définition de la position initiale ({color}): ({x}, {y}, {theta})')
        
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        
        # Orientation (quaternion pour représenter theta)
        qx, qy, qz, qw = self.euler_to_quaternion(theta)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        
        # Covariance (valeurs par défaut)
        msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        
        # Publication de la pose initiale
        self.initial_pose_pub.publish(msg)
    
    def send_goal_pose(self, pose):
        """Envoie un goal de navigation à Nav2"""
        # Attendre que le serveur d'action soit disponible
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('En attente du serveur de navigation...')
            return
        
        # Préparer l'action de navigation
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        # Envoi asynchrone du goal
        self.get_logger().info('Envoi du goal de navigation')
        self.nav_client.send_goal_async(goal_msg)
    
    def cancel_navigation(self):
        """Annule toute navigation en cours"""
        if self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Annulation de la navigation en cours")
            # Envoyer une demande d'annulation
            self.nav_client.cancel_all_goals()
    
    def stop_robot(self):
        """Arrête le mouvement du robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
    
    def start_movement(self, linear=0.0, angular=0.0, duration=1.0):
        """Démarre un mouvement avec une durée définie"""
        if self.obstacle_detected and linear > 0:
            self.get_logger().warn("Mouvement bloqué : obstacle présent.")
            return False
        
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_publisher.publish(twist)
        
        # Enregistrer l'action en cours
        self.current_action = {
            'type': 'move' if linear != 0 else 'turn' if angular != 0 else 'stop',
            'duration': duration,
            'linear': linear,
            'angular': angular
        }
        self.action_start_time = time.time()
        return True
    
    def move_forward(self, distance):
        """Avance d'une distance donnée"""
        speed = 0.2  # m/s
        duration = distance / speed
        return self.start_movement(linear=speed, duration=duration)
    
    def turn_left(self, angle_deg):
        """Tourne à gauche d'un angle donné"""
        angular_speed = 0.5  # rad/s
        angle_rad = angle_deg * math.pi / 180
        duration = angle_rad / angular_speed
        return self.start_movement(angular=angular_speed, duration=duration)
    
    def turn_right(self, angle_deg):
        """Tourne à droite d'un angle donné"""
        angular_speed = 0.5  # rad/s
        angle_rad = angle_deg * math.pi / 180
        duration = angle_rad / angular_speed
        return self.start_movement(angular=-angular_speed, duration=duration)
    
    def start_manual_sequence(self, color):
        """Démarre la séquence manuelle pour la couleur spécifiée"""
        if self.mode != "manu":
            return
            
        self.manual_sequence_active = True
        self.manual_sequence_step = 0
        self.position_color = color
        self.get_logger().info(f"Démarrage de la séquence MANUELLE pour position {color}")
        self.execute_next_manual_step()
    
    def stop_manual_sequence(self):
        """Arrête la séquence manuelle en cours"""
        self.manual_sequence_active = False
        self.current_action = None
        self.manual_sequence_step = 0  # Ajout de cette ligne
        self.stop_robot()
        self.get_logger().info("Séquence manuelle arrêtée")
    
    def execute_next_manual_step(self):
        """Exécute l'étape suivante de la séquence manuelle"""
        if not self.manual_sequence_active:
            return
            
        if self.position_color == "yellow":
            yellow_sequence = [
                lambda: self.move_forward(0.4),    # 400 mm
                lambda: self.turn_left(90),
                lambda: self.move_forward(0.8),    # 800 mm
                lambda: self.turn_right(90),
                lambda: self.move_forward(1.0),    # 1000 mm
                lambda: self.finish_manual_sequence()
            ]
            
            if self.manual_sequence_step < len(yellow_sequence):
                if yellow_sequence[self.manual_sequence_step]():
                    self.manual_sequence_step += 1
                    
        if self.position_color == "blue":
            blue_sequence = [
                lambda: self.move_forward(0.4),
                lambda: self.turn_right(90),
                lambda: self.move_forward(0.8),
                lambda: self.turn_left(90),
                lambda: self.move_forward(1.0),
                lambda: self.finish_manual_sequence()
            ]
            
            if self.manual_sequence_step < len(blue_sequence):
                if blue_sequence[self.manual_sequence_step]():
                    self.manual_sequence_step += 1
    
    def finish_manual_sequence(self):
        """Termine la séquence manuelle"""
        self.manual_sequence_active = False
        self.get_logger().info("Séquence manuelle terminée avec succès")
        return True

def main(args=None):
    rclpy.init(args=args)
    
    # Utiliser un exécuteur multi-thread pour éviter les blocages
    executor = MultiThreadedExecutor()
    node = ButtonControlNode()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Arrêt du programme")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 