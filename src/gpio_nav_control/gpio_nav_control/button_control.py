#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import RPi.GPIO as GPIO
import time
import threading

class ButtonControlNode(Node):
    def __init__(self):
        super().__init__('button_control_node')
        
        # Configuration des GPIO - Boutons connectés à la masse (ground)
        GPIO.setmode(GPIO.BCM)
        self.POSITION_BUTTON = 23  # Bouton pour définir la position
        self.GOAL_BUTTON = 24      # Bouton fin de course pour envoyer goal
        
        # Configuration des entrées avec pull-up interne
        # Les boutons seront à l'état LOW quand appuyés (connectés à GND)
        GPIO.setup(self.POSITION_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.GOAL_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Configuration des événements pour les boutons
        # Détection sur front descendant (FALLING) car bouton vers GND
        GPIO.add_event_detect(self.POSITION_BUTTON, GPIO.FALLING, callback=self.position_button_callback, bouncetime=300)
        
        # Pour le bouton fin de course NF (Normalement Fermé)
        # On détecte le front montant (RISING) car il sera ouvert quand activé
        GPIO.add_event_detect(self.GOAL_BUTTON, GPIO.RISING, callback=self.goal_button_callback, bouncetime=300)
        
        # Positions initiales (convertie avec (0,0) au centre et en mètres)
        self.blue_positions = [
            #{'x': -1.3, 'y': -0.125, 'theta': 0.0},  # Zone Gauche
            {'x': 0.275, 'y': -0.8, 'theta': 0.0}    # Zone en Bas
        ]
        self.yellow_positions = [
            #{'x': 1.3, 'y': -0.125, 'theta': 0.0},   # Zone Droite
            {'x': -0.275, 'y': -0.8, 'theta': 0.0}   # Zone en Bas
        ]
        
        # Goals d'arrivée
        self.blue_goal = self.create_pose(1.125, 0.8, 0.0)   # Zone d'arrivée bleue
        self.yellow_goal = self.create_pose(-1.125, 0.8, 0.0)  # Zone d'arrivée jaune
        
        # État du système
        self.current_position_index = 0
        self.position_color = "yellow"  # Position par défaut: jaune
        
        # Publishers et clients d'action
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            'initialpose', 
            10
        )
        
        self.nav_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose'
        )
        
        self.get_logger().info('Node démarré - En attente des actions sur les boutons')
        
    def position_button_callback(self, channel):
        """Gère les appuis sur le bouton de position (GPIO 23)"""
        self.get_logger().info('Bouton de position appuyé')
        
        # Mesure du temps d'appui pour distinguer appui court/long
        start_time = time.time()
        
        # Attente du relâchement du bouton
        while GPIO.input(self.POSITION_BUTTON) == GPIO.LOW:
            time.sleep(0.01)
            
        duration = time.time() - start_time
        
        # Détermine le type d'appui
        if duration < 1.0:  # Appui court (moins d'1s)
            self.position_color = "yellow"
            position = self.yellow_positions[self.current_position_index]
            self.set_initial_pose(position['x'], position['y'], position['theta'], self.position_color)
            self.get_logger().info(f'Position initiale jaune {self.current_position_index+1} définie')
        else:  # Appui long
            self.position_color = "blue"
            position = self.blue_positions[self.current_position_index]
            self.set_initial_pose(position['x'], position['y'], position['theta'], self.position_color)
            self.get_logger().info(f'Position initiale bleue {self.current_position_index+1} définie')
        
        # Passe à la position suivante pour le prochain appui
        self.current_position_index = (self.current_position_index + 1) % len(self.blue_positions)
    
    def goal_button_callback(self, channel):
        """Gère le relâchement du bouton fin de course (GPIO 24)"""
        self.get_logger().info('Bouton fin de course relâché - Envoi du goal')
        
        # Sélection du goal en fonction de la couleur
        if self.position_color == "yellow":
            self.send_goal_pose(self.yellow_goal)
            self.get_logger().info('Goal jaune envoyé')
        else:  # blue
            self.send_goal_pose(self.blue_goal)
            self.get_logger().info('Goal bleu envoyé')
    
    def create_pose(self, x, y, theta):
        """Crée une pose pour le goal de navigation"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0  # Orientation par défaut (sans rotation)
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
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        
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
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('En attente du serveur de navigation...')
        
        # Préparer l'action de navigation
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        # Envoi asynchrone du goal
        self.get_logger().info('Envoi du goal de navigation')
        self.nav_client.send_goal_async(goal_msg)
    
    def cleanup(self):
        """Nettoie les ressources GPIO à la fermeture"""
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = ButtonControlNode()
    
    try:
        # Utiliser un thread séparé pour spin pour éviter de bloquer la gestion des GPIO
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
        spin_thread.start()
        
        while rclpy.ok():
            # Boucle principale pour maintenir le programme actif
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("Arrêt du programme")
    finally:
        node.cleanup()
        rclpy.shutdown()
        if 'spin_thread' in locals() and spin_thread.is_alive():
            spin_thread.join()

if __name__ == '__main__':
    main()
