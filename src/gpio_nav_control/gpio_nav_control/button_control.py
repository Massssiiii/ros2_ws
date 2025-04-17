#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from gpiozero import Button
import time
import threading

class ButtonControlNode(Node):
    def __init__(self):
        super().__init__('button_control_node')
        
        # Configuration des GPIO avec gpiozero
        # Les boutons sont connectés à GND donc pull_up=True
        self.position_button = Button(23, pull_up=True, bounce_time=0.1)
        self.goal_switch = Button(24, pull_up=True, bounce_time=0.1)
        
        # Ajout des gestionnaires d'événements
        self.position_button.when_pressed = self.position_button_pressed
        self.goal_switch.when_released = self.goal_switch_released  # Déclenche quand le switch est relâché (n'est plus à False)
        
        # Variables pour mesurer la durée d'appui
        self.press_start_time = 0
        
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
        
    def position_button_pressed(self):
        """Appelé quand le bouton de position est pressé"""
        self.press_start_time = time.time()
        
        # On ajoute un gestionnaire pour le relâchement
        self.position_button.when_released = self.position_button_released
    
    def position_button_released(self):
        """Appelé quand le bouton de position est relâché"""
        # Calcul de la durée d'appui
        duration = time.time() - self.press_start_time
        
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
    
    def goal_switch_released(self):
        """Appelé quand le bouton/switch de goal est relâché (n'est plus à FALSE)"""
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

def main(args=None):
    rclpy.init(args=args)
    node = ButtonControlNode()
    
    try:
        # Lancer le nœud ROS2
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Arrêt du programme")
    finally:
        # Nettoyage explicite non nécessaire avec gpiozero
        rclpy.shutdown()

if __name__ == '__main__':
    main()