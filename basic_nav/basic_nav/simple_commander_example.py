from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import rclpy
import time

def main():
    try:
        # Inicializa el sistema ROS 2
        rclpy.init()
        
        # Crear el navegador (sin pasar un nodo explícito)
        navigator = BasicNavigator()

        # Definir la pose objetivo
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = 4.43
        goal_pose.pose.position.y = -11.7
        goal_pose.pose.orientation.w = 1.0

        # Enviar el objetivo al robot
        navigator.goToPose(goal_pose)

        # Esperar hasta que la tarea esté completa
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback and hasattr(feedback, 'distance_remaining'):
                print(f"Distancia restante al objetivo: {feedback.distance_remaining:.2f} m")
            time.sleep(1)

        # Obtener y mostrar el resultado
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:  # Use imported TaskResult
            print("El robot alcanzó el objetivo.")
        elif result == TaskResult.CANCELED:
            print("La navegación fue cancelada.")
        else:
            print("La navegación falló.")

    except Exception as e:
        print(f"Error en la navegación: {e}")
    finally:
        # Asegurarse de que siempre se cierre ROS 2
        rclpy.shutdown()

if __name__ == '__main__':
    main()
