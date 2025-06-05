from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import rclpy
import yaml
import time
import os
from ament_index_python.packages import get_package_share_directory

def read_waypoints(file_path):
    with open(file_path, 'r') as f:
        waypoints = yaml.safe_load(f)
    return waypoints

def main():
    rclpy.init()
    
    navigator = BasicNavigator()

    # Espera hasta que el sistema de navegación esté listo
    navigator.waitUntilNav2Active()

    # Get the full path to the waypoints file
    waypoints_path = os.path.join(
        get_package_share_directory('basic_nav'),
        'config',
        'waypoints.yaml'
    )
    
    # Leer waypoints desde el archivo
    waypoints = read_waypoints(waypoints_path)

    for idx, wp in enumerate(waypoints):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = wp['x']
        goal_pose.pose.position.y = wp['y']
        goal_pose.pose.orientation.z = wp['z']
        goal_pose.pose.orientation.w = wp['w']

        # Enviar objetivo al robot
        navigator.goToPose(goal_pose)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                print(f"Dist. restante al WP {idx + 1}: {feedback.distance_remaining:.2f} m")
            time.sleep(1)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"Objetivo {idx + 1} alcanzado.")
            # Aquí puedes añadir código para que el robot hable o solicite input
        else:
            print(f"Fallo al alcanzar el objetivo {idx + 1}.")
            break

    # Lógica para cambiar de planificador local si es necesario
    # changePlanner('nombre_del_planificador')

    rclpy.shutdown()
    return 0  # Add an explicit return value

if __name__ == '__main__':
    main()

# Add this line at the end to ensure main is accessible at module level
__all__ = ['main']
