# Paquete Basic Nav

## Descripción
Este paquete proporciona funcionalidades básicas de navegación para robots utilizando ROS 2 y Nav2, permitiendo la navegación autónoma mediante puntos de referencia (waypoints) predefinidos.

## Componentes Principales
- Scripts de navegación por waypoints
- Ejemplo de uso del Nav2 Simple Commander
- Configuración de puntos de referencia personalizables

## Uso Rápido
1. Compile el paquete: `colcon build --packages-select basic_nav`
2. Cargue el entorno: `source ~/tfm_ws/install/setup.bash`
3. Ejecute la navegación: `ros2 run basic_nav waypoints_navigation`

## Configuración
Los waypoints se definen en `config/waypoints.yaml` especificando coordenadas X, Y y orientación (Z, W).

# Paquete de Navegación y SLAM

## Descripción
Este paquete integra la pila de Navegación 2 (Nav2) con capacidades de Localización y Mapeo Simultáneos (SLAM) para robots móviles, proporcionando una solución completa para navegación autónoma en entornos desconocidos.

## Componentes Principales
- Configuraciones para Nav2 y algoritmos SLAM
- Archivos de lanzamiento para navegación y mapeo
- Directorio para almacenar mapas generados

## Uso Rápido
1. Compile el paquete: `colcon build --packages-select my_nav2_slam_pkg`
2. Cargue el entorno: `source ~/tfm_ws/install/setup.bash`
3. Inicie la navegación con SLAM: `ros2 launch my_nav2_slam_pkg navigation_with_slam.launch.py`
