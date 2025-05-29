cmake_minimum_required(VERSION 3.5)
project(battery_act)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Genera las interfaces de la acción (nombre exacto del archivo)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Battery.action"
)

# Expone las interfaces generadas a tiempo de ejecución
ament_export_dependencies(rosidl_default_runtime)

# Instala como paquete Python (busca __init__.py en battery_act/)
ament_python_install_package(${PROJECT_NAME})

# Instala los ejecutables Python
install(
  PROGRAMS
    battery_act/battery_charger.py
    battery_act/battery_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
