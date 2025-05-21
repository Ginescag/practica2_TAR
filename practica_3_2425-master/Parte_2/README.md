# Práctica 3: Explorando nuevas herramientas

## Parte 2: Migrando a ROS 2 (Humble)

### Instalación

Para comenzar con la instalación de ROS 2 (Humble) dentro de un contenedor `Docker` se requiere abrir una terminal dentro de la carpeta `Parte_2` de este repositorio y ejecutar los siguientes comandos:

1. Crear y construir la imágen de ROS 2 (Humble) a través del `Dockerfile`:
```bash
docker build --build-arg USERNAME=$(whoami)-docker --build-arg USER_UID=$(id -u) -t ros_humble:latest .
```
Este comando comenzará la instalación de ROS en su Distro Humble.

2. Una vez instalado ROS 2 ya podemos ejecutar el contenedor y conectarnos a él de la siguiente manera **sin usar Nvidia-Docker**:
```bash
sudo chmod u+x run.sh # Solo la primera vez para dar los permisos necesarios
./run.sh 
```
Si tenemos `nvidia-docker` instalado, podemos hacer que ROS 2 emplee la GPU de nuestro dispositivo. En ese caso ejecuta lo siguiente:
```bash
sudo chmod u+x run_nvidia.sh # Solo la primera vez para dar los permisos necesarios
./run_nvidia.sh 
```
Ya estaremos dentro del contenedor y podremos empezar a trabajar con ROS 2.

3. Para conectarnos al contenedor desde nuevas terminales una vez lanzado el contenedor, ejecutar:
```bash
sudo chmod u+x connect_ros.sh # Solo la primera vez para dar los permisos necesarios
./connect_ros.sh
```

**IMPORTANTE**: Cada vez que entremos en el contenedor de ROS ya sea ejecutando `./run.sh` (o si usamos nvidia-docker `./run_nvidia.sh`) o si accedemos al contenedor, una vez lanzado por el script anterior, en nuevas terminales para ejecutar diferentes herramientas de ROS ejecutando `./connect_ros.sh`, necesitamos que nuestro sistema tenga conocimiento de las variables del entorno. Para evitar tener que ejecutar siempre los mismos comandos constantemente cada vez que iniciamos el contenedor o generamos una nueva terminal, debemos ejecutar lo siguiente:

1. Configurar el archivo `~/.bashrc` para que automáticamente establezca las variables de entorno en nuestro espacio de trabajo `catkin_ws`:
```bash
nano ~/.bashrc
```

2. Dentro de este archivo en la última línea escribiremos lo siguiente:
```bash
source /workspace/catkin_ws/devel/setup.bash
```

### Primeros Pasos con Ros 2

Como ya sabes de la Práctica 1, cuando se maneja el código fuente de ROS, resulta beneficioso realizarlo en un `workspace` (Espacio de trabajo). Para crear un espacio de trabajo ROS, lo único que debemos hacer es buscar el directorio donde deseamos establecer el espacio de trabajo y ejecutar las instrucciones que se presentan a continuación. Se aconseja establecerlo en la carpeta principal de nuestro contenedor de `Docker`, bajo el nombre de `ros2_ws`, de la forma siguiente:

```bash
mkdir -p ros2_ws/src
cd ros2_ws/
```

Previamente a la construcción del espacio de trabajo, es necesario resolver las dependencias de los paquetes. Es posible que el usuario disponga de todas las dependencias, sin embargo, es recomendable verificar al menos la primera vez que se genera el espacio de trabajo. Para llevar a cabo el proceso, es necesario acceder al entorno de trabajo (`ros2_ws`) y ejecutar el comando siguiente:

```bash
cd ..
rosdep install -i --from-path src --rosdistro humble -y
```

Desde la raíz de su espacio de trabajo (`ros2_ws`), ahora puede construir sus paquetes utilizando el comando:
```bash
colcon build
```

Como puedes observar, el primer cambio respecto a ROS 1 es qeu ya no se usa la herramienta `catkin`.