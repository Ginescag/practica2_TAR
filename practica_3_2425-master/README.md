# Práctica 3: Explorando nuevas herramientas

## Entrega
La entrega de esta práctica se realizará a través de la herramienta de 'Evaluación' de UaCloud. La misma debe ser una memoria en formato `.pdf` en la cual se encuentren las respuestas a las preguntas teóricas y a los ejercicios propuestos de las diferentes partes. Deberá tener el nombre de *Apellidos_Nombre.pdf*. Asimismo, esta práctica se puede hacer tanto **individual como en parejas** (en el caso de ser en parejas el nombre del `.pdf` debe ser el primer apellido y nombre de los dos integrantes y en la memoria también se deben añadir, Ej: `Ramirez_Tamai_Pujol_Paco.pdf`). En esta práctica se deben entregar también los códigos que se hayan generado para resolver los ejercicios. Podéis o bien compartir un enlace al repositorio de `GitHub` que estéis usando, o añadirnos como colaboradores (Nuestros usuarios de github son TamaiRamirez y bigpacopujol, aunque nos añadáis como colaboradores, añadid el enlace al repositorio en la memoria) o compartir enlace de `Drive`, como prefiráis. Por otro lado, es recomendable que grabéis la resolución de los ejercicios y compartáis los videos en la memoria a través de un enlace también, así se puede observar la correcta ejecución de los ejercicios.

## Parte 1: SLAM con Turtlebot 3

En esta parte se abordará la generación de mapas bidimensionales del entorno. Posteriormente, el mapa generado será empleado para la localización del robot. Para ello, se emplearán los paquetes predeterminados en el marco de trabajo ROS (Robot Operating System) para el proceso de mapeo.

El robot es el encargado de construir el mapa a medida que se desplaza. Se recomienda el uso de la teleoperación mediante teclado para el manejo del robot, ya que permite adaptar la velocidad de acuerdo a las necesidades del usuario y dirigir el robot de acuerdo a la dirección deseada. Es importante destacar que el robot no está capacitado para mapear áreas del entorno que no haya explorado previamente, y que la calidad del mapa se ve afectada negativamente por la celeridad con la que se desplaza el robot, con lo cual, se recomienda emplear una velocidad de movimiento moderada.

Previamente a la realización del mapeado de un entorno con el Turtlebot 3, se debe generar en primera instancia un paquete de ROS denominado `slam_pkg`. En este paquete se volcarán los ficheros y directorios que se encuentran en la carpeta `Parte_1` del presente repositorio.

Una vez compilado el paquete con los archivos y directorios mencionados, es necesario abrir múltiples terminales y ejecutar los siguientes comandos:

Terminal 1:
```bash
export TURTLEBOT3_MODEL=waffle
roslaunch slam_pkg maps.launch world_file:=/workspace/catkin_ws/src/slam_pkg/worlds/maze_2.world
```

Terminal 2:
```bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_slam turtlebot3_slam.launch
```

Terminal 3:
```bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

Como se puede observar, a partir de la información del LiDAR, conforme el robot se desplaza por el entorno, va recopilando datos para generar un mapa del entorno. Una vez que el robot haya completado el proceso de mapeado, para garantizar la conservación de esta información, se debe almacenar dicho mapa en un fichero. Para ello, se debe ejecutar el siguiente comando en una nueva terminal:

Terminal 4: 
```bash
roscd slam_pkg
mkdir Maps && cd Maps
rosrun map_server map_saver -f <nombre_mapa>
```

> Pregunta 1: Analiza el archivo `.yaml` y explica que significa cada uno de los campos que se muestran.

Una vez que se ha obtenido el mapa, este se puede emplear para localizar al robot (determinar su ubicación en tiempo real) y para navegar por el entorno. Este procedimiento implica desplazarse del punto actual del robot a un destino específico, planificar la trayectoria óptima y evitar colisiones con obstáculos en el camino. En la interfaz de comandos de `rviz`, se han asignado dos botones para gestionar estas funciones:

- `2D pose estimate`: Permite marcar la posición (clic con el ratón) y la orientación (arrastrar el ratón) donde se encuentra ahora mismo el robot.  Esto es necesario para poder inicializar el algoritmo de localización que es un filtro de partículas. La nube de "flechitas" verdes representa las posiciones y orientaciones más plausibles para el robot en el instante actual, si el algoritmo funciona bien esta nube irá siguiendo la posición real del robot en todo momento, y cuanto mejor localizado esté el robot más "condensada" estará la nube.

- `2D Nav Goal`: Permite marcar el punto al que queremos que se mueva el robot. Primero tenemos que asegurarnos de que el robot está localizado (que la "nube" de flechitas verdes está en torno a la posición real). Si hay una trayectoria posible, aparecerá dibujada en rviz y el robot se irá moviendo por ella.

No obstante, estos botones no funcionarán si el mapa no está cargado en memoria y los nodos de ROS necesarios para la planificación de trayectorias y evitación de obstáculos:

Terminal 1:
```bash
export TURTLEBOT3_MODEL=waffle
roslaunch slam_pkg maps.launch world_file:=/workspace/catkin_ws/src/slam_pkg/worlds/maze_2.world
```

Terminal 2:
```bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/workspace/catkin_ws/src/slam_pkg/Maps/<nombre_mapa>.yaml
```

> Pregunta 2: Investiga qué significa esa especie de "recuadro de colores" que aparece rodeando al robot cuando se pone a calcular la trayectoria y se va moviendo ¿qué significan los colores cálidos/frios?
>
> Pregunta 3: Investiga qué algoritmo usa ROS por defecto para calcular la trayectoria hasta el destino. Explica su funcionamiento lo más intuitivamente que puedas en aprox. 100-150 palabras (no el código línea por línea sino la idea de cómo funciona).
>
> Pregunta 4: Averigua cuáles son esos nodos que necesitamos cargar en memoria para que funcione la navegación, pon los nombres y describe brevemente el papel de cada uno en 1-2 frases.

### Ejercicios:

1. Como puedes observar, en la carpeta `worlds` proporcionada en el directorio `Parte_1`, existe un fichero llamado `muchos_obstaculos.world`. Prueba a mapear este entorno y responde a la siguiente pregunta:
    > Pregunta 2: ¿Observas diferencias en el mapeado respecto al primer entorno probado anteriormente?

2. Genera un entorno propio con obstáculos y con diferentes configuraciones. Asimismo, responde a la siguiente pregunta: 
    > Pregunta 3: ¿Crees que hay cierto tipo de entornos en los que funciona mejor? (espacios abiertos, espacios pequeños, pasillos,...)

3. Elige uno de los entornos, puede ser uno de los proporcionados o el que hayas generado en el ejercicio anterior y construye el mapa variando los parámetros del algoritmo. Varía al menos `particles`, `linearUpdate` y `angularUpdate`. En el wiki de ROS tienes la [lista completa de parámetros](https://wiki.ros.org/gmapping#slam_gmapping) (sección 4.1.4). Para modificar un parámetro podemos hacerlo de varias formas, una de ellas es modificando el `Parameter Server`.

    >Pregunta 4: ¿Cómo afectan estos parámetros en la generación del mapa?

    Para poder repetir la misma prueba variando los parámetros del algoritmo, es posible grabar los datos de los sensores en un fichero `rosbag`. Este último puede ser reproducido posteriormente tantas veces como sea necesario, como si se tratara de información que el robot recibe en tiempo real. De esta manera, es posible repetir un experimento múltiples veces utilizando los mismos datos de entrada.

    Un `bag` es un formato de archivo para almacenar la información de los mensajes que se mandan. Estos ficheros se crean principalmente a través de la herramienta rosbag, que se suscribe a uno o más topics y almacena los mensajes de datos de forma consecutiva. Este fichero se usa para reproducir lo que ha ocurrido durante una experimentación y también para poder procesar los datos adquiridos, analizar o visualizar estos datos. 

    Parámetros:
    - record: Graba en un fichero bag el contenido de los topics especificados.
    - info: Muestra un resumen del contenido de un fichero bag.
    - play: Reproduce el contenido de uno o más ficheros bag.
    - check: Comprueba si el fichero bag es reproducible en el sistema actual o si puede ser migrado a otro sistema.
    - fix: Repara los mensajes en un fichero bag de forma que se pueda reproducir en el sistema actual.
    - filter: Convierte un fichero bag utilizando expresiones de Python. 
    - compress: Comprime uno o más ficheros bag.
    - decompress: Descomprime uno o más ficheros bag.
    - reindex: Reindexa uno o más ficheros bag que esten corruptos.

    Para poder emplear esta herramienta, lo conveniente es generar una carpeta dentro del paquete de ROS donde estemos trabajando. en una terminal nueva, ejecuta:
    ```bash
    mkdir bagfiles && cd bagfiles
    ```
    Investiga como usar esta herramienta para guardar y ejecutar los archivos `bag`. Asimismo, contesta a las siguientes preguntas:
    > Pregunta 5: ¿Qué información puedo extraer de un fichero bag?
    >
    > Pregunta 6: ¿Se puede modificar la velocidad de reproducción del archivo? ¿Cómo se puede modificar? ¿Afecta a la resolución del mapa generado?

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


### Parte 2.1: Primeros Pasos con ROS 2

#### Creación de un espacio de trabajo

Como ya sabes de la Práctica 1, cuando se maneja el código fuente de ROS, resulta beneficioso realizarlo en un `workspace` (Espacio de trabajo). Para crear un espacio de trabajo ROS, lo único que debemos hacer es buscar el directorio donde deseamos establecer el espacio de trabajo y ejecutar las instrucciones que se presentan a continuación. Se aconseja establecerlo en la carpeta principal de nuestro contenedor de `Docker`, bajo el nombre de `ros2_ws`, de la forma siguiente:

```bash
mkdir -p ros2_ws/src
cd ros2_ws/
```

Previamente a la construcción del espacio de trabajo, es necesario resolver las dependencias de los paquetes. Es posible que se disponga de todas las dependencias tras la instalación, sin embargo, es recomendable verificar al menos la primera vez que se genera el espacio de trabajo. Para llevar a cabo el proceso, es necesario acceder al espacio de trabajo (`ros2_ws`) y ejecutar el comando siguiente:

```bash
sudo rosdep init
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
```

Desde la raíz del espacio de trabajo (`ros2_ws`), ahora puedes construir sus paquetes utilizando el comando:
```bash
colcon build
```

Como puedes observar, el primer cambio respecto a ROS 1 es que ya no se usa la herramienta `catkin`.

> Pregunta 1: Dentro del espacio de trabajo, ¿Qué cambios notas respecto al espacio de trabajo en ROS Noetic?

A continuación, es necesario superponer este espacio de trabajo al entorno raíz de ROS 2, para ello ejecuta desde la raíz del espacio de trabajo (`ros2_ws`): 
```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

**IMPORTANTE**: Cada vez que entremos en el contenedor de ROS 2 ya sea ejecutando `./run.sh` (o si usamos nvidia-docker `./run_nvidia.sh`) o si accedemos al contenedor, una vez lanzado por el script anterior, en nuevas terminales para ejecutar diferentes herramientas de ROS ejecutando `./connect_ros.sh`, necesitamos que nuestro sistema tenga conocimiento de las variables del entorno. Para evitar tener que ejecutar siempre los mismos comandos constantemente cada vez que iniciamos el contenedor o generamos una nueva terminal, debemos ejecutar lo siguiente:

1. Configurar el archivo `~/.bashrc` para que automáticamente establezca las variables de entorno en nuestro espacio de trabajo `catkin_ws`:
```bash
nano ~/.bashrc
```

2. Dentro de este archivo en la última línea escribiremos lo siguiente:
```bash
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/local_setup.bash
```

3. Guardaremos el archivo actualizado. No obstante, esto no asegura que cada vez que ejecutemos de nuevo el contenedor, esta actualización se haya guardado. Esto se debe a que el contenedor se resetea cada vez que se cierra para evitar actualizarse automáticamente y que pueda que haya incompatibilidades en algún momento con algún cambio que hagamos, de esta forma, si hay algo que hemos tocado que haga una incompatibilidad, con cerrar y volver a lanzar el contenedor sería suficiente. En este caso, sí que nos interesa que el cambio se mantenga, por tanto, abriremos una nueva terminal (sin haber salido y cerrado el contenedor) y ejecutaremos el siguiente comando:
```bash
docker commit ros_humble ros_humble:latest
```
Y ya estaría guardado para siempre. (**NOTA:** Esto sirve a nivel genérico como se ha explicado para hacer que el contenedor mantenga todos los cambios. En el caso de archivos que se van a generar para las prácticas, esto no es necesario porque los archivos que se generen estarán guardados en nuestro dispositivo local y no íntegramente en nuestro contenedor únicamente.)

#### Crear un Paquete de ROS
Al igual que en ROS Noetic, un paquete es una unidad organizativa. La creación de paquetes en ROS 2 utiliza `ament` como su sistema de construcción y `colcon` como su herramienta de construcción. Puedes crear un paquete usando `CMake` o `Python`, que son los soportados oficialmente, aunque existen otros tipos de construcción. Para crear un paquete, ejecuta los siguientes comandos:

```bash
cd /workspace/ros2_ws/src
ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name> --dependencies rclpy std_msgs
```

Para compilar exclusivamente el paquete creado ejecuta:
```bash
cd ..
colcon build --packages-select <package_name>
source install/local_setup.bash
```

> Pregunta 2: De nuevo, los archivos que se crean dentro del paquete son diferentes a los que se creaban en ROS Noetic, ¿Donde se ubicarán ahora los nodos que creemos en el paquete? 
>
> Pregunta 3: ¿Qué son los archivos `package.xml` y `setup.py`? ¿Tienen alguna similitud con algún archivo de los paquetes de ROS Noetic?

#### Ejercicios
1. Crea un paquete llamado `service_temp`, este paquete deberá albergar un servicio de ROS 2. Este servicio se encargará de realizar conversiones de temperatura, concretamente de Grados Celcius a Farenheit y viceversa. El funcionamiento es el siguiente:
    - `Servidor`: Recibe el valor de temperatura y realiza la conversión.
    - `Cliente`: Envía el valor de temperatura y muestra el resultado obtenido desde el servidor.

Para realizar este ejercicio, se recomienda que revises la documentación de ROS 2 acerca de los [Servicios](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html), para que puedas entender las diferencias respecto a ROS Noetic. Asimismo, revisa como crear tus propios archivos [`.srv`](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html).

En este caso, el archivo `.srv` debería tener el siguiente contenido:
```
# Request
float64 input_temp
---
# Response
float64 converted_temp
string conversion_type # Cel_to_Far o Far_to_Cel
```

Asimismo, contesta a las siguientes preguntas:
> Pregunta 4: ¿Dónde van los archivos `.srv` en un paquete ROS 2?
>
> Pregunta 5: ¿Cuáles son las principales diferencias en cómo se implementan y ejecutan los servicios entre ROS Noetic y ROS 2 Humble?
>
> Pregunta 6: ¿En qué se diferencia `rclpy` en ROS 2 de `rospy` en cuanto al manejo de servicios?
>
> Pregunta 7: ¿Cuál es el comportamiento del cliente del servicio en ROS 2 cuando el servidor aún no está disponible?

2. Implementar un sistema basado en acciones ROS 2 utilizando Python, para ello deberás crear un nuevo paquete llamado `battery_act`, puedes encontrar la info necesaria en el siguiente [enlace](https://docs.ros.org/en/humble/Tutorials/Intermediate/Creating-an-Action.html) y en el [siguiente](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html). Esta acción se va a encargar de simular un proceso de descarga de la batería de un robot, enviando información sobre el progreso y devolviendo un resultado al finalizar. El objetivo es establecer un valor de batería en el cual el robot debe enviar un aviso de "batería baja" para proceder a su carga. El funcionamiento de la acción sería el siguiente: 
    - El nodo `Servidor` llamado `battery_charger`, recibirá el `Goal` que será el valor de batería, en tanto porcentual, en el cual el robot deberá mandar el aviso (Ej: 20%). Por tanto, el robot partirá de un 100% de batería y esta se irá reduciendo en un 5% por cada segundo que pase. El `Servidor` deberá ir actualizando el valor del `Feedback` mostrando el valor actual de la batería del robot. Asimismo, el `Servidor` devolverá como `Result` un aviso (Ej: "Batería Baja, por favor cargue el robot!"). Ten en cuenta que la acción se debe poder cancelar si se manda la petición de cancelación.
    - El nodo `Cliente` llamado `battery_client`, enviará el `Goal` al servidor teniendo en cuenta que este valor se recogerá desde la línea de comandos al lanzar el nodo. El nodo `Cliente` deberá ir publicando el `Feedback` de la acción, así como el mensaje del `Result` una vez haya finalizado la acción.

En base al ejercicio, el archivo `.action` deberá tener el siguiente contenido:
```
# Goal
int32 target_percentage
---
# Result
string warning
---
# Feedback
int32 current_percentage
```

Asimismo, contesta a las siguientes preguntas:
> Pregunta 8: Describe la arquitectura de una acción en ROS 2. ¿En qué se diferencia del sistema basado en `actionlib` de ROS 1?
>
> Pregunta 9: ¿Qué ocurre internamente cuando se cancela la acción en ROS 2? ¿En qué se diferencia con ROS Noetic?
>
> Pregunta 10: ¿Cómo mejora el uso de DDS en ROS 2 la fiabilidad y escalabilidad de la comunicación basada en acciones en comparación con ROS 1?



