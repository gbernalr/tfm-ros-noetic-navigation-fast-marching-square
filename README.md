El estado del proyecto es el siguiente.

Instalé Ubuntu 20.04 y ROS1 Noetic en una nueva partición en un disco duro. Tras configurar todo el entorno me dediqué a recordar comandos de ROS1 Noetic, así como; mapeado, navegación, etc...

Pasé a estudiar el Fast Marching Square un poco por mi cuenta para entender el algoritmo, y tras hacerme una idea más robusta de cómo funciona, fuí al proyecto de Matlab que usa Fast Marching con ROS para poder jugar un poco con la ruta y diferentes parámetros para entenderlo de una manera más práctica.

Después de ello me lancé a probar la implementación en python. Tras esto, quise implementar mi propio nodo de ROS que haga uso de este FM2 como librería pero para llevar a cabo mi navegación. Usando el turtlebot3 de gazebo, la navegación del paquete de navegación de turtlebot3 (matando el nodo de /move_base para que su planificador no opaque al mío). Creé a partir de esto un nodo auxiliar de creación de CostMap donde básicamente se coge el mapa y la información del sensor láser para poder inferir un mapa donde se detecten posibles obstáculos activamente pensado para obstáculos dinámicos como teníamos pensado.
El segundo de los nodos es el de navegación. En este nodo, se suscribe al topic donde publica el mapa el anterior nodo, y a partir de él y de la librería fm2, calcula el camino de puntos a seguir y su matriz de velocidades que el robot adoptará, lo hace continuamente o, al menos, a la velocidad que nos podemos permitir actualmente.

De momento solo he probado con el primer mapa de turtlebot3: roslaunch turtlebot3_gazebo turtlebot3_world.launch

El comportamiento en este mapa es bastante robusto adjunto vídeo mostrando la ejecución y comportamiento.

Cabe resaltar que he migrado toda la librería de fm2 de python3.10 a python3.8 para que pueda usarse con ros noetic de manera sencilla.

Ahora mismo hay varios frentes que afrontar en este estado:

- El principal problema es que no funciona esta navegación dinámica: si añado objeto que obstruyan el camino en el mundo, el mapa costmap que recibe mi nodo los recoge bien, pero por alguna razón el fm2 decide pasar por encima de ellos. He llegado al punto de creer con casi toda certeza que el problema es de la propia librería y no de alguno de mis dos nodos

- Esto se suma a que a veces, cuando esta cerca de un obstáculo estático, que ya pertenecía al mapa, también planifica pasando sobre ellos pero esto sucede con mucha menos frecuencia.

- Intenté mapear el mapa de la casa de turtlebot3 y tras hacerlo mi robot no navegaba bien, pero como estaba en un estado en el que aún no funcionaba bien en el mundo default, decidí dejarlo para después.

- Crear el launch que lance el gazebo, rviz, mi navegación y nodo de costmap

- Medir el tiempo de cómputo del nodo y algoritmo fm2 exclusivamente en cada iteración de cáculo de ruta para ver cómo mejorar estos tiempos.

- Ajuste al llegar al punto con un local_planner simplón. Actualmente el que tengo es que cuando al llegar o estar muy cerca del goal, dejo de planificar y me dedico a alinear mi yaw con el que se impuso en el topic de goal. Pero no sé si es la mejor opción o debería usar uno nativo del paquete turtlebot3.

Código en el siguiente repositorio: 


Aún no he explorado el robot físico.