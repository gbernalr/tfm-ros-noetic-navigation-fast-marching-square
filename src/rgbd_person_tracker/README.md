# rgbd_person_tracker

Tracking **multi-persona** en tiempo real mediante **detección YOLO + estimación de pose 2D (MediaPipe)** y **cámara RGB-D simulada**, con filtro de Kalman y asociación húngara.

---

## Descripción general

Este paquete ROS implementa un pipeline completo de detección y seguimiento de personas a partir de imágenes RGB-D. A diferencia del paquete `person_tracking` (que trabaja directamente sobre nubes de puntos 3D), este enfoque:

1. **Detecta personas** en la imagen RGB usando **YOLOv8** (bounding boxes).
2. **Estima el esqueleto 2D** de cada persona detectada ejecutando **MediaPipe Pose** sobre cada recorte (ROI).
3. **Selecciona un punto anatómico de referencia** (pelvis, torso o tobillos) con prioridad jerárquica.
4. **Lee la profundidad** de forma robusta (mediana de un parche, con filtrado de outliers).
5. **Backprojection a 3D** usando los intrínsecos de la cámara.
6. **Proyecta al plano del suelo** mediante la cadena TF (`camera_depth_optical_frame` → `world`).
7. **Filtra y trackea** con un filtro de Kalman lineal de 4 estados (x, y, vx, vy) y asociación húngara (scipy).
8. **Visualiza** resultados como topics ROS (imagen anotada con bounding boxes, vista cenital, markers 3D para RViz).

---

## Arquitectura

```
┌──────────────┐     ┌──────────────┐
│  /camera/     │     │  /camera/     │
│  color/       │     │  depth/       │
│  image_raw    │     │  image_raw    │
└──────┬───────┘     └──────┬───────┘
       │                     │
       └────────┬────────────┘
                │  ApproximateTimeSynchronizer
                ▼
       ┌────────────────────┐
       │  Person Detector    │  (YOLOv8 → bboxes)
       │ (person_detector)   │
       └───────┬────────────┘
               ▼
       ┌────────────────────┐
       │  Pose Detector      │  (MediaPipe Pose per ROI)
       │  (pose_detector)    │
       └───────┬────────────┘
               ▼
       ┌────────────────────┐
       │ Reference Selector  │  pelvis > torso > ankles
       │(reference_selector) │
       └───────┬────────────┘
               ▼
       ┌────────────────┐
       │  Depth Utils    │  robust_depth + backproject_to_3d
       │  (depth_utils)  │
       └───────┬────────┘
               ▼
       ┌────────────────────┐
       │  Ground Projector   │  TF2: optical_frame → world
       │ (ground_projector)  │
       └───────┬────────────┘
               ▼
       ┌────────────────────┐
       │  Kalman Tracker     │  predict → associate (Hungarian)
       │  (kalman_tracker)   │  → update → create/prune
       └───────┬────────────┘
               ▼
       ┌────────────────┐
       │   Visualizer    │  Image + TopView + MarkerArray
       │  (visualizer)   │
       └────────────────┘
```

---

## Módulos Python

Todos los módulos están en `src/rgbd_person_tracker/`:

### `person_detector.py`
Wrapper de **YOLOv8** para detección multi-persona. Ejecuta `YOLO('yolov8n.pt').predict()` filtrando solo la clase `person` (ID 0). Devuelve una lista de `PersonBBox(x1, y1, x2, y2, conf)` para cada persona detectada en la imagen.

### `pose_detector.py`
Pipeline **YOLO + MediaPipe** multi-persona. Para cada bounding box detectada por `PersonDetector`:

1. Recorta la región de interés (ROI) con un **10% de margen** adicional.
2. Ejecuta **MediaPipe Pose** en modo `static_image_mode=True` sobre el recorte.
3. Re-proyecta los keypoints al sistema de coordenadas de la imagen completa.

Extrae los keypoints relevantes: `left_hip`, `right_hip`, `left_shoulder`, `right_shoulder`, `left_ankle`, `right_ankle`. Cada keypoint tiene coordenadas (u, v) en píxeles y una confianza de visibilidad.

> **Multi-persona**: Gracias a la detección previa con YOLO, se pueden rastrear múltiples personas simultáneamente.

### `reference_selector.py`
Implementa la selección jerárquica del punto anatómico de referencia:

| Prioridad | Nombre   | Fuente                              | Criterio                            |
|-----------|----------|-------------------------------------|-------------------------------------|
| 1         | `pelvis` | Media de left_hip + right_hip       | Ambas caderas > umbral confianza    |
| 2         | `torso`  | Media de ≥ 2 de {caderas, hombros}  | Al menos 2 puntos > umbral          |
| 3         | `ankles` | Media de left_ankle + right_ankle   | Ambos tobillos > umbral confianza   |

Devuelve `(u, v)`, la confianza mínima de los puntos usados, y el tipo.

### `depth_utils.py`
Dos funciones:

- **`robust_depth(depth_img, u, v, patch_size, depth_min, depth_max)`**: Extrae un parche cuadrado alrededor del píxel `(u, v)`, filtra valores inválidos (NaN, fuera de rango) y calcula la **mediana**. Si menos del 40% de los píxeles son válidos, retorna `None`. Calcula una **confianza** basada en la desviación estándar: más dispersión → menos confianza.

- **`backproject_to_3d(u, v, z, K)`**: Des-proyecta un píxel con profundidad `z` a un punto 3D en el frame óptico de la cámara usando los intrínsecos `K = {fx, fy, cx, cy}`:
  ```
  X = (u - cx) * z / fx
  Y = (v - cy) * z / fy
  Z = z
  ```

### `ground_projector.py`
Usa **TF2** para transformar puntos 3D del frame óptico de la cámara (`camera_depth_optical_frame`) al frame mundo (`world`). El resultado es la coordenada `(x, y)` en el plano del suelo. No necesita una homografía explícita porque la cadena TF ya proporciona la transformación completa.

### `kalman_tracker.py`
Implementación completa del tracker multi-objeto:

- **`Detection`**: Estructura con posición en suelo, confianzas, covarianza de medida, referencia 2D y tipo.

- **`Track`**: Estado Kalman con:
  - Estado: `[x, y, vx, vy]` — posición y velocidad en el suelo
  - Modelo de transición: velocidad constante
  - Matrices: `A` (4×4), `H` (2×4), `Q` (4×4), `P` (4×4)
  - Métodos `predict()` y `update(z, R)`
  - Contadores de edad, hits, misses y flag `confirmed`

- **`build_measurement_cov(conf_pose, conf_depth)`**: Genera una covarianza de medida `R` (2×2) adaptativa. Combina las confianzas de pose y profundidad para interpolar entre σ = 0.05 m (alta confianza) y σ = 0.40 m (baja confianza).

- **`TrackerManager`**: Orquesta el ciclo completo en cada frame:
  1. **Predicción** de todos los tracks existentes
  2. **Asociación** detecciones ↔ tracks mediante **algoritmo húngaro** (`scipy.optimize.linear_sum_assignment`) con gating por distancia euclidiana
  3. **Actualización** de tracks emparejados (Kalman update con `R` adaptativa)
  4. **Manejo de tracks no observados** (solo predicción)
  5. **Creación de nuevos tracks** para detecciones no asignadas
  6. **Eliminación de tracks muertos** (demasiados misses consecutivos)

### `visualizer.py`
Publica tres topics de visualización:

1. **Imagen anotada** (`~image_annotated`): Imagen RGB con **bounding boxes** YOLO, círculos en el punto de referencia anatómico y los IDs de tracks confirmados.
2. **Vista cenital** (`~top_view`): Canvas 2D tipo bird's-eye con fondo oscuro, grid métrico, posición de la cámara, trayectorias como polilíneas, posición actual como círculo, y flechas de velocidad.
3. **Markers 3D** (`~markers`): `MarkerArray` para RViz con esferas de posición, texto de ID, líneas de historial y flechas de velocidad, todo en el frame `world`.

---

## Topics

### Suscripciones

| Topic                          | Tipo                   | Descripción                    |
|-------------------------------|------------------------|--------------------------------|
| `/camera/color/image_raw`      | `sensor_msgs/Image`    | Imagen RGB de la cámara        |
| `/camera/depth/image_raw`      | `sensor_msgs/Image`    | Mapa de profundidad            |
| `/camera/color/camera_info`    | `sensor_msgs/CameraInfo` | Intrínsecos (leído una vez)  |

### Publicaciones

| Topic                                      | Tipo                      | Descripción                         |
|-------------------------------------------|---------------------------|-------------------------------------|
| `~image_annotated`                         | `sensor_msgs/Image`       | Imagen RGB con anotaciones          |
| `~top_view`                                | `sensor_msgs/Image`       | Vista cenital de trayectorias       |
| `~markers`                                 | `MarkerArray`             | Markers 3D para RViz               |

---

## Parámetros configurables

Todos los parámetros se cargan desde `config/tracker_params.yaml` vía el servidor de parámetros de ROS:

| Parámetro                     | Default | Descripción                                                |
|------------------------------|---------|------------------------------------------------------------|
| `pose_conf_threshold`         | 0.4     | Umbral de confianza mínima para aceptar un keypoint        |
| `depth_min`                   | 0.5     | Profundidad mínima válida (metros)                         |
| `depth_max`                   | 8.0     | Profundidad máxima válida (metros)                         |
| `patch_size`                  | 5       | Tamaño del parche para profundidad robusta                 |
| `max_assoc_dist`              | 1.0     | Distancia máxima para asociar detección ↔ track (metros)   |
| `max_misses`                  | 10      | Frames consecutivos sin observación antes de borrar track  |
| `min_hits_to_confirm`         | 3       | Frames necesarios para confirmar un track                  |
| `kalman/q_pos`                | 0.05    | Ruido de proceso — posición                                |
| `kalman/q_vel`                | 0.20    | Ruido de proceso — velocidad                               |
| `mediapipe/min_detection_confidence` | 0.5 | Confianza mínima de detección MediaPipe                |
| `mediapipe/min_tracking_confidence`  | 0.5 | Confianza mínima de tracking MediaPipe                 |
| `mediapipe/model_complexity`  | 0       | Complejidad del modelo (0=lite, 1=full, 2=heavy)           |
| `yolo/model`                  | `yolov8n.pt` | Modelo YOLO para detección de personas            |
| `yolo/conf_threshold`         | 0.4     | Confianza mínima de detección YOLO                         |
| `yolo/device`                 | `cpu`   | Dispositivo de inferencia (`cpu`, `0` para GPU)            |
| `yolo/imgsz`                  | 320     | Resolución de entrada YOLO (320=rápido, 640=preciso)       |
| `process_every_n`             | 2       | Procesar detección cada N frames (Kalman predice el resto) |
| `optical_frame`               | `camera_depth_optical_frame` | Frame TF del sensor óptico              |
| `world_frame`                 | `world` | Frame TF de referencia global                              |

---

## Instalación

### Dependencias del sistema

```bash
# ROS (ya instalado si tienes el workspace)
sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-tf2-geometry-msgs

# Python
pip install mediapipe scipy numpy opencv-python ultralytics
```

### Compilar

```bash
cd ~/tfm/intro_ws
catkin_make
source devel/setup.bash
```

---

## Uso

### Demo completa (Gazebo + Tracker + RViz)

```bash
roslaunch rgbd_person_tracker sim_tracker.launch
```

Esto lanza:
1. **Gazebo** con la cámara RGB-D estática y un actor humano animado (mesh `walk.dae`) describiendo una trayectoria en **figura de ocho** (lemniscata).
2. **El nodo tracker** que procesa las imágenes y publica los resultados.
3. **RViz** con la configuración predefinida mostrando markers 3D, imagen anotada y vista cenital.

### Solo el tracker (si ya hay una fuente RGB-D corriendo)

```bash
roslaunch rgbd_person_tracker tracker.launch
```

### Sin GUI de Gazebo

```bash
roslaunch rgbd_person_tracker sim_tracker.launch gui:=false
```

### Sin RViz (solo publicar topics)

```bash
roslaunch rgbd_person_tracker sim_tracker.launch rviz:=false
```

---

## Optimización de rendimiento

El nodo imprime un log periódico con los tiempos por etapa:
```
[perf] total=85ms  det=75ms  trk=0ms  vis=10ms  (det=ON, #trk=1, skip_n=2)
```

### Estrategias aplicadas

| Técnica | Descripción |
|---------|-------------|
| **`process_every_n=2`** | Solo ejecuta YOLO + MediaPipe en 1 de cada N frames. En los demás, el filtro de Kalman predice la posición sin nueva observación. Reduce la carga de detección a la mitad (o más). |
| **`yolo/imgsz=320`** | YOLO procesa imágenes escaladas a 320px en vez de 640. ~4× más rápido con mínima pérdida de precisión a distancias <8m. |
| **`mediapipe/model_complexity=0`** | Modelo *lite* de MediaPipe, ~2× más rápido que `model_complexity=1`. |
| **Lazy visualization** | Las imágenes anotada y top-view solo se generan cuando hay un suscriptor activo (RViz/rqt). |

### Ajustes adicionales (según hardware)

```yaml
# Si tienes GPU NVIDIA:
yolo:
  device: "cuda:0"    # inferencia YOLO en GPU

# Para máxima velocidad (menor precisión):
process_every_n: 3
yolo:
  imgsz: 256

# Para máxima precisión (más lento):
process_every_n: 1
yolo:
  imgsz: 640
mediapipe:
  model_complexity: 1
```

---

## Algoritmo — Resumen teórico

### Filtro de Kalman lineal

El estado del tracker es $\mathbf{x} = [x, y, v_x, v_y]^T$ — posición y velocidad en el plano del suelo.

**Modelo de transición** (velocidad constante):

$$
\mathbf{x}_{k+1} = \mathbf{A}\,\mathbf{x}_k + \mathbf{w}_k, \quad
\mathbf{A} = \begin{bmatrix} 1 & 0 & \Delta t & 0 \\ 0 & 1 & 0 & \Delta t \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

**Modelo de observación**:

$$
\mathbf{z}_k = \mathbf{H}\,\mathbf{x}_k + \mathbf{v}_k, \quad
\mathbf{H} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix}
$$

### Covarianza de medida adaptativa

La covarianza $\mathbf{R}$ se adapta según la confianza combinada de pose y profundidad:

$$
c = 0.5 \cdot c_{\text{pose}} + 0.5 \cdot c_{\text{depth}}, \quad
\sigma = 0.05 + 0.35 \cdot (1 - c), \quad
\mathbf{R} = \sigma^2 \, \mathbf{I}_2
$$

### Asociación húngara

Se construye una **matriz de costes** basada en la distancia euclidiana entre la posición predicha de cada track y la posición observada de cada detección. Se aplica **gating**: si la distancia supera `max_assoc_dist`, el coste se pone a infinito. El **algoritmo húngaro** (`scipy.optimize.linear_sum_assignment`) resuelve la asignación óptima en tiempo polinómico.

### Ciclo de vida de un track

```
[Nuevo] ──(hits >= min_hits_to_confirm)──► [Confirmado] ──(misses > max_misses)──► [Eliminado]
```

- Un track nuevo se crea cuando una detección no se asocia a ningún track existente.
- Se confirma tras `min_hits_to_confirm` frames con asociación exitosa.
- Se elimina tras `max_misses` frames consecutivos sin observación.
- Solo los tracks confirmados se visualizan.

---

## Estructura del paquete

```
rgbd_person_tracker/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── README.md
├── config/
│   ├── tracker_params.yaml      # Parámetros configurables
│   └── rgbd_tracker.rviz        # Configuración de RViz
├── launch/
│   ├── tracker.launch           # Solo el nodo tracker
│   └── sim_tracker.launch       # Gazebo + tracker + RViz
├── worlds/
│   └── tracker_scene.world      # Escena Gazebo: actor walk.dae en figura de ocho
├── scripts/
│   └── rgbd_person_tracker_node.py   # Nodo principal ROS
└── src/
    └── rgbd_person_tracker/
        ├── __init__.py
        ├── person_detector.py    # YOLOv8 detección multi-persona
        ├── pose_detector.py      # YOLO + MediaPipe Pose por ROI
        ├── reference_selector.py # Selección punto anatómico
        ├── depth_utils.py        # Profundidad robusta + backprojection
        ├── ground_projector.py   # Proyección 3D → suelo vía TF2
        ├── kalman_tracker.py     # Kalman + Hungarian + TrackerManager
        └── visualizer.py         # Publicadores de visualización ROS
```
