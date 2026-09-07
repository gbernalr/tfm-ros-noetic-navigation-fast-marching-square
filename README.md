# ROS Workspace Source Tree

This branch is intended to publish the contents of `src/` as the repository root.

Packages in this workspace:

- `rgbd_person_tracker` — detección/tracking multi-persona RGB-D (YOLO + MediaPipe + Kalman), publica `/person_tracks`. Ver su [README](rgbd_person_tracker/README.md).
- `rgbd_sim` — simulación Gazebo con cámara RGB-D estática y modelo `person_target`.
- `person_tracking` — tracker legado basado en PCL (nube de puntos 3D), independiente del pipeline RGB-D.
- `turtlebot3_fm2_nav` — navegación TurtleBot3 con planificador FM2 (fast marching). Consume tracks y predicciones para tratar personas como obstáculos dinámicos. Incluye `fm2_nav_moving_person.launch`, una demo con una persona patrullando el escenario para validar la reacción del planificador ante obstáculos móviles.

## Predicción de trayectorias

`rgbd_person_tracker/scripts/trajectory_predictor_node.py` consume
`/person_tracks`, mantiene historial por `track_id` y publica
`/person_predictions`. La primera implementación es un baseline de velocidad
constante ajustado sobre el historial y añade incertidumbre creciente por
horizonte. `fm2_costmap_node.py` usa esa salida para pintar elipses de
ocupación; si aún no hay historial suficiente o el predictor deja de publicar,
vuelve automáticamente a la predicción de velocidad constante de los tracks.

Los launchers `fm2_nav.launch` y `fm2_nav_moving_person.launch` ya arrancan el
predictor. Un modelo aprendido puede sustituir el contenido del predictor sin
cambiar `/person_predictions`.

`fm2_nav_moving_person.launch` usa ahora una ZED virtual: el nodo
`zed_body_tracking_sim_node.py` aplica FOV, rango, frecuencia, ruido, latencia,
pérdidas y tracking a los modelos de persona de Gazebo antes de publicar
`/person_tracks`. No entrega el ground truth de forma directa.

Use this branch for the source packages only. Build artifacts should stay outside Git.
