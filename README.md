# ROS Workspace Source Tree

This branch is intended to publish the contents of `src/` as the repository root.

Packages in this workspace:

- `rgbd_person_tracker` — detección/tracking multi-persona RGB-D (YOLO + MediaPipe + Kalman), publica `/person_tracks`. Ver su [README](rgbd_person_tracker/README.md).
- `rgbd_sim` — simulación Gazebo con cámara RGB-D estática y modelo `person_target`.
- `person_tracking` — tracker legado basado en PCL (nube de puntos 3D), independiente del pipeline RGB-D.
- `turtlebot3_fm2_nav` — navegación TurtleBot3 con planificador FM2 (fast marching). Consume `/person_tracks` en `fm2_costmap_node.py` para tratar personas como obstáculos dinámicos (con predicción por velocidad constante y auto-limpieza por timeout). Incluye `fm2_nav_moving_person.launch`, una demo con una persona patrullando el escenario para validar la reacción del planificador ante obstáculos móviles.

Use this branch for the source packages only. Build artifacts should stay outside Git.
