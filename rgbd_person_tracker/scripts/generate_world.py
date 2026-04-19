#!/usr/bin/env python3
"""
generate_world.py — Genera un fichero .world de Gazebo con N actores animados.

Uso:
  python3 generate_world.py <num_persons> <output_path>

Cada actor recibe una trayectoria distinta (figure-8, círculo, línea recta,
zigzag) separada en Y para que no se solapen entre sí.
"""

import sys
import math

# ═══════════════════════════════════════════════════════════════════════
# Generadores de trayectorias (devuelven lista de waypoints)
# Cada waypoint: (time, x, y, yaw)
# ═══════════════════════════════════════════════════════════════════════

def _lemniscate(cx, cy, amp_x=0.75, amp_y=1.5, period=16.0, n_pts=12):
    """Trayectoria en forma de 8 (lemniscate)."""
    wps = []
    for i in range(n_pts + 1):
        t_param = 2.0 * math.pi * i / n_pts
        x = cx + amp_x * math.sin(2.0 * t_param)
        y = cy + amp_y * math.sin(t_param)
        # Calcular yaw mirando hacia el siguiente punto
        t_next = 2.0 * math.pi * (i + 1) / n_pts
        x_next = cx + amp_x * math.sin(2.0 * t_next)
        y_next = cy + amp_y * math.sin(t_next)
        yaw = math.atan2(y_next - y, x_next - x)
        time_s = period * i / n_pts
        wps.append((time_s, x, y, yaw))
    return wps, period


def _circle(cx, cy, radius=1.2, period=14.0, n_pts=12):
    """Trayectoria circular."""
    wps = []
    for i in range(n_pts + 1):
        theta = 2.0 * math.pi * i / n_pts
        x = cx + radius * math.cos(theta)
        y = cy + radius * math.sin(theta)
        yaw = theta + math.pi / 2.0  # tangente al círculo
        time_s = period * i / n_pts
        wps.append((time_s, x, y, yaw))
    return wps, period


def _straight_line(cx, cy, length=3.0, period=10.0, n_pts=6):
    """Trayectoria recta ida y vuelta."""
    wps = []
    half = n_pts // 2
    # Ida
    for i in range(half + 1):
        frac = i / half
        x = cx + frac * length
        y = cy
        yaw = 0.0
        time_s = (period / 2.0) * frac
        wps.append((time_s, x, y, yaw))
    # Vuelta
    for i in range(1, half + 1):
        frac = i / half
        x = cx + length * (1.0 - frac)
        y = cy
        yaw = math.pi
        time_s = period / 2.0 + (period / 2.0) * frac
        wps.append((time_s, x, y, yaw))
    return wps, period


def _zigzag(cx, cy, amp_x=2.0, amp_y=1.0, period=12.0, n_pts=8):
    """Trayectoria en zigzag."""
    wps = []
    for i in range(n_pts + 1):
        frac = i / n_pts
        x = cx + amp_x * frac
        y = cy + amp_y * (1.0 if (i % 2 == 0) else -1.0)
        # yaw hacia el siguiente punto
        if i < n_pts:
            frac_n = (i + 1) / n_pts
            x_n = cx + amp_x * frac_n
            y_n = cy + amp_y * (1.0 if ((i + 1) % 2 == 0) else -1.0)
            yaw = math.atan2(y_n - y, x_n - x)
        else:
            yaw = wps[-1][3] if wps else 0.0
        time_s = period * frac
        wps.append((time_s, x, y, yaw))
    return wps, period


_TRAJECTORY_GENERATORS = [_lemniscate, _circle, _straight_line, _zigzag]


# ═══════════════════════════════════════════════════════════════════════
# Generador de actor SDF
# ═══════════════════════════════════════════════════════════════════════

def _actor_sdf(actor_name, trajectory_index, center_x, center_y):
    """Genera el bloque XML <actor> para un actor."""
    gen = _TRAJECTORY_GENERATORS[trajectory_index % len(_TRAJECTORY_GENERATORS)]
    waypoints, _period = gen(center_x, center_y)

    wp_lines = []
    for (t, x, y, yaw) in waypoints:
        wp_lines.append(
            "          <waypoint>\n"
            "            <time>{:.2f}</time>\n"
            "            <pose>{:.3f} {:.3f} 0 0 0 {:.3f}</pose>\n"
            "          </waypoint>".format(t, x, y, yaw)
        )

    return (
        '    <actor name="{name}">\n'
        '      <skin>\n'
        '        <filename>walk.dae</filename>\n'
        '        <scale>1.0</scale>\n'
        '      </skin>\n'
        '      <animation name="walking">\n'
        '        <filename>walk.dae</filename>\n'
        '        <scale>1.0</scale>\n'
        '        <interpolate_x>true</interpolate_x>\n'
        '      </animation>\n'
        '      <script>\n'
        '        <loop>true</loop>\n'
        '        <delay_start>{delay:.1f}</delay_start>\n'
        '        <auto_start>true</auto_start>\n'
        '        <trajectory id="0" type="walking">\n'
        '{waypoints}\n'
        '        </trajectory>\n'
        '      </script>\n'
        '    </actor>\n'
    ).format(
        name=actor_name,
        delay=trajectory_index * 1.0,   # stagger start
        waypoints="\n".join(wp_lines),
    )


# ═══════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════

_WORLD_TEMPLATE = """\
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="tracker_scene">

    <!-- Suelo y luz -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

{actors}
  </world>
</sdf>
"""


def main():
    # Filtrar argumentos de remapping de ROS (__name:=..., __log:=..., etc.)
    args = [a for a in sys.argv[1:] if not a.startswith("__") and ":=" not in a]

    if len(args) < 2:
        print("Uso: generate_world.py <num_persons> <output_path>")
        sys.exit(1)

    n = int(args[0])
    output_path = args[1]

    if n < 1:
        print("Error: num_persons debe ser >= 1")
        sys.exit(1)

    # Distribuir actores en el espacio frente a la cámara.
    # La cámara está en (0, 0, 1.2) mirando hacia +X.
    # Centramos los actores en X≈3..5 y los separamos en Y.
    y_spread = 2.0  # separación en Y entre actores
    y_offset = -(n - 1) * y_spread / 2.0  # centrar en Y=0

    actors_sdf = []
    for i in range(n):
        cx = 3.0 + (i % 2) * 1.5          # alternar X para variedad
        cy = y_offset + i * y_spread       # separar en Y
        traj_idx = i                       # distinta trayectoria por persona
        name = "walking_person_{}".format(i)
        actors_sdf.append(_actor_sdf(name, traj_idx, cx, cy))

    world_content = _WORLD_TEMPLATE.format(actors="\n".join(actors_sdf))

    with open(output_path, "w") as f:
        f.write(world_content)

    print("[generate_world] Escrito {} con {} actores → {}".format(
        output_path, n, output_path))


if __name__ == "__main__":
    main()
