#!/usr/bin/env python3
"""
Genera un archivo SDF con N actores humanos, cada uno con trayectoria en 8 o circular.
Uso:
  python3 generate_multi_person_world.py N > tracker_scene_N.world
"""
import sys
import math

N = int(sys.argv[1]) if len(sys.argv) > 1 else 1

header = '''<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="tracker_scene">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
'''

actor_template = '''
    <actor name="walking_person_{i}">
      <skin>
        <filename>walk.dae</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walking">
        <filename>walk.dae</filename>
        <scale>1.0</scale>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>0.0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walking">
{waypoints}
        </trajectory>
      </script>
    </actor>
'''

def lemniscate_waypoints(cx, cy, phase=0.0, period=16.0, radius=0.75, yscale=1.5, n=13):
    wps = []
    for k in range(n):
        t = 2 * math.pi * k / (n-1)
        t_shift = t + phase
        x = cx + radius * math.sin(2*t_shift)
        y = cy + yscale * math.sin(t_shift)
        yaw = math.atan2(
            yscale * math.cos(t_shift),
            2 * radius * math.cos(2*t_shift)
        )
        time = period * k / (n-1)
        wps.append(f"          <waypoint>\n            <time>{time:.2f}</time>\n            <pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.3f}</pose>\n          </waypoint>")
    return '\n'.join(wps)

print(header)
for i in range(N):
    # Espaciado en círculo
    angle = 2 * math.pi * i / N
    cx = 3.0 + 1.5 * math.cos(angle)
    cy = 0.0 + 1.5 * math.sin(angle)
    phase = angle  # desfase para que no se solapen
    waypoints = lemniscate_waypoints(cx, cy, phase=phase)
    print(actor_template.format(i=i, waypoints=waypoints))
print("  </world>\n</sdf>")
