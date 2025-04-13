# AUTHOR: GINES CABALLERO GUIJARRO 54639204J

#!/usr/bin/env python3
import cv2
import numpy as np
import sys
import os

# Parámetros de escalado en el mundo de Gazebo

RESOLUTION = 0.1  # metros por pixel (ejemplo)
WALL_HEIGHT = 0.5  # altura de la pared en metros
WALL_THICKNESS = 0.05  # grosor de la pared en metros
WORLD_FILENAME = 'mazePng2World.world'
IMAGE_FILENAME = 'laberinto.png'  # Tu imagen en blanco y negro

def generate_gazebo_world_from_image(image_path, output_world_path):
    # Lee la imagen en escala de grises
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"No se pudo leer la imagen {image_path}")

    # Binariza la imagen (umbral 127 por ejemplo)
    # Todo lo que sea menor a 127 -> negro (pared), mayor -> blanco (espacio)
    _, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

    # Dimensiones de la imagen
    height, width = binary.shape

    # Plantilla básica del .world
    # Puedes modificar la iluminación, ground_plane, etc. a tu gusto
    world_header = """<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default_world">

    <!-- Un cielo por defecto -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Un piso por defecto -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Empieza la definición de obstáculos -->
"""

    world_footer = """
  </world>
</sdf>
"""

    # En esta sección generamos un "modelo" o “link” para cada pared
    boxes_str = ""
    box_count = 0

    for row in range(height):
        for col in range(width):
            # Si es una pared
            if binary[row, col] == 0:
                # Calcula posición (x, y) en Gazebo
                x = col * RESOLUTION
                y = (height - row) * RESOLUTION  # invertimos el eje Y para que no aparezca al revés

                # Creamos un bloque
                model_name = f"wall_{box_count}"
                boxes_str += f"""
    <model name="{model_name}">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{WALL_THICKNESS} {WALL_THICKNESS} {WALL_HEIGHT}</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <friction>
              <ode/>
            </friction>
            <contact>
              <ode/>
            </contact>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{WALL_THICKNESS} {WALL_THICKNESS} {WALL_HEIGHT}</size>
            </box>
          </geometry>
        </visual>
        <pose>{x} {y} {WALL_HEIGHT/2} 0 0 0</pose>
      </link>
    </model>
                """
                box_count += 1

    # Armamos el archivo .world final
    world_contents = world_header + boxes_str + world_footer

    # Escribimos en el fichero
    with open(output_world_path, 'w') as f:
        f.write(world_contents)

    print(f"Se ha generado el archivo {output_world_path} con {box_count} boxes para el laberinto.")


if __name__ == "__main__":

  image = sys.argv[1] if len(sys.argv) > 1 else sys.exit("Uso: python png2world.py <imagen.png>")

  generate_gazebo_world_from_image(image, WORLD_FILENAME)
