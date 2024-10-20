import os
import cv2
import numpy as np

def generate_sdf_from_image(image_path, sdf_output_path, map_size=(100, 100), height=1.0, wall_height=1.0):
    # Load the image
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    height_img, width_img = image.shape

    # Calculate the pixel to meter ratio
    pixel_to_meter = map_size[0] / width_img

    image = cv2.flip(image, 0)
    # Threshold the image to binary (black and white)
    _, binary_image = cv2.threshold(image, 128, 255, cv2.THRESH_BINARY_INV)

    # Find contours
    contours, _ = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    sdf_lines = []
    sdf_lines.append('<sdf version="1.6">')
    sdf_lines.append('<world name="default">')


    # add some boilerplate plugins for gazebo
    sdf_lines.append('  <physics name="1ms" type="ignored">')
    sdf_lines.append('    <max_step_size>0.001</max_step_size>')
    sdf_lines.append('    <real_time_factor>1.0</real_time_factor>')
    sdf_lines.append('  </physics>')
    sdf_lines.append('  <plugin')
    sdf_lines.append('    filename="gz-sim-physics-system"')
    sdf_lines.append('    name="gz::sim::systems::Physics">')
    sdf_lines.append('  </plugin>')
    sdf_lines.append('  <plugin')
    sdf_lines.append('    filename="gz-sim-user-commands-system"')
    sdf_lines.append('    name="gz::sim::systems::UserCommands">')
    sdf_lines.append('  </plugin>')
    sdf_lines.append('  <plugin')
    sdf_lines.append('    filename="gz-sim-scene-broadcaster-system"')
    sdf_lines.append('    name="gz::sim::systems::SceneBroadcaster">')
    sdf_lines.append('  </plugin>')
    sdf_lines.append('  <light type="directional" name="sun">')
    sdf_lines.append('    <cast_shadows>true</cast_shadows>')
    sdf_lines.append('    <pose>0 0 10 0 0 0</pose>')
    sdf_lines.append('    <diffuse>0.8 0.8 0.8 1</diffuse>')
    sdf_lines.append('    <specular>0.2 0.2 0.2 1</specular>')
    sdf_lines.append('    <attenuation>')
    sdf_lines.append('      <range>1000</range>')
    sdf_lines.append('      <constant>0.9</constant>')
    sdf_lines.append('      <linear>0.01</linear>')
    sdf_lines.append('      <quadratic>0.001</quadratic>')
    sdf_lines.append('    </attenuation>')
    sdf_lines.append('    <direction>-0.5 0.1 -0.9</direction>')
    sdf_lines.append('  </light>')


    # set the grid size for the map
    sdf_lines.append('  <scene>')
    sdf_lines.append('    <grid>true</grid>')
    sdf_lines.append('    <ambient>0.4 0.4 0.4 1</ambient>')
    sdf_lines.append('  </scene>')

    # Create the boundary wall
    for i in range(4):
        x, y, w, h = 0, 0, 0, 0
        if i == 0:
            x, y, w, h = 0, 0, width_img, wall_height
        elif i == 1:
            x, y, w, h = 0, 0, wall_height, height_img
        elif i == 2:
            x, y, w, h = 0, height_img - wall_height, width_img, wall_height
        elif i == 3:
            x, y, w, h = width_img - wall_height, 0, wall_height, height_img

        sdf_lines.append(f'  <model name="wall_{i}">')
        sdf_lines.append('    <static>true</static>')
        sdf_lines.append('    <link name="wall_link">')
        sdf_lines.append('      <visual name="visual">')
        sdf_lines.append('        <geometry>')
        sdf_lines.append('          <box>')
        sdf_lines.append(f'            <size>{w * pixel_to_meter} {h * pixel_to_meter} {height}</size>')
        sdf_lines.append('          </box>')
        sdf_lines.append('        </geometry>')
        sdf_lines.append('      </visual>')
        sdf_lines.append('      <collision name="collision">')
        sdf_lines.append('        <geometry>')
        sdf_lines.append('          <box>')
        sdf_lines.append(f'            <size>{w * pixel_to_meter} {h * pixel_to_meter} {height}</size>')
        sdf_lines.append('          </box>')
        sdf_lines.append('        </geometry>')
        sdf_lines.append('      </collision>')
        sdf_lines.append(f'      <pose>{(x + w / 2) * pixel_to_meter} {(y + h / 2) * pixel_to_meter} {height / 2} 0 0 0</pose>')
        sdf_lines.append('    </link>')
        sdf_lines.append('  </model>')

    # Create obstacles based on contours
    for contour in contours:
        # Get a list of the walls of the contour
        walls = []
        for i in range(len(contour) - 1):
            walls.append((contour[i ][0], contour[i + 1][0]))
            print(contour[i], contour[i + 1])

        sdf_lines.append(f'  <model name="obstacle{contour}">')
        sdf_lines.append('    <static>true</static>')
        sdf_lines.append('    <link name="wall_link">')
        sdf_lines.append('      <visual name="visual">')
        sdf_lines.append('        <geometry>')
        sdf_lines.append('          <polyline>')
        sdf_lines.append('            <height>0.1</height>')
        for wall in walls:
            x1, y1 = wall[0][0], wall[0][1]
            x2, y2 = wall[1][0], wall[1][1]
            sdf_lines.append('            <point>')
            sdf_lines.append(f'              {x1 * pixel_to_meter} {y1 * pixel_to_meter} 0')
            sdf_lines.append(f'              {x2 * pixel_to_meter} {y2 * pixel_to_meter} 0')
            sdf_lines.append('            </point>')
        sdf_lines.append('          </polyline>')
        # sdf_lines.append('          <box>')
        # sdf_lines.append(f'            <size>{w * pixel_to_meter} {h * pixel_to_meter} {height}</size>')
        # sdf_lines.append('          </box>')
        sdf_lines.append('        </geometry>')
        sdf_lines.append('      </visual>')
        # sdf_lines.append('      <collision name="collision">')
        # sdf_lines.append('        <geometry>')
        # sdf_lines.append('          <box>')
        # sdf_lines.append(f'            <size>{w * pixel_to_meter} {h * pixel_to_meter} {height}</size>')
        # sdf_lines.append('          </box>')
        # sdf_lines.append('        </geometry>')
        # sdf_lines.append('      </collision>')
        sdf_lines.append('    </link>')
        sdf_lines.append('  </model>')

    sdf_lines.append('</world>')
    sdf_lines.append('</sdf>')

    # Write the SDF content to the output file
    with open(sdf_output_path, 'w') as sdf_file:
        sdf_file.write('\n'.join(sdf_lines))

def get_all_files(directory):
    # List all files and directories in the given directory
    all_entries = os.listdir(directory)
    # Filter out only the files
    files = [entry for entry in all_entries if os.path.isfile(os.path.join(directory, entry))]
    return files

if __name__=="__main__":
    # get all files in the 2d_maps/2d_map_images folder
    files = []

    # Example usage
    directory_path = '2d_maps/images'
    files = get_all_files(directory_path)

    # for each file, generate the sdf
    for file in files:
        world_name = file.split('.')[0]
        generate_sdf_from_image(f'{directory_path}/{file}', f'2d_maps/sdf_worlds/{world_name}.sdf')
