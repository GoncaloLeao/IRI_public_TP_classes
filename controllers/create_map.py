"""
Script used to create Webots maps from a YAML file.
By: Gonçalo Leão
"""
import yaml
from PIL import Image
import numpy as np
import csv

if __name__ == '__main__':
    custom_maps_filepath: str = '../worlds/custom_maps/'
    map_name: str = 'obstacles'

    # Parse the YAML file
    yaml_filepath: str = custom_maps_filepath + map_name + '_config.yaml'
    with open(yaml_filepath, 'r') as stream:
        yaml_data = yaml.safe_load(stream)

    # Process the image to get the coords of the wall pixels
    image_filename: str = yaml_data['image']
    resolution: float = yaml_data['resolution']
    origin: [float, float, float] = yaml_data['origin']
    occupied_thresh: float = yaml_data['occupied_thresh']
    max_pixel_value_for_wall: int = int(255*occupied_thresh)
    wall_pixels_coords: [(int, int)] = []

    img: Image = Image.open(custom_maps_filepath + image_filename).convert('L')

    np_img: np.array = np.array(img)
    height: int = len(np_img)
    width: int = len(np_img[0])
    for row in range(len(np_img)):
        for col in range(len(np_img[row])):
            if np_img[row][col] <= max_pixel_value_for_wall:
                wall_pixels_coords.append((origin[0] + resolution*col, origin[1] + resolution*(height - row)))

    print('num walls = ', len(wall_pixels_coords))

    # Create and save the coords file
    f = open(custom_maps_filepath + map_name + '_points.csv', 'w', newline='')
    writer = csv.writer(f)
    #    Add the borders
    for x in range(width):
        writer.writerow((origin[0] + resolution*x, origin[1]))
        writer.writerow((origin[0] + resolution*x, origin[1] + resolution*(height - 1)))
    for y in range(1, width-1):
        writer.writerow((origin[0], origin[1] + resolution*y))
        writer.writerow((origin[0] + resolution*(width - 1), origin[1] + resolution*y))
    #    Add the walls
    for coord in wall_pixels_coords:
        writer.writerow(coord)
    f.close()

    # Create and save the new Webots file
    base_map_webots_filepath: str = custom_maps_filepath + 'base_map.wbt'
    f = open(base_map_webots_filepath, 'r')
    webots_str: str = f.read()
    f.close()

    map_webots_filepath: str = custom_maps_filepath + map_name + '.wbt'
    f = open(map_webots_filepath, 'w')
    f.write(webots_str)

    #    Add the rectangular arena
    f.write('RectangleArena {')
    f.write('  translation ' + str(origin[0] + resolution*width/2) + ' ' + str(origin[1] + resolution*height/2) + ' 0.0')
    f.write('  floorSize ' + str(resolution*width) + ' ' + str(resolution*height))
    f.write('  floorTileSize 0.25 0.25')
    f.write('  floorAppearance Parquetry {')
    f.write('    type "light strip"')
    f.write('  }')
    f.write('  wallHeight 0.05')
    f.write('}')

    #    Add the walls
    index: int = 0
    for coord in wall_pixels_coords:
        f.write('Solid {')
        f.write('    translation ' + str(coord[0]) + ' ' + str(coord[1]) + ' 0.025')
        f.write('    children [')
        f.write('        Shape {')
        f.write('            geometry Box {')
        f.write('                size ' + str(resolution) + ' ' + str(resolution) + ' 0.05')
        f.write('            }')
        f.write('        }')
        f.write('    ]')
        f.write('    name "solid' + str(index) + '"')
        # f.write('    boundingObject Box {')
        # f.write('        size ' + str(resolution) + ' ' + str(resolution) + ' 0.05')
        # f.write('    }')
        f.write('}')
        """
        Solid {
            translation -0.54 -0.06 0.025
            children [
                Shape {
                    geometry Box {
                        size 0.125 0.125 0.05
                    }
                }
            ]
            name "solid1"
            boundingObject Box {
                size 0.125 0.125 0.05
            }
        }
        """
        index += 1
    f.close()



