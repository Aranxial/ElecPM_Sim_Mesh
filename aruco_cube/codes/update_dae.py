import sys
import os
from lxml import etree

def update_dae_texture(dae_file, number):
    # Base path for textures
    base_texture_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../Aruco_tags')
    texture_filename = os.path.join(base_texture_path, f"{number:02d}{str(dae_file[-5])}.png")

    # Parse the DAE file
    tree = etree.parse(dae_file)
    root = tree.getroot()
    
    # Find the <image> element and update its <init_from> child with the new texture path
    namespace = {"collada": "http://www.collada.org/2005/11/COLLADASchema"}
    for image in root.findall(".//collada:image", namespaces=namespace):
        init_from = image.find("collada:init_from", namespaces=namespace)
        if init_from is not None:
            init_from.text = texture_filename

    # Save the modified DAE file
    tree.write(dae_file)

    print(f"Updated {dae_file} with texture {texture_filename}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <number>")
        sys.exit(1)

    number = int(sys.argv[1])

    # List of .dae files to be updated
    base_path = os.path.dirname(os.path.abspath(__file__))
    dae_files = [
        os.path.join(base_path, '../meshes/face_1.dae'),
        os.path.join(base_path, '../meshes/face_2.dae'),
        os.path.join(base_path, '../meshes/face_3.dae'),
        os.path.join(base_path, '../meshes/face_4.dae'),
        os.path.join(base_path, '../meshes/face_5.dae'),
        os.path.join(base_path, '../meshes/face_6.dae')
    ]

    # Update each .dae file with the corresponding texture
    for dae_file in dae_files:
        update_dae_texture(dae_file, number)

