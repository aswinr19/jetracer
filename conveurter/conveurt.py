import subprocess
import shutil
import os

# Define the directory containing config.json
in_directory = "./jetracer"
in_urdf_file = "robot.urdf"
out_urdf_dir = "../urdf"
out_urdf_file = "jetracer.urdf"
out_stl_dir = "../meshes"

try:
    # Clear the output directories
    if os.path.exists(out_urdf_dir):
        shutil.rmtree(out_urdf_dir)
    os.makedirs(out_urdf_dir)

    if os.path.exists(out_stl_dir):
        shutil.rmtree(out_stl_dir)
    os.makedirs(out_stl_dir)

    # Run the onshape-to-robot command
    subprocess.run(["onshape-to-robot", in_directory], check=True)
    print("convert urdf for ros description")

    # Open the file robot.urdf and replace substrings
    with open(in_directory + "/" + in_urdf_file, "r") as file:
        urdf_content = file.read()

    urdf_content = urdf_content.replace("package:///", "package://jetracer_description/meshes/")
    urdf_content = urdf_content.replace('<joint_properties friction="0.0"/>', '<dynamics friction="0.0"/>')

    with open(os.path.join(out_urdf_dir, out_urdf_file), "w") as file:
        file.write(urdf_content)

    print("Move converted files for ros")

    # Copy all STL files from in_directory to out_stl_dir
    for file_name in os.listdir(in_directory):
        if file_name.endswith(".stl"):
            full_file_name = os.path.join(in_directory, file_name)
            if os.path.isfile(full_file_name):
                shutil.copy(full_file_name, out_stl_dir)
    print("STL files copied successfully")

except subprocess.CalledProcessError as e:
    print(f"An error occurred while running the onshape-to-robot command: {e}")
except FileNotFoundError as e:
    print(f"File not found: {e}")
except IOError as e:
    print(f"An I/O error occurred: {e}")
except Exception as e:
    print(f"An unexpected error occurred: {e}")
