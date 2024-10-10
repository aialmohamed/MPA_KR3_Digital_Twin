import os
import trimesh
import numpy as np

# Define the directory path containing the STL files
stl_directory = "/home/robolab/MPA_Repo/MPA_KR3_Digital_Twin/Software/ROS2_Env/kr3r540_ws/src/kr3r540_description/meshes/collision/"

# Dictionary to define specific mass values for particular STL files
mass_values = {
    "link_3.stl": 3.0,
    "link_5.stl": 1.0,
    "link_6.stl": 0.03
}

# Function to calculate and display inertia properties
def calculate_inertia_properties(mesh, mass):
    # Calculate the original properties
    center_of_mass = [0.0, 0.0, 0.0]  # New origin specified as [0, 0, 0]
    inertia_tensor = mesh.moment_inertia * (mass / mesh.volume)  # Scale inertia tensor to match new mass

    # Extract the principal moments of inertia
    ixx, iyy, izz = np.diag(inertia_tensor)

    # Check the triangle inequality condition
    inequality_check = (ixx + iyy > izz) and (ixx + izz > iyy) and (iyy + izz > ixx)

    # Generate URDF friendly inertia parameters
    urdf_inertia = f"""
<inertial>
    <origin xyz="{center_of_mass[0]} {center_of_mass[1]} {center_of_mass[2]}" rpy="0 0 0"/>
    <mass value="{mass}"/>
    <inertia ixx="{ixx}" ixy="{inertia_tensor[0][1]}" ixz="{inertia_tensor[0][2]}"
            iyy="{iyy}" iyz="{inertia_tensor[1][2]}" izz="{izz}"/>
</inertial>
"""

    return mass, center_of_mass, inertia_tensor, inequality_check, urdf_inertia

# Iterate through all STL files in the directory
for stl_file in os.listdir(stl_directory):
    if stl_file in mass_values:  # Only process specific STL files
        stl_path = os.path.join(stl_directory, stl_file)
        try:
            # Load the STL file using trimesh
            mesh = trimesh.load(stl_path)

            # Calculate inertia properties using the specified mass
            mass, center_of_mass, inertia_tensor, inequality_check, urdf_inertia = calculate_inertia_properties(mesh, mass_values[stl_file])

            # Print the results for each file
            print(f"STL File: {stl_file}")
            print(f"Adjusted Mass: {mass}")
            print(f"Center of Mass: {center_of_mass}")
            print(f"Inertia Tensor: {inertia_tensor}")
            print(f"Triangle Inequality Satisfied: {inequality_check}")
            print("Generated URDF Inertial Parameters:")
            print(urdf_inertia)
            print("\n-----------------------------------------\n")

        except Exception as e:
            print(f"Error processing {stl_file}: {e}")
