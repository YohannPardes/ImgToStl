import os
import time
import numpy as np
from stl import mesh
from PIL import Image
import open3d as o3d

def image_to_3d_model(image_path, output_stl, max_height=3.5, min_height=0.15, max_size_cm=12):
    # Load the grayscale image
    img = Image.open(image_path).convert('L')  # Convert to grayscale (L mode)

    # Resize image based on the printing size
    resize_factor = (max_size_cm * 10) / max(img.size[0], img.size[1])
    width, height = int(img.size[0] * resize_factor), int(img.size[1] * resize_factor)
    img = img.resize((width, height), Image.LANCZOS)  # Resize with LANCZOS filter

    # transform the img to a height map vertices
    cols, faces, rows, vertices = grey_scale_to_height_map(img, max_height, min_height)

    # Add a rectangular base at the bottom of the model
    close_bottom(cols, faces, rows, vertices)

    # Add walls on the sides by connecting top and bottom vertices along the edges
    close_sides(cols, faces, rows)

    # Convert lists to numpy arrays
    vertices = np.array(vertices)
    faces = np.array(faces)

    # Create the mesh object
    model_mesh = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))

    # Assign the vertices to the mesh
    for i, face in enumerate(faces):
        for j in range(3):
            model_mesh.vectors[i][j] = vertices[face[j], :]

    # Save the mesh to an STL file with incremental filenames
    i = 0
    base_name, extension = os.path.splitext(output_stl)
    while i < 50:
        img_name = image_path.split(".")[0]
        path = f"{int(max_height*100)}_{int(min_height*100)}_{img_name}{extension}"
        if not os.path.exists(path):
            model_mesh.save(path)
            print(f"3D model saved to {path}")
            break
        i += 1
    else:
        print("Could not save the 3D model, exceeded file limit.")


def grey_scale_to_height_map(img, max_height, min_height):
    # Normalize pixel values (0 to 255) to height (max_height to min_height)
    img = np.array(img)
    height_map = (1 - (img / 255.0)) * (max_height - min_height) + min_height
    # Get the image dimensions
    rows, cols = height_map.shape
    # Create arrays to store vertices and faces
    vertices = []
    faces = []
    # Generate vertices based on the height map
    for i in range(rows):
        for j in range(cols):
            # Each vertex has an (x, y, z) coordinate
            vertices.append([i, j, height_map[i, j]])  # Top vertex (minus to face up)
            vertices.append([i, j, 0])  # Bottom vertex (flat at z = 0)
    # Generate faces (triangular mesh) connecting the vertices
    for i in range(rows - 1):
        for j in range(cols - 1):
            # Calculate indices for the four corners of the current grid cell
            top_left = 2 * (i * cols + j)
            top_right = 2 * (i * cols + (j + 1))
            bottom_left = 2 * ((i + 1) * cols + j)
            bottom_right = 2 * ((i + 1) * cols + (j + 1))

            # Create two triangles per square
            faces.append([top_left, bottom_left, top_right])  # First triangle (top-left)
            faces.append([bottom_left, bottom_right, top_right])  # Second triangle (bottom-right)
    return cols, faces, rows, vertices


def close_bottom(cols, faces, rows, vertices):
    # The four corner vertices of the rectangle are the corners of the bottom layer
    bottom_rect_top_left = [0, 0, 0]
    bottom_rect_top_right = [0, cols - 1, 0]
    bottom_rect_bottom_left = [rows - 1, 0, 0]
    bottom_rect_bottom_right = [rows - 1, cols - 1, 0]
    # Append the rectangle vertices to the vertices list
    rect_start_index = len(vertices)
    vertices.extend([bottom_rect_top_left, bottom_rect_top_right, bottom_rect_bottom_left, bottom_rect_bottom_right])
    # Create two triangles to form the rectangular base
    faces.append([rect_start_index, rect_start_index + 2, rect_start_index + 1])  # First triangle of rectangle
    faces.append([rect_start_index + 1, rect_start_index + 2, rect_start_index + 3])  # Second triangle of rectangle


def close_sides(cols, faces, rows):
    # Top and bottom edges along the rows (left and right sides)
    for i in range(rows - 1):
        # Left side (connect first column vertices)
        top_left = 2 * (i * cols)  # Top vertex on left edge
        bottom_left = 2 * ((i + 1) * cols)  # Top vertex on the next row's left edge
        faces.append([top_left, top_left + 1, bottom_left])  # Triangle 1 (vertical wall)
        faces.append([top_left + 1, bottom_left + 1, bottom_left])  # Triangle 2 (vertical wall)

        # Right side (connect last column vertices)
        top_right = 2 * (i * cols + (cols - 1))  # Top vertex on right edge
        bottom_right = 2 * ((i + 1) * cols + (cols - 1))  # Top vertex on the next row's right edge
        faces.append([top_right, top_right + 1, bottom_right])  # Triangle 1 (vertical wall)
        faces.append([top_right + 1, bottom_right + 1, bottom_right])  # Triangle 2 (vertical wall)
    # Top and bottom edges along the columns (top and bottom sides)
    for j in range(cols - 1):
        # Top side (connect first row vertices)
        top_top = 2 * j  # Top vertex on the top edge
        bottom_top = 2 * (j + 1)  # Top vertex on the next column in the first row
        faces.append([top_top, top_top + 1, bottom_top])  # Triangle 1 (vertical wall)
        faces.append([top_top + 1, bottom_top + 1, bottom_top])  # Triangle 2 (vertical wall)

        # Bottom side (connect last row vertices)
        top_bottom = 2 * ((rows - 1) * cols + j)  # Top vertex on the bottom edge
        bottom_bottom = 2 * ((rows - 1) * cols + (j + 1))  # Top vertex on the next column in the last row
        faces.append([top_bottom, top_bottom + 1, bottom_bottom])  # Triangle 1 (vertical wall)
        faces.append([top_bottom + 1, bottom_bottom + 1, bottom_bottom])  # Triangle 2 (vertical wall)


if __name__ == "__main__":

    # Example usage
    image_to_3d_model("NOYandYO.png", 'output_model.stl')
    time.sleep(2)


    path = sorted([path for path in os.listdir() if path.split(".")[1] == "stl"])[-1]
    mesh = o3d.io.read_triangle_mesh(path)
    print(f"showing {path}")
    mesh = mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh], window_name="STL", left=1000, top=200, width=1000, height=1000)

