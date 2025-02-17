import numpy as np

def compute_fov_x(camera_matrix, image_width):
    """
    Compute the horizontal Field of View (FoV_x).
    
    :param camera_matrix: 3x3 intrinsic camera matrix
    :param image_width: Width of the image in pixels
    :return: Horizontal FoV in degrees
    """
    fx = camera_matrix[0, 0]  # Focal length in x-direction
    fov_x = 2 * np.degrees(np.arctan(image_width / (2 * fx)))
    return fov_x

# Example usage
image_width = frameSize[0]  # Extract width from (width, height)
fov_x = compute_fov_x(cameraMatrix, image_width)

print(f"Horizontal FoV: {fov_x:.2f} degrees")
