import cv2
import numpy as np

# Load the image
image_path = "example.png"
image = cv2.imread(image_path)

# Define the list of colors to be replaced (BGR format)
colors_to_replace = [
    #[0, 0, 255],  # Red
    #[0, 255, 0],  # Green
    #[255, 0, 0]   # Blue
    [30, 21, 201] 
]

# Define the replacement color (white)
replacement_color = [255, 255, 255]

# Iterate through each pixel and replace the specified colors with white
for color in colors_to_replace:
    mask = np.all(image == color, axis=-1)
    image[mask] = replacement_color

# Save the modified image
output_path = "modified_example.png"
cv2.imwrite(output_path, image)

print("Modified image has been saved as", output_path)
