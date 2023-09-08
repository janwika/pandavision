import cv2
import numpy as np
import os

# Directory containing PNG files
input_directory = '/home/jan/fiftyone/open-images-v7/train/labels/masks_filtered'

# Output directory for text files
output_directory = '/home/jan/fiftyone/open-images-v7/yolo_format/train/labels'

# Create the output directory if it doesn't exist
if not os.path.exists(output_directory):
    os.makedirs(output_directory)

# Iterate over PNG files in the input directory
for filename in os.listdir(input_directory):
    if filename.endswith(".png"):
        # Load the binary mask image
        binary_mask_image = cv2.imread(os.path.join(input_directory, filename), cv2.IMREAD_GRAYSCALE)

        # Find contours in the binary mask image
        contours, _ = cv2.findContours(binary_mask_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)

        # Only keep the first contour if it exists
        if len(contours) > 0:
            contour = contours[0]

            # Get the dimensions of the image
            height, width = binary_mask_image.shape[:2]

            # Create a list to store normalized contour points
            normalized_contour_points = []

            # Extract contour points, normalize them, and add them to the list
            for point in contour:
                x, y = point[0]
                normalized_x = x / width
                normalized_y = y / height
                normalized_contour_points.extend([normalized_x, normalized_y])

            # Extract the part of the filename before the first underscore
            file_prefix = filename.split('_')[0]

            # Create the output text file path with the file_prefix
            output_file_path = os.path.join(output_directory, file_prefix + ".txt")

            # Check if the text file already exists
            file_exists = os.path.exists(output_file_path)

            # Append or create the text file and write the normalized contour points to it
            with open(output_file_path, 'a') as output_file:
                if not file_exists:
                    output_file.write('0 ')
                output_file.write(' '.join(map(str, normalized_contour_points)) + '\n')

            if file_exists:
                print(f"Appended to {output_file_path}")
            else:
                print(f"Normalized contour points saved to {output_file_path}")
