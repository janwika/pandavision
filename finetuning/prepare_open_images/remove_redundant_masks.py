import os
import shutil

# Specify the directory path you want to iterate over
data_path = "/home/jan/fiftyone/open-images-v7/validation/data"
mask_path = "/home/jan/fiftyone/open-images-v7/validation/labels/masks"
dest_path = "/home/jan/fiftyone/open-images-v7/validation/labels/masks_filtered"

name_list = []

# Check if the directory exists
if not os.path.exists(data_path):
    print(f"The directory '{data_path}' does not exist.")
else:
    # Iterate over the files in the directory
    for filename in os.listdir(data_path):
        # Check if the item is a file
        if os.path.isfile(os.path.join(data_path, filename)):
            # Split the file name and extension
            name_without_extension, file_extension = os.path.splitext(filename)
            name_list.append(name_without_extension)

# Check if the source directory exists
if not os.path.exists(mask_path):
    print(f"The source directory '{mask_path}' does not exist.")
else:
    # Create the destination directory if it doesn't exist
    if not os.path.exists(dest_path):
        os.makedirs(dest_path)

    # Iterate over subdirectories '0' to 'F' (hexadecimal)
    for subdirectory_name in range(16):
        subdirectory_path = os.path.join(mask_path, format(subdirectory_name, 'X'))
        print(f"searching directory {format(subdirectory_name, 'X')}")

        # Check if the subdirectory exists
        if os.path.exists(subdirectory_path):
            # Iterate over the files in the subdirectory
            for filename in os.listdir(subdirectory_path):
                file_path = os.path.join(subdirectory_path, filename)

                # Check if the item is a file and ends with '.png'
                if os.path.isfile(file_path) and filename.endswith('.png'):
                    # Extract the corresponding filename without extension
                    name_without_extension, _ = os.path.splitext(filename)

                    # Check if the name exists in the previous list
                    if any(name_without_extension.startswith(item) for item in name_list):
                        # Copy the file to the destination directory
                        shutil.copy(file_path, os.path.join(dest_path, filename))