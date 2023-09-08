import os

# Specify the directory path
directory_path = "/home/jan/fiftyone/open-images-v7/train/labels/masks_filtered"

# List all files in the directory
file_list = os.listdir(directory_path)

# Define the pattern that files should follow
pattern = "_m0k1tl_"

# Iterate through the files and delete those that don't match the pattern
for filename in file_list:
    if pattern not in filename:
        file_path = os.path.join(directory_path, filename)
        try:
            os.remove(file_path)
            print(f"Deleted: {filename}")
        except Exception as e:
            print(f"Error deleting {filename}: {str(e)}")