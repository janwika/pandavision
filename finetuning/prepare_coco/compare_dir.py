import os

# Replace these paths with the paths to your directories
dir1_path = '/home/jan/fiftyone/coco-2017/yolo_format/validation/images'
dir2_path = '/home/jan/fiftyone/coco-2017/yolo_format/validation/labels'

# Get the list of file names without extensions in directory 1
dir1_files = [os.path.splitext(file)[0] for file in os.listdir(dir1_path) if file.endswith('.jpg')]

# Get the list of file names without extensions in directory 2
dir2_files = [os.path.splitext(file)[0] for file in os.listdir(dir2_path) if file.endswith('.txt')]

# Find files that are in one directory but not in the other
files_not_in_dir2 = [file for file in dir1_files if file not in dir2_files]
files_not_in_dir1 = [file for file in dir2_files if file not in dir1_files]

# Print file names that are not corresponding
print("Files in directory 1 but not in directory 2:")
for file in files_not_in_dir2:
    print(file)

print("\nFiles in directory 2 but not in directory 1:")
for file in files_not_in_dir1:
    print(file)
