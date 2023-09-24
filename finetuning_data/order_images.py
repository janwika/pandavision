import os
import re

directory_path = './finetuning_data/side_view/rgb'

# Get a list of files in the directory
files = os.listdir(directory_path)

# Extract numbers from the filenames and store them with the original filenames
file_numbers = {}  # {file_number: original_filename}
for file_name in files:
    if os.path.isfile(os.path.join(directory_path, file_name)):
        match = re.match(r'^.*?(\d+)\..*$', file_name)
        if match:
            file_number = int(match.group(1))
            file_numbers[file_number] = file_name

# Sort the file numbers
sorted_file_numbers = sorted(file_numbers.keys())

# Counter for renaming
counter = 82

# Iterate through the sorted file numbers and rename the files
for file_number in sorted_file_numbers:
    original_filename = file_numbers[file_number]
    extension = os.path.splitext(original_filename)[1]
    
    # Generate the new name with leading zeros
    new_name = f'{counter:05d}'
    
    # Construct the new file path with .png extension
    new_file_path = os.path.join(directory_path, f'{new_name}{extension}')
    
    # Rename the file
    os.rename(os.path.join(directory_path, original_filename), new_file_path)
    
    # Increment the counter
    counter += 1
