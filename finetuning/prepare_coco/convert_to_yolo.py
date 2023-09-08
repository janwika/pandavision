import json
import os

def get_image_size(image_id, image_directory):
    image_filename = f"{image_id:012d}.jpg"
    image_path = os.path.join(image_directory, image_filename)
    if os.path.exists(image_path):
        try:
            from PIL import Image
            with Image.open(image_path) as img:
                width, height = img.size
                return width, height
        except Exception as e:
            print(f"Error reading image '{image_path}': {str(e)}")
    else:
        print(f"Image '{image_filename}' not found in '{image_directory}'")

def normalize_coordinates(coordinates, width, height):
    normalized_coordinates = [coord / width if i % 2 == 0 else coord / height for i, coord in enumerate(coordinates)]
    return normalized_coordinates

def extract_and_save_normalized_coordinates(data, target_category_id, image_directory, txt_path):
    if isinstance(data, dict):
        category = data.get('category_id')
        if category in target_category_id:
            segmentation = data.get('segmentation')
            if isinstance(segmentation, list):  # Check if segmentation is a list
                image_id = data.get('image_id')
                category = category - 47  # Map 48 to 1, 49 to 2, and 50 to 3
                width, height = get_image_size(image_id, image_directory)
                normalized_coordinates = normalize_coordinates(segmentation[0], width, height)
                coordinates_str = " ".join(map(str, normalized_coordinates))
                
                # Create or append to the txt file
                image_filename = f"{image_id:012d}.txt"
                txt_file_path = os.path.join(txt_path, image_filename)
                with open(txt_file_path, 'a') as txt_file:
                    txt_file.write(f"{category} {coordinates_str}\n")

    elif isinstance(data, list):
        for item in data:
            extract_and_save_normalized_coordinates(item, target_category_id, image_directory, txt_path)

file_path = '/home/jan/fiftyone/coco-2017/train/labels.json'
target_category_id = [48, 49, 50]
image_directory = '/home/jan/fiftyone/coco-2017/train/data'
txt_path = '/home/jan/fiftyone/coco-2017/yolo_format/train/labels'

try:
    with open(file_path, 'r', encoding='utf-8') as file:
        json_data = json.load(file)
        if 'annotations' in json_data:
            annotations = json_data['annotations']
            for annotation in annotations:
                extract_and_save_normalized_coordinates(annotation, target_category_id, image_directory, txt_path)
        else:
            print("No 'annotations' key found in the JSON data.")
except FileNotFoundError:
    print(f"File '{file_path}' not found.")
except json.JSONDecodeError as e:
    print(f"JSON decoding error: {str(e)}")
