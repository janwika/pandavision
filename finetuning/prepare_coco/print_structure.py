import json

def print_json_structure(data, indent=0):
    if isinstance(data, dict):
        for key, value in data.items():
            print("  " * indent + f"Key: '{key}'")
            print("  " * indent + "Type: 'dict'")
            print_json_structure(value, indent + 1)
    elif isinstance(data, list):
        if len(data) > 0:
            print("  " * indent + f"Type: 'list'")
            print_json_structure(data[0], indent + 1)
    else:
        print("  " * indent + f"Type: '{type(data).__name__}'")

# Replace 'your_huge.json' with the path to your JSON file
file_path = '/home/jan/fiftyone/coco-2017/train/labels.json'

try:
    with open(file_path, 'r', encoding='utf-8') as file:
        json_data = json.load(file)
        print_json_structure(json_data)
except FileNotFoundError:
    print(f"File '{file_path}' not found.")
except json.JSONDecodeError as e:
    print(f"JSON decoding error: {str(e)}")
except Exception as e:
    print(f"An error occurred: {str(e)}")
