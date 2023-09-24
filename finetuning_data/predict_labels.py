import os
import json
from ultralytics import YOLO

# Load a model
model = YOLO('/home/jan/Desktop/pandavision/finetuned_post_own_labels.pt')  # load an official model

# Directory containing images
image_directory = '/home/jan/Desktop/pandavision/finetuning_data/both_views'

# Get a list of image files in the directory
image_files = [f for f in os.listdir(image_directory) if f.endswith(('jpg', 'jpeg', 'png'))]

# Directory to save the segmentation masks
output_directory = '/home/jan/Desktop/pandavision/finetuning_data'

# Create the output directory if it doesn't exist
if not os.path.exists(output_directory):
    os.makedirs(output_directory)

# List to store the JSON objects
json_objects = []

# Function to normalize pixel values to the range [0, 100]
def normalize_to_100(value, max_value):
    return (value / max_value) * 100

# Predict with the model for each image and create the JSON objects
for image_file in image_files:
    image_path = os.path.join(image_directory, image_file)

    # Predict with the model
    results = model.predict(image_path, save=False)

    # Extract the polygon coordinates if available
    if results[0].masks is not None:
        width = 640
        height = 480
        masks = results[0].masks.xy
        classes = results[0].boxes.cls
        polygons = []

        for i, mask in enumerate(masks):
            # Normalize pixel values to the range [0, 100]
            normalized_polygon = [
                [normalize_to_100(x, width), normalize_to_100(y, height)] for x, y in mask.astype(int)
            ]

            polygons.append({
                "original_width": 100,  # Adjusted width
                "original_height": 100,  # Adjusted height
                "image_rotation": 0,
                "value": {
                    "points": normalized_polygon,
                    "closed": True,
                    "polygonlabels": [f"{int(classes[i])}"]
                },
                "id": f"polygon_{i}",
                "from_name": "tag",
                "to_name": "image",
                "type": "polygonlabels"
            })
    else:
        # No labels found by YOLO, so set polygons to an empty array
        polygons = []

    # Construct the JSON object for this image with polygons
    json_obj = {
        "data": {
            "image": f"http://192.168.0.150:5000/files/{image_file}"
        },
        "predictions": [
            {
                "model_version": "one",
                "score": 0.5,
                "result": polygons
            }
        ]
    }

    # Append the JSON object to the list
    json_objects.append(json_obj)

# Save the JSON objects to a JSON file
output_json_file = f"{output_directory}/output_finetuned.json"
with open(output_json_file, 'w') as json_file:
    json.dump(json_objects, json_file, indent=2)
