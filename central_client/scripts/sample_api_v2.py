import rospy
from deprojection_pipeline_msgs.srv import GetObjectLocations, GetObjectLocationsResponse
import cv_bridge
import cv2
import numpy as np
from PIL import Image, ImageDraw
import io
import base64
import json
import requests
import random


def convert_image_to_text(image: Image) -> str:
    # This is also how OpenAI encodes images: https://platform.openai.com/docs/guides/vision
    # from PIL import Image
    # import io
    # import base64

    with io.BytesIO() as output:
        image.save(output, format="PNG")
        data = output.getvalue()
    return base64.b64encode(data).decode("utf-8")


def test_demo_api(
    image_path: str = "assets/demo_broccoli_and_hotdog.png",
    url: str = "https://robot-api-2.glitch.me/handle_request",
):
    # wget https://cdn.glitch.global/1ab1555e-3a98-4cdc-b609-9ca116d921fd/demo_broccoli_and_hotdog.png
    # import requests

    image = {
        "base64_string": convert_image_to_text(Image.open(image_path)),
        "objects": [
            {"object_id": "1", "x_min": 210, "y_min": 220, "x_max": 777, "y_max": 798},
            {"object_id": "2", "x_min": 340, "y_min": 393, "x_max": 635, "y_max": 667},
            {"object_id": "3", "x_min": 653, "y_min": 202, "x_max": 1000, "y_max": 805},
            {"object_id": "4", "x_min": 757, "y_min": 282, "x_max": 932, "y_max": 737},
        ],
    }

    data = {
        "instruction": "Please swap the food positions.",
        "images": [image],
    }

    headers = {"Content-Type": "application/json"}
    content = dict(data=data, job_type="planning")
    response = requests.post(url, data=json.dumps(content), headers=headers)
    print(response.json())

    # {'raw_output': 'Plan:\n1. pick_and_place(2, 3)\n2. pick_and_place(4, 1)\n3. done()', 'success': True, 'plan_actions': [{'action_type': 'pick_and_place', 'source_object_id': '2', 'target_object_id': '3'}, {'action_type': 'pick_and_place', 'source_object_id': '4', 'target_object_id': '1'}]}


def test_grounding_server(
    url: str = "https://robot-api-2.glitch.me/handle_request",
    image_path: str = "assets/demo_kitchen_top.jpg",
):

    image = Image.open(image_path)
    headers = {"Content-Type": "application/json"}
    content = dict(data=dict(image=convert_image_to_text(image)), job_type="grounding")
    response = requests.post(url, json=content, headers=headers)
    objects = response.json()["objects"]

    random.seed(1)
    color_list = [
        [255, 0, 0, 128],
        [0, 255, 0, 128],
        [0, 0, 255, 128],
        [255, 255, 0, 128],
        [0, 255, 255, 128],
        [255, 0, 255, 128],
    ]

    def decode_mask(text: str, width: int, height: int):
        s = text.split()
        starts, lengths = [np.asarray(x, dtype=int) for x in (s[0:][::2], s[1:][::2])]
        starts -= 1
        ends = starts + lengths
        img = np.zeros(width * height, dtype=np.uint8)
        for lo, hi in zip(starts, ends):
            img[lo:hi] = 1
        return img.reshape(height, width).tolist()

    for o in objects:
        mask = np.zeros((image.height, image.width, 4), dtype=np.uint8)
        color = random.choice(color_list)  # Random with 50% opacity
        decoded = decode_mask(o["encoded_mask"], image.width, image.height)
        mask[np.array(decoded) > 0] = color
        overlay = Image.fromarray(mask, mode="RGBA")
        image = Image.alpha_composite(image.convert("RGBA"), overlay)

    draw = ImageDraw.Draw(image)
    for o in objects:
        draw.rectangle(o["box"], outline="red", width=2)
        position = (o["box"][0], o["box"][1] - min(image.size) // 32)
        draw.rectangle(draw.textbbox(position, o["label"]), fill="red")
        draw.text(position, o["label"], fill="white")
        print(dict(label=o["label"], box=o["box"]))
    image.save("demo.png")
# This is the demo image I used to test the grounding api
# Thanks ken, Will test this at the earliest