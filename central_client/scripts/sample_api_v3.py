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


def test_unified_server(
    url: str = "https://robot-api-2.glitch.me/handle_request",
    instruction: str = "Put the objects in the bowl in order of most sweet to least sweet.",
    image_path: str = "sample_image.jpeg",
    job_type: str = "grounding_and_planning",
):
    image = Image.open(image_path)
    data = {
        "instruction": instruction,
        "images": [dict(base64_string=convert_image_to_text(image), objects=[])],
    }
    headers = {"Content-Type": "application/json"}
    content = dict(data=data, job_type=job_type)
    response = requests.post(url, data=json.dumps(content), headers=headers)
    objects = response.json()["objects"]
    print(objects)
    print(type(objects), objects[0].keys(), len(objects))

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
        # print(dict(label=o["label"], box=o["box"]))

    image.save("demo.png")
    # print(dict(raw_output=response.json()["raw_output"]))
    # print(dict(plan_actions=response.json()["plan_actions"]))

test_unified_server()