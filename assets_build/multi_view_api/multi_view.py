from openai import OpenAI
import base64
import mimetypes
from pathlib import Path
from typing import Optional
import os, pdb

DEFAULT_BASE_URL = "https://genaiapi.cloudsway.net/v1/ai/wMqMOXitfiwPvecC/"
DEFAULT_MODEL = "MaaS_Ge_3.1_flash_image_preview_20260226"
DEFAULT_API_KEY = "tIGuTG6A31JqFgHPCw0k"

def extract_first_base64_image(response_dict: dict) -> str:
    image_url = response_dict["choices"][0]["message"]["images"][0]["image_url"]["url"]
    return image_url.split(";base64,", 1)[1]

def image_path_to_data_url(image_path: str | Path) -> str:
    p = Path(image_path)
    image_bytes = p.read_bytes()
    b64 = base64.b64encode(image_bytes).decode("utf-8")
    mime, _ = mimetypes.guess_type(str(p))
    mime = mime or "image/jpeg"
    return f"data:{mime};base64,{b64}"

def image2image_function(system_prompt = None, prompt = None, imgs_path = None, write_path = None):
    client = OpenAI(api_key=DEFAULT_API_KEY, base_url=DEFAULT_BASE_URL)

    if isinstance(imgs_path, list):
        data_urls = [image_path_to_data_url(p) for p in imgs_path]
    else:
        data_urls = [image_path_to_data_url(imgs_path)]
    
    messages = []
    if system_prompt is not None:
        messages.append({"role": "system", "content": system_prompt})
    user_message_content = {
        "role": "user",
        "content": [],
    }
    user_message_content["content"].append({"type": "text", "text": prompt})
    for data_url in data_urls:  
        user_message_content["content"].append({"type": "image_url", "image_url": {"url": data_url}})

    messages.append(user_message_content)

    response_dict = client.chat.completions.create(messages=messages, model=DEFAULT_MODEL).model_dump()
    img_b64 = extract_first_base64_image(response_dict)
    out = Path(write_path)
    out.write_bytes(base64.b64decode(img_b64))

def per_img_aibuild_funtion(img_path, per_img_output_path, output_prefix=""):
    per_img_output_path.mkdir(parents=True, exist_ok=True)

    prefix = f"{output_prefix}_" if output_prefix else ""
    all_img_path = {}
    all_img_path["front"] = per_img_output_path / f"{prefix}front.png"
    all_img_path["back"] = per_img_output_path / f"{prefix}back.png"
    all_img_path["left"] = per_img_output_path / f"{prefix}left.png"
    all_img_path["right"] = per_img_output_path / f"{prefix}right.png"
    
    image2image_function(
        system_prompt = None,
        prompt = "生成对应的正视图,背景为白色",
        imgs_path = img_path,
        write_path = all_img_path["front"]
    )

    image2image_function(
        system_prompt = None,
        prompt = "生成对应的背视图",
        imgs_path = all_img_path["front"],
        write_path = all_img_path["back"]
    )

    image2image_function(
        system_prompt = None,
        prompt = "第一张图为正视图，第二张图为背视图，请生产对应的左视图",
        imgs_path = [all_img_path["front"], all_img_path["back"]],
        write_path = all_img_path["left"]
    )

    image2image_function(
        system_prompt = None,
        prompt = "第一张图为正视图，第二张图为背视图，请生产对应的右视图",
        imgs_path = [all_img_path["front"], all_img_path["back"]],
        write_path = all_img_path["right"]
    )
    

if __name__ == "__main__":
    
    per_img_aibuild_funtion(
        img_path="./ori_img/300ml_cola.png",
        output_root_path="./aibuild_out"
    )
