
import os
import base64
from pathlib import Path
from openai import OpenAI
client = OpenAI()


#ai_image.py
#Generate an image from a text prmpt using OpenAI image API
#The putput image is  saved to computer and file path is given
#Inspired by the Real Python DALL·E tutorial

def generate_ai_image(
    prompt: str,
    output_path: str = "ai_image.png",
    size: str = "512x512",
    model: str = "dall-e-2",
) -> str:
    """
    Generate an image from a text prompt and save it as a PNG.

    Parameters
    --
    prompt : Text description of the desired image (e.g. "a simple outline
        of a cat logo, bold black lines, no shading").
        Keep this 'plotter-friendly' so the path generator has clean edges.
    output_path : Where to save the PNG (relative or absolute path).
    size : Image resolution string accepted by the Images API, e.g.
        "256x256", "512x512", "1024x1024" (depends on model).
    model : Dall-E2

    Returns: The filesystem path of the saved image (for use with
        generate_drawing_from_image()).
    RuntimeError
        If image generation fails or no data is returned.
    """

   
    # Call OpenAI Images API
    response = client.images.generate(
        model=model,
        prompt=prompt,
        n=1,
        size=size,
        response_format="b64_json",  # get raw data so we can save directly
    )

    if not response.data or not response.data[0].b64_json:
        raise RuntimeError("No image data returned from Images API.")

    # Decode base64 -> bytes
    image_bytes = base64.b64decode(response.data[0].b64_json)

    # Ensure directory exists
    out_path = Path(output_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # Write PNG
    with open(out_path, "wb") as f:
        f.write(image_bytes)

    return str(out_path)


# -- Convenience: AI -> Path in one call --

def generate_path_from_ai_prompt(
    prompt: str,
    resolution_mode: str = "medium",
    threshold: int = 128,
    x_min: float = -40.0,
    x_max: float = 40.0,
    y_min: float = 70.0,
    y_max: float = 150.0,
):
    """
    Pipelinepipeline:
      1. Ask AI to draw something (PNG).
      2. Convert that image into a single-stroke path
         using the existing backend tools
    Depends on:
      - generate_ai_image()
      - generate_drawing_from_image()
        (your existing function from the backend)
    """

    from backend import generate_drawing_from_image 

    img_path = generate_ai_image(prompt, output_path="ai_generated.png")

    path = generate_drawing_from_image(
        img_path,
        mode=resolution_mode,
        threshold=threshold,
        x_min=x_min,
        x_max=x_max,
        y_min=y_min,
        y_max=y_max,
    )

    return path
