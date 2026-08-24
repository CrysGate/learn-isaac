# Qwen Image Edit Four-View Generation

This project generates object four-view images from one front-view reference
image with Qwen-Image-Edit.

Given one front-view image, it writes:

```text
outputs/object_001/
  front_front.png
  front_back.png
  front_left.png
  front_right.png
```

`front_front.png` is the original input normalized and saved as PNG.
`front_back.png`, `front_left.png`, and `front_right.png` are generated.

## Install

Use Python 3.10+:

```bash
cd qwen_image_edit_views
pip install -r requirements.txt
```

The preferred model, `Qwen/Qwen-Image-Edit-2511`, needs the latest Diffusers
because it uses `QwenImageEditPlusPipeline`. The fallback model,
`Qwen/Qwen-Image-Edit`, uses `QwenImageEditPipeline`.

For CUDA, install the PyTorch build matching your driver first. Example:

```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu126
pip install -r requirements.txt
```

Optional Hugging Face settings for the server where you download the models:

```bash
export HF_TOKEN="<your_huggingface_token>"
export HF_HOME="/path/to/huggingface/cache"
export HF_ENDPOINT="https://hf-mirror.com"
```

## Model Loading

Model weights are not included in this repository. Download or mount the models
on the inference server under these local directories:

```text
qwen_image_edit_views/models/Qwen-Image-Edit-2511
```

Optional fallback:

```text
qwen_image_edit_views/models/Qwen-Image-Edit
```

By default the script tries the local `Qwen-Image-Edit-2511` directory first
and falls back to the local `Qwen-Image-Edit` directory.

Equivalent command:

```bash
python generate_views.py \
  --input path/to/front.png \
  --output_dir outputs/object_001 \
  --model models/Qwen-Image-Edit-2511 \
  --fallback-model models/Qwen-Image-Edit
```

Use another local model directory (relative to your current working directory, or under `qwen_image_edit_views/`):

```bash
python generate_views.py \
  --input path/to/front.png \
  --output_dir outputs/object_001 \
  --model models/Qwen-Image-Edit-2511 \
  --fallback-model models/Qwen-Image-Edit
```

Use Hugging Face model ids only if the server is allowed to download/cache
weights:

```bash
python generate_views.py \
  --input path/to/front.png \
  --output_dir outputs/object_001 \
  --model Qwen/Qwen-Image-Edit-2511 \
  --fallback-model Qwen/Qwen-Image-Edit
```

Use `--local-files-only` when the server must not access Hugging Face during
runtime.

## Run

From the business directory:

```bash
cd qwen_image_edit_views
python generate_views.py \
  --input path/to/front.png \
  --output_dir outputs/object_001
```

From the repository root:

```bash
python qwen_image_edit_views/generate_views.py \
  --input path/to/front.png \
  --output_dir qwen_image_edit_views/outputs/object_001
```

Batch process all images under the repository `sample/` directory:

```bash
bash qwen_image_edit_views/run_sample_infer.sh
```

This loads the model once, skips images whose four directional outputs already
exist, and writes:

```text
assets/<object>/<variant>/<image_stem>_views/
  <image_stem>_front.png
  <image_stem>_back.png
  <image_stem>_left.png
  <image_stem>_right.png
```

## Multi-Image And Single-Image Modes

`Qwen/Qwen-Image-Edit-2511` uses the `plus` pipeline and can pass both
`front.png` and generated `back.png` directly when generating `left.png` and
`right.png`.

If the loaded model uses the older single-image pipeline, the script internally
creates a horizontal front/back reference image before generating side views.
Add `--save-reference` to write this helper image as
`reference_front_back.png`.

## Quality Checks

After generation, the script verifies:

- `<image_stem>_front.png`, `<image_stem>_back.png`, `<image_stem>_left.png`, and
  `<image_stem>_right.png` exist and can be opened.
- The four view images have identical dimensions.

Generated images are placed on a white canvas matching the input front-view
size if the model returns a different image size.

## Memory And Speed

Qwen-Image-Edit models are large. Use a CUDA GPU for practical inference. If
CUDA is unavailable, the script prints a warning because CPU inference can be
extremely slow.

Useful options:

```bash
# Use bfloat16 on CUDA by default
python generate_views.py --input front.png --output_dir outputs/object_001 --dtype auto

# Use fp16
python generate_views.py --input front.png --output_dir outputs/object_001 --dtype float16

# Lower VRAM, slower
python generate_views.py --input front.png --output_dir outputs/object_001 --cpu-offload

# Even lower VRAM, much slower
python generate_views.py --input front.png --output_dir outputs/object_001 --sequential-cpu-offload

# Additional memory optimizations if supported
python generate_views.py --input front.png --output_dir outputs/object_001 --vae-tiling --attention-slicing
```

For reproducibility:

```bash
python generate_views.py --input front.png --output_dir outputs/object_001 --seed 123
```
