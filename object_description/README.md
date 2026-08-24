# Image Object Description JSON

This project uses an open-source vision-language model through vLLM to extract
the main object from one image and generate strict JSON:

```json
{
  "coarse_description": "bottle",
  "medium_description": "white bottle",
  "normal_description": "white plastic bottle",
  "size_description": "white plastic bottle with height {height} and width {width}"
}
```

Model weights are not included in this repository. The code leaves local model
path placeholders for deployment on the inference server.

Directory layout:

```text
VLA-Benchmark-Rush/
  assets/
  object_description/
    infer.py
    requirements.txt
    images/
    models/
```

## Install

Use Python 3.10+ and install dependencies:

```bash
cd object_description
pip install -r requirements.txt
```

Qwen3-VL support requires recent `vllm` and `transformers` releases. If your
installed Transformers version cannot load Qwen3-VL, install Transformers from
source:

```bash
pip install git+https://github.com/huggingface/transformers
```

For GPU environments, install the `torch` build that matches your CUDA version
from the official PyTorch instructions before installing the rest of the
requirements.

Optional Hugging Face settings, only needed if the deployment server loads from
Hugging Face instead of a local model directory:

```bash
export HF_TOKEN="<your_huggingface_token>"
export HF_HOME="/path/to/model/cache"
export HF_ENDPOINT="https://hf-mirror.com"
```

## Run

Put or mount the preferred model at:

```text
object_description/models/Qwen3-VL-30B-A3B-Instruct
```

Optionally put the lower-resource fallback model at:

```text
object_description/models/Qwen3-VL-8B-Instruct
```

Put input images under `object_description/images/`. The script mirrors the
image parent directory under the repository-level `assets/` directory.

Single-image inference from the repository root:

```bash
python object_description/infer.py --image object_description/images/cola/500/500ml_cola.png
```

Or run from inside the business directory:

```bash
cd object_description
python infer.py --image images/cola/500/500ml_cola.png
```

The command is equivalent to:

```bash
cd object_description
python infer.py \
  --image images/cola/500/500ml_cola.png \
  --backend vllm \
  --model models/Qwen3-VL-30B-A3B-Instruct \
  --fallback-model models/Qwen3-VL-8B-Instruct
```

The terminal output is only valid JSON:

```json
{
  "coarse_description": "bottle",
  "medium_description": "white bottle",
  "normal_description": "white plastic bottle",
  "size_description": "white plastic bottle with height {height} and width {width}"
}
```

By default, the same JSON is saved as `description.json` under the matching
directory in `assets`.

If that output JSON already exists, inference is skipped only when all four
description fields are present and pass validation. Older two-field JSON files
are regenerated.

If the input image is under:

```text
object_description/images/cola/500/500ml_cola.png
```

the output file is:

```text
assets/cola/500/description.json
```

The script also supports the existing repository image queue:

```text
assets_build/img_wait_process/cola/500/500ml_cola.png
```

This path also writes to:

```text
assets/cola/500/description.json
```

If the image is outside the configured input roots, the output directory is
inferred from the image filename:

```text
assets/<image_stem>/description.json
```

You can override the output path:

```bash
python object_description/infer.py \
  --image path/to/image.jpg \
  --output assets/my_object/description.json
```

## Useful Options

Use the 8B model directly:

```bash
python infer.py --image path/to/image.jpg --model models/Qwen3-VL-8B-Instruct --fallback-model ""
```

Use multiple GPUs with vLLM tensor parallelism:

```bash
python infer.py \
  --image path/to/image.jpg \
  --model models/Qwen3-VL-30B-A3B-Instruct \
  --tensor-parallel-size 4 \
  --gpu-memory-utilization 0.9
```

Limit vLLM context length if the server has constrained GPU memory:

```bash
python infer.py \
  --image path/to/image.jpg \
  --model models/Qwen3-VL-30B-A3B-Instruct \
  --max-model-len 8192
```

Use the original Transformers backend instead of vLLM:

```bash
python infer.py \
  --image path/to/image.jpg \
  --backend transformers \
  --model models/Qwen3-VL-8B-Instruct \
  --fallback-model ""
```

Use a different local model path (relative to your current working directory, or under `object_description/`):

```bash
python infer.py --image path/to/image.jpg --model models/Qwen3-VL-30B-A3B-Instruct
```

Use Hugging Face model ids on a server that can download/cache weights:

```bash
python infer.py \
  --image path/to/image.jpg \
  --model Qwen/Qwen3-VL-30B-A3B-Instruct \
  --fallback-model Qwen/Qwen3-VL-8B-Instruct
```

Enable FlashAttention 2 if it is installed:

```bash
python infer.py --image path/to/image.jpg --backend transformers --attn-implementation flash_attention_2
```

Print loading and save details to stderr:

```bash
python infer.py --image path/to/image.jpg --verbose
```

Print JSON without writing a file:

```bash
python infer.py --image path/to/image.jpg --no-save
```

## Output Rules

The script validates model output before printing or saving it:

- JSON must contain exactly `coarse_description`, `medium_description`,
  `normal_description`, and `size_description`.
- All fields must be strings.
- `coarse_description` is the broad object category only, for example `book`.
- `medium_description` adds the most important visible attribute, for example
  `black book`.
- `normal_description` is the fine-grained short description, for example
  `black leather notebook cover`.
- `coarse_description`, `medium_description`, and `normal_description` must not
  contain size placeholders.
- `size_description` must contain at least one of `{height}`, `{width}`,
  `{depth}`, `{diameter}`, `{length}`.
- Numeric real-world dimensions are not requested from the model; placeholders
  are used instead.

If validation fails, the script retries once by default. If it still cannot
produce valid JSON, it exits with an error on stderr and does not print invalid
JSON to stdout.
