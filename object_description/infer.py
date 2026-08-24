#!/usr/bin/env python3
"""Generate strict object-description JSON from one or more images with Qwen3-VL."""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable


APP_DIR = Path(__file__).resolve().parent
REPO_ROOT = APP_DIR.parent

DEFAULT_MODEL_PATH = str(APP_DIR / "models" / "Qwen3-VL-30B-A3B-Instruct")
DEFAULT_FALLBACK_MODEL_PATH = str(APP_DIR / "models" / "Qwen3-VL-8B-Instruct")
DEFAULT_ASSETS_ROOT = str(REPO_ROOT / "assets")
DEFAULT_INPUT_ROOTS = (
    str(APP_DIR / "images"),
    str(REPO_ROOT / "assets_build" / "img_wait_process"),
)
DEFAULT_BACKEND = "vllm"

ALLOWED_PLACEHOLDERS = {"height", "width", "depth", "diameter", "length"}

PROMPT = """You are an object description extractor.

Analyze the main object in the image and return only valid JSON.

Schema:
{
  "coarse_description": string,
  "medium_description": string,
  "normal_description": string,
  "size_description": string
}

Rules:
- coarse_description: broad object category only, without color, material, brand, or fine visual details.
- medium_description: object category plus the most important visible attribute, usually color.
- normal_description: fine-grained short English description of the main object without dimensions.
- size_description: same fine-grained object description with dimension placeholders.
- Prefer a clear detail progression, for example: "book" -> "black book" -> "black leather notebook cover".
- Use placeholders such as {height}, {width}, {depth}, {diameter}, {length}.
- Do not estimate real-world numeric dimensions.
- If there are multiple objects, describe the most visually prominent main object.
- Do not output markdown.
- Do not output explanations.
- Do not output extra fields.

Example output:
{
  "coarse_description": "bottle",
  "medium_description": "white bottle",
  "normal_description": "white plastic bottle",
  "size_description": "white plastic bottle with height {height} and width {width}"
}
"""

RETRY_PROMPT = PROMPT + """
The answer must be exactly one JSON object with only these keys:
coarse_description, medium_description, normal_description, size_description.
"""


class JsonValidationError(ValueError):
    """Raised when generated text cannot be coerced into the required schema."""


@dataclass(frozen=True)
class LoadedBackend:
    model_id: str
    model: Any
    processor: Any
    generate: Callable[[Any, Any, Path, str, int, float, float], str]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate object-description JSON for one or more images with Qwen3-VL.",
    )
    parser.add_argument(
        "--image",
        required=True,
        nargs="+",
        help="Path(s) to local image(s). Pass multiple paths to process all in one model load.",
    )
    parser.add_argument(
        "--model",
        default=DEFAULT_MODEL_PATH,
        help=(
            "Local model path or Hugging Face model id. "
            f"Default: {DEFAULT_MODEL_PATH}"
        ),
    )
    parser.add_argument(
        "--fallback-model",
        default=DEFAULT_FALLBACK_MODEL_PATH,
        help=(
            "Optional fallback local model path or Hugging Face model id. "
            "Set to an empty string to disable fallback. "
            f"Default: {DEFAULT_FALLBACK_MODEL_PATH}"
        ),
    )
    parser.add_argument(
        "--assets-root",
        default=DEFAULT_ASSETS_ROOT,
        help="Root directory where the JSON file is saved when --output is not set.",
    )
    parser.add_argument(
        "--input-root",
        default=",".join(DEFAULT_INPUT_ROOTS),
        help=(
            "Comma-separated input roots used to mirror relative parent directories "
            "under --assets-root. Use an empty string to disable."
        ),
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Explicit output JSON path. Overrides --assets-root path inference.",
    )
    parser.add_argument(
        "--output-name",
        default="",
        help=(
            "Output filename used inside the inferred assets directory. "
            "Defaults to '<image_stem>_description.json' when empty."
        ),
    )
    parser.add_argument(
        "--no-save",
        action="store_true",
        help="Print JSON to stdout without writing an output file.",
    )
    parser.add_argument(
        "--backend",
        default=DEFAULT_BACKEND,
        choices=("vllm", "transformers"),
        help=f"Inference backend. Default: {DEFAULT_BACKEND}",
    )
    parser.add_argument(
        "--device-map",
        default="auto",
        help="Transformers-only device_map passed to from_pretrained.",
    )
    parser.add_argument(
        "--dtype",
        default="auto",
        choices=("auto", "bfloat16", "float16", "float32"),
        help="Model dtype passed to from_pretrained.",
    )
    parser.add_argument(
        "--attn-implementation",
        default=None,
        choices=("eager", "sdpa", "flash_attention_2"),
        help="Transformers-only attention implementation.",
    )
    parser.add_argument(
        "--tensor-parallel-size",
        type=int,
        default=1,
        help="vLLM tensor_parallel_size.",
    )
    parser.add_argument(
        "--gpu-memory-utilization",
        type=float,
        default=0.9,
        help="vLLM gpu_memory_utilization.",
    )
    parser.add_argument(
        "--max-model-len",
        type=int,
        default=None,
        help="Optional vLLM max_model_len.",
    )
    parser.add_argument(
        "--limit-mm-images",
        type=int,
        default=1,
        help="vLLM limit_mm_per_prompt image count.",
    )
    parser.add_argument(
        "--temperature",
        type=float,
        default=0.0,
        help="Sampling temperature. 0.0 gives deterministic output.",
    )
    parser.add_argument(
        "--top-p",
        type=float,
        default=1.0,
        help="Sampling top_p.",
    )
    parser.add_argument(
        "--max-new-tokens",
        type=int,
        default=128,
        help="Maximum number of generated tokens.",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=1,
        help="Number of extra generation attempts if JSON validation fails.",
    )
    parser.add_argument(
        "--trust-remote-code",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Allow custom model code from Hugging Face repositories.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Write loading and save details to stderr.",
    )
    return parser.parse_args()


def resolve_image_path(raw_path: str) -> Path:
    image_path = Path(raw_path).expanduser()
    if not image_path.exists():
        raise FileNotFoundError(f"Image not found: {image_path}")
    if not image_path.is_file():
        raise ValueError(f"Image path is not a file: {image_path}")
    return image_path.resolve()


def parse_input_roots(raw_value: str) -> tuple[Path, ...]:
    if not raw_value:
        return ()
    return tuple(Path(item.strip()).expanduser() for item in raw_value.split(",") if item.strip())


def infer_output_path(
    image_path: Path,
    assets_root: Path,
    output_name: str,
    input_roots: tuple[Path, ...],
) -> Path:
    assets_root = assets_root.expanduser().resolve()

    for input_root in input_roots:
        try:
            relative_parent = image_path.parent.resolve().relative_to(input_root.resolve())
            return assets_root / relative_parent / output_name
        except ValueError:
            continue

    parts = image_path.parts
    if "img_wait_process" in parts:
        index = parts.index("img_wait_process")
        relative_parent = Path(*parts[index + 1 : -1]) if index + 1 < len(parts) - 1 else Path()
        if str(relative_parent) != ".":
            return assets_root / relative_parent / output_name

    try:
        relative_parent = image_path.parent.resolve().relative_to(assets_root)
        return assets_root / relative_parent / output_name
    except ValueError:
        return assets_root / image_path.stem / output_name


def model_ids_from_args(model_arg: str, fallback_model_arg: str) -> tuple[str, ...]:
    model_ids = [model_arg]
    if fallback_model_arg and fallback_model_arg != model_arg:
        model_ids.append(fallback_model_arg)
    return tuple(model_ids)


def load_transformers_objects(
    model_id: str,
    dtype: str,
    device_map: str,
    attn_implementation: str | None,
    trust_remote_code: bool,
    verbose: bool,
) -> tuple[Any, Any]:
    try:
        import torch
        import transformers
        from transformers import AutoProcessor
    except ImportError as exc:
        raise RuntimeError(
            "Missing runtime dependencies. Install them with: pip install -r requirements.txt"
        ) from exc

    model_class_names = []
    if "30B-A3B" in model_id:
        model_class_names.append("Qwen3VLMoeForConditionalGeneration")
    elif "Qwen3-VL" in model_id:
        model_class_names.append("Qwen3VLForConditionalGeneration")
    model_class_names.append("AutoModelForImageTextToText")

    model_class = None
    for class_name in model_class_names:
        candidate = getattr(transformers, class_name, None)
        if candidate is not None:
            model_class = candidate
            break

    if model_class is None:
        raise RuntimeError(
            "Installed transformers does not expose Qwen3-VL model classes. "
            "Upgrade transformers or install it from the Hugging Face source repository."
        )

    dtype_value: Any = "auto"
    if dtype != "auto":
        dtype_value = {
            "bfloat16": torch.bfloat16,
            "float16": torch.float16,
            "float32": torch.float32,
        }[dtype]

    kwargs: dict[str, Any] = {
        "device_map": device_map,
        "trust_remote_code": trust_remote_code,
        "dtype": dtype_value,
    }
    if attn_implementation:
        kwargs["attn_implementation"] = attn_implementation

    if verbose:
        print(f"Loading processor: {model_id}", file=sys.stderr)
    processor = AutoProcessor.from_pretrained(model_id, trust_remote_code=trust_remote_code)

    if verbose:
        print(f"Loading model: {model_id}", file=sys.stderr)
    try:
        model = model_class.from_pretrained(model_id, **kwargs)
    except TypeError as exc:
        if "dtype" not in str(exc):
            raise
        kwargs["torch_dtype"] = kwargs.pop("dtype")
        model = model_class.from_pretrained(model_id, **kwargs)

    model.eval()
    return model, processor


def load_first_available_model(
    model_ids: tuple[str, ...],
    args: argparse.Namespace,
) -> LoadedBackend:
    errors: list[str] = []
    for model_id in model_ids:
        try:
            if args.backend == "vllm":
                model, processor = load_vllm_objects(
                    model_id=model_id,
                    dtype=args.dtype,
                    tensor_parallel_size=args.tensor_parallel_size,
                    gpu_memory_utilization=args.gpu_memory_utilization,
                    max_model_len=args.max_model_len,
                    limit_mm_images=args.limit_mm_images,
                    trust_remote_code=args.trust_remote_code,
                    verbose=args.verbose,
                )
                return LoadedBackend(model_id, model, processor, generate_text_vllm)

            model, processor = load_transformers_objects(
                model_id=model_id,
                dtype=args.dtype,
                device_map=args.device_map,
                attn_implementation=args.attn_implementation,
                trust_remote_code=args.trust_remote_code,
                verbose=args.verbose,
            )
            return LoadedBackend(model_id, model, processor, generate_text_transformers)
        except Exception as exc:
            errors.append(f"{model_id}: {exc}")
            if len(model_ids) == 1:
                break
            print(f"Failed to load {model_id}; trying next model.", file=sys.stderr)
    raise RuntimeError("Could not load any configured model:\n" + "\n".join(errors))


def get_input_device(model: Any) -> Any:
    device = getattr(model, "device", None)
    if device is not None and str(device) != "meta":
        return device

    for parameter in model.parameters():
        if str(parameter.device) != "meta":
            return parameter.device

    return None


def build_messages(image_path: Path, prompt: str) -> list[dict[str, Any]]:
    return [
        {
            "role": "user",
            "content": [
                {"type": "image", "image": str(image_path)},
                {"type": "text", "text": prompt},
            ],
        }
    ]


def load_vllm_objects(
    model_id: str,
    dtype: str,
    tensor_parallel_size: int,
    gpu_memory_utilization: float,
    max_model_len: int | None,
    limit_mm_images: int,
    trust_remote_code: bool,
    verbose: bool,
) -> tuple[Any, Any]:
    try:
        from transformers import AutoProcessor
        from vllm import LLM
    except ImportError as exc:
        raise RuntimeError(
            "Missing vLLM runtime dependencies. Install them with: pip install -r requirements.txt"
        ) from exc

    llm_kwargs: dict[str, Any] = {
        "model": model_id,
        "trust_remote_code": trust_remote_code,
        "tensor_parallel_size": tensor_parallel_size,
        "gpu_memory_utilization": gpu_memory_utilization,
        "enforce_eager": True,
#                "limit_mm_per_prompt": {"image": limit_mm_images},
    }
    if dtype != "auto":
        llm_kwargs["dtype"] = dtype
    if max_model_len is not None:
        llm_kwargs["max_model_len"] = max_model_len

    if verbose:
        print(f"Loading processor: {model_id}", file=sys.stderr)
    processor = AutoProcessor.from_pretrained(model_id, trust_remote_code=trust_remote_code)

    if verbose:
        print(f"Loading vLLM model: {model_id}", file=sys.stderr)
    model = LLM(**llm_kwargs)
    return model, processor


def generate_text_transformers(
    model: Any,
    processor: Any,
    image_path: Path,
    prompt: str,
    max_new_tokens: int,
    _temperature: float,
    _top_p: float,
) -> str:
    import torch

    messages = build_messages(image_path, prompt)
    inputs = processor.apply_chat_template(
        messages,
        tokenize=True,
        add_generation_prompt=True,
        return_dict=True,
        return_tensors="pt",
    )

    input_device = get_input_device(model)
    if input_device is not None:
        inputs = inputs.to(input_device)

    with torch.inference_mode():
        generated_ids = model.generate(
            **inputs,
            max_new_tokens=max_new_tokens,
            do_sample=False,
        )

    generated_ids_trimmed = [
        output_ids[len(input_ids) :]
        for input_ids, output_ids in zip(inputs.input_ids, generated_ids)
    ]
    output_text = processor.batch_decode(
        generated_ids_trimmed,
        skip_special_tokens=True,
        clean_up_tokenization_spaces=False,
    )
    return output_text[0].strip()


def generate_text_vllm(
    model: Any,
    processor: Any,
    image_path: Path,
    prompt: str,
    max_new_tokens: int,
    temperature: float,
    top_p: float,
) -> str:
    from PIL import Image
    from vllm import SamplingParams

    messages = build_messages(image_path, prompt)
    text_prompt = processor.apply_chat_template(
        messages,
        tokenize=False,
        add_generation_prompt=True,
    )
    image = Image.open(image_path).convert("RGB")
    sampling_params = SamplingParams(
        temperature=temperature,
        top_p=top_p,
        max_tokens=max_new_tokens,
    )
    outputs = model.generate(
        [
            {
                "prompt": text_prompt,
                "multi_modal_data": {"image": image},
            }
        ],
        sampling_params=sampling_params,
    )
    return outputs[0].outputs[0].text.strip()


def extract_json_object(text: str) -> dict[str, Any]:
    stripped = text.strip()
    if stripped.startswith("```"):
        stripped = re.sub(r"^```(?:json)?\s*", "", stripped, flags=re.IGNORECASE)
        stripped = re.sub(r"\s*```$", "", stripped)

    candidates = [stripped]
    first = stripped.find("{")
    last = stripped.rfind("}")
    if first != -1 and last != -1 and last > first:
        candidates.append(stripped[first : last + 1])

    for candidate in candidates:
        try:
            parsed = json.loads(candidate)
        except json.JSONDecodeError:
            continue
        if isinstance(parsed, dict):
            return parsed

    field_pattern = re.compile(
        r'"(?P<key>coarse_description|medium_description|normal_description|size_description)"\s*:\s*"(?P<value>(?:\\.|[^"\\])*)"',
        re.DOTALL,
    )
    fields = {
        match.group("key"): json.loads(f'"{match.group("value")}"')
        for match in field_pattern.finditer(stripped)
    }
    if fields:
        return fields

    raise JsonValidationError("Model output is not valid JSON.")


def validate_schema(parsed: dict[str, Any]) -> dict[str, str]:
    expected_keys = {
        "coarse_description",
        "medium_description",
        "normal_description",
        "size_description",
    }
    if set(parsed.keys()) != expected_keys:
        missing = expected_keys - set(parsed.keys())
        extra = set(parsed.keys()) - expected_keys
        raise JsonValidationError(f"JSON keys mismatch. Missing={missing}, extra={extra}")

    coarse = parsed["coarse_description"]
    medium = parsed["medium_description"]
    normal = parsed["normal_description"]
    size = parsed["size_description"]
    descriptions = {
        "coarse_description": coarse,
        "medium_description": medium,
        "normal_description": normal,
        "size_description": size,
    }
    if not all(isinstance(value, str) for value in descriptions.values()):
        raise JsonValidationError("All JSON values must be strings.")

    coarse = " ".join(coarse.strip().split())
    medium = " ".join(medium.strip().split())
    normal = " ".join(normal.strip().split())
    size = " ".join(size.strip().split())
    if not all((coarse, medium, normal, size)):
        raise JsonValidationError("Descriptions must be non-empty strings.")

    for key, value in (
        ("coarse_description", coarse),
        ("medium_description", medium),
        ("normal_description", normal),
    ):
        placeholders = set(re.findall(r"\{([^{}]+)\}", value))
        if placeholders:
            raise JsonValidationError(f"{key} must not include placeholders.")

    size_placeholders = set(re.findall(r"\{([^{}]+)\}", size))
    if not size_placeholders:
        raise JsonValidationError("size_description must include at least one placeholder.")
    unsupported = size_placeholders - ALLOWED_PLACEHOLDERS
    if unsupported:
        raise JsonValidationError(f"Unsupported placeholders: {sorted(unsupported)}")

    return {
        "coarse_description": coarse,
        "medium_description": medium,
        "normal_description": normal,
        "size_description": size,
    }


def parse_and_validate_model_output(text: str) -> dict[str, str]:
    parsed = extract_json_object(text)
    return validate_schema(parsed)


def load_complete_existing_description(path: Path) -> dict[str, str] | None:
    if not path.is_file():
        return None

    try:
        parsed = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None

    if not isinstance(parsed, dict):
        return None

    try:
        return validate_schema(parsed)
    except JsonValidationError:
        return None


def infer_description(
    model: Any,
    processor: Any,
    generate: Callable[[Any, Any, Path, str, int, float, float], str],
    image_path: Path,
    max_new_tokens: int,
    temperature: float,
    top_p: float,
    retries: int,
) -> dict[str, str]:
    attempts = max(1, retries + 1)
    last_error: Exception | None = None

    for attempt in range(attempts):
        prompt = PROMPT if attempt == 0 else RETRY_PROMPT
        generated = generate(
            model,
            processor,
            image_path,
            prompt,
            max_new_tokens,
            temperature,
            top_p,
        )
        try:
            return parse_and_validate_model_output(generated)
        except JsonValidationError as exc:
            last_error = exc

    raise JsonValidationError(f"Could not produce valid schema JSON: {last_error}")


def write_json(path: Path, result: dict[str, str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    try:
        image_paths = [resolve_image_path(img) for img in args.image]
        input_roots = parse_input_roots(args.input_root)

        if args.output and len(image_paths) > 1:
            print(
                "WARNING: --output is ignored when multiple --image paths are given; "
                "output paths are inferred automatically.",
                file=sys.stderr,
            )

        jobs: list[tuple[Path, Path]] = []
        existing_results: dict[Path, dict[str, str]] = {}
        needs_inference = False

        for image_path in image_paths:
            output_name = args.output_name or f"{image_path.stem}_description.json"
            output_path = (
                Path(args.output).expanduser().resolve()
                if args.output and len(image_paths) == 1
                else infer_output_path(
                    image_path=image_path,
                    assets_root=Path(args.assets_root),
                    output_name=output_name,
                    input_roots=input_roots,
                )
            )
            jobs.append((image_path, output_path))

            existing = None if args.no_save else load_complete_existing_description(output_path)
            if existing is None:
                needs_inference = True
            else:
                existing_results[output_path] = existing

        loaded: LoadedBackend | None = None
        if needs_inference:
            loaded = load_first_available_model(
                model_ids_from_args(args.model, args.fallback_model),
                args,
            )
            if args.verbose:
                print(f"Using {args.backend} model: {loaded.model_id}", file=sys.stderr)
        elif args.verbose:
            print("All output JSON files are complete; skipping model load.", file=sys.stderr)

        exit_code = 0
        for image_path, output_path in jobs:
            try:
                existing = existing_results.get(output_path)
                if existing is not None:
                    if args.verbose:
                        print(f"Skipped complete JSON: {output_path}", file=sys.stderr)
                    print(json.dumps(existing, ensure_ascii=False, indent=2))
                    continue

                if loaded is None:
                    raise RuntimeError("Model was not loaded for an image that needs inference.")
                result = infer_description(
                    model=loaded.model,
                    processor=loaded.processor,
                    generate=loaded.generate,
                    image_path=image_path,
                    max_new_tokens=args.max_new_tokens,
                    temperature=args.temperature,
                    top_p=args.top_p,
                    retries=args.retries,
                )

                if not args.no_save:
                    write_json(output_path, result)
                    if args.verbose:
                        print(f"Saved JSON: {output_path}", file=sys.stderr)

                print(json.dumps(result, ensure_ascii=False, indent=2))
            except Exception as exc:
                print(f"ERROR [{image_path}]: {exc}", file=sys.stderr)
                exit_code = 1

        return exit_code
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
