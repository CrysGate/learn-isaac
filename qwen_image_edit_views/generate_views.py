#!/usr/bin/env python3
"""Generate orthographic four-view object images with Qwen-Image-Edit."""

from __future__ import annotations

import argparse
import inspect
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    from PIL import Image, ImageOps
except ImportError:
    Image = None
    ImageOps = None


APP_DIR = Path(__file__).resolve().parent

DEFAULT_MODEL = str(APP_DIR / "models" / "Qwen-Image-Edit-2511")
DEFAULT_FALLBACK_MODEL = str(APP_DIR / "models" / "Qwen-Image-Edit")

# Tencent Hunyuan 3D multiview upload limits (images2obj / 混元 3D 网页端)
HUNYUAN_SUPPORTED_SUFFIXES = {".png", ".jpg", ".jpeg", ".webp"}
HUNYUAN_MIN_SIDE = 128
HUNYUAN_MAX_BYTES = 10 * 1024 * 1024
HUNYUAN_DEFAULT_MIN_SUBJECT_RATIO = 0.20
HUNYUAN_DEFAULT_TARGET_SUBJECT_RATIO = 0.55
HUNYUAN_WHITE_THRESHOLD = 245

HUNYUAN_3D_VIEW_REQUIREMENTS = """
Hunyuan 3D multiview upload requirements (mandatory — violations break 3D reconstruction):
- Format intent: clean product photo suitable for PNG export (no collage, no borders).
- Single subject only: exactly ONE main object, centered; no secondary objects, hands, props, or duplicates.
- Subject scale: the object must be LARGE in the frame (roughly 50–70% of image height); NOT tiny, distant, or lost in empty space.
- No text: absolutely NO letters, numbers, brand names, packaging text, barcodes, logos, watermarks, captions, or UI.
- Background: pure flat white (#FFFFFF) studio backdrop; no scenery, gradients, patterns, or cast shadows on the floor.
"""

BACK_PROMPT = (
    """Generate the back view of the exact same object shown in the reference front-view image.

Requirements:
- The output must show only the back side of the same object.
- Preserve the object's category, material, color, proportions, silhouette, and distinctive visual details.
- Infer the hidden back-side structure consistently from the visible front view.
- Use an orthographic product-view style.
- Keep the object upright, centered, and at the same scale as the reference image.
- Use a plain white background.
- No perspective distortion.
- No extra objects.
- No text, labels, arrows, measurements, watermark, or decorations.
- Do not change the object identity.
"""
    + HUNYUAN_3D_VIEW_REQUIREMENTS
)

LEFT_PROMPT = (
    """Generate the left side view of the exact same object using the provided front-view and back-view reference images.

Requirements:
- The output must show only the left side of the same object.
- Use the front view and back view to maintain consistent shape, depth, material, color, proportions, and visual details.
- The left side must be geometrically consistent with both the front and back views.
- Use an orthographic product-view style.
- Keep the object upright, centered, and at the same scale as the reference images.
- Use a plain white background.
- No perspective distortion.
- No extra objects.
- No text, labels, arrows, measurements, watermark, or decorations.
- Do not change the object identity.
"""
    + HUNYUAN_3D_VIEW_REQUIREMENTS
)

RIGHT_PROMPT = (
    """Generate the right side view of the exact same object using the provided front-view and back-view reference images.

Requirements:
- The output must show only the right side of the same object.
- Use the front view and back view to maintain consistent shape, depth, material, color, proportions, and visual details.
- The right side must be geometrically consistent with both the front and back views.
- Use an orthographic product-view style.
- Keep the object upright, centered, and at the same scale as the reference images.
- Use a plain white background.
- No perspective distortion.
- No extra objects.
- No text, labels, arrows, measurements, watermark, or decorations.
- Do not change the object identity.
"""
    + HUNYUAN_3D_VIEW_REQUIREMENTS
)

NEGATIVE_PROMPT = (
    "different object, changed shape, changed material, inconsistent proportions, "
    "perspective view, angled view, tilted object, cropped object, extra object, "
    "second object, multiple objects, pair of objects, scene with clutter, "
    "tiny object, small object, distant object, object in corner, lots of empty space, "
    "text, typography, letters, numbers, words, logo, brand mark, barcode, label, "
    "caption, subtitle, watermark, arrow, measurement, price tag, packaging text, "
    "shadow, decorative background, gradient background, textured background, "
    "low quality, distorted geometry"
)

VIEW_FILES = {
    "front": "front.png",
    "back": "back.png",
    "left": "left.png",
    "right": "right.png",
}


def require_pillow() -> None:
    if Image is None or ImageOps is None:
        raise RuntimeError("Missing Pillow. Install dependencies with: pip install -r requirements.txt")


@dataclass(frozen=True)
class LoadedPipeline:
    model_ref: str
    pipeline_kind: str
    pipeline: Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate front/back/left/right object views with Qwen-Image-Edit.",
    )
    parser.add_argument(
        "--input",
        required=True,
        nargs="+",
        help="Input front-view image path(s). Pass multiple paths to process all in one model load.",
    )
    parser.add_argument(
        "--output_dir",
        required=True,
        help=(
            "Directory for generated views. With one input this is the exact output directory; "
            "with multiple inputs it is treated as an output root."
        ),
    )
    parser.add_argument(
        "--input-root",
        default="",
        help=(
            "Comma-separated input roots used to mirror relative directories under --output_dir "
            "when multiple --input paths are given."
        ),
    )
    parser.add_argument(
        "--output-suffix",
        default="_views",
        help="Suffix for each per-image output directory when output paths are inferred.",
    )
    parser.add_argument(
        "--model",
        default=DEFAULT_MODEL,
        help=f"Primary model id or local model path. Default: {DEFAULT_MODEL}",
    )
    parser.add_argument(
        "--fallback-model",
        default=DEFAULT_FALLBACK_MODEL,
        help=(
            "Fallback model id or local path. Set to an empty string to disable. "
            f"Default: {DEFAULT_FALLBACK_MODEL}"
        ),
    )
    parser.add_argument(
        "--pipeline",
        default="auto",
        choices=("auto", "plus", "base"),
        help="Pipeline type. Use plus for QwenImageEditPlusPipeline, base for QwenImageEditPipeline.",
    )
    parser.add_argument(
        "--device",
        default="auto",
        help="Device for inference. Use auto, cuda, cuda:0, or cpu.",
    )
    parser.add_argument(
        "--dtype",
        default="auto",
        choices=("auto", "bfloat16", "float16", "float32"),
        help="Torch dtype. auto uses bfloat16 on CUDA and float32 on CPU.",
    )
    parser.add_argument("--seed", type=int, default=0, help="Base random seed.")
    parser.add_argument("--num-inference-steps", type=int, default=40)
    parser.add_argument("--true-cfg-scale", type=float, default=4.0)
    parser.add_argument("--guidance-scale", type=float, default=1.0)
    parser.add_argument("--negative-prompt", default=NEGATIVE_PROMPT)
    parser.add_argument(
        "--cpu-offload",
        action="store_true",
        help="Enable diffusers model CPU offload. Requires accelerate and a CUDA device.",
    )
    parser.add_argument(
        "--sequential-cpu-offload",
        action="store_true",
        help="Enable sequential CPU offload for lower VRAM at slower speed.",
    )
    parser.add_argument(
        "--vae-tiling",
        action="store_true",
        help="Enable VAE tiling when the pipeline supports it.",
    )
    parser.add_argument(
        "--attention-slicing",
        action="store_true",
        help="Enable attention slicing when the pipeline supports it.",
    )
    parser.add_argument(
        "--local-files-only",
        action="store_true",
        help="Load only from local Hugging Face cache or local model paths.",
    )
    parser.add_argument(
        "--save-reference",
        action="store_true",
        help="Save reference_front_back.png when single-image fallback input is used.",
    )
    parser.add_argument(
        "--progress",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Show diffusers progress bars.",
    )
    parser.add_argument(
        "--overwrite",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Overwrite existing output files.",
    )
    parser.add_argument(
        "--no-hunyuan-constraints",
        action="store_true",
        help="Disable Hunyuan 3D post-processing (resolution, file size, subject scale).",
    )
    parser.add_argument(
        "--max-output-mb",
        type=float,
        default=10.0,
        help="Max output file size per view in MB (Hunyuan 3D limit). Default: 10.",
    )
    parser.add_argument(
        "--min-subject-ratio",
        type=float,
        default=HUNYUAN_DEFAULT_MIN_SUBJECT_RATIO,
        help=(
            "Min fraction of image area occupied by the object bbox; "
            "smaller subjects are auto-scaled up. Default: 0.20."
        ),
    )
    parser.add_argument(
        "--target-subject-ratio",
        type=float,
        default=HUNYUAN_DEFAULT_TARGET_SUBJECT_RATIO,
        help="Target bbox area fraction after auto-scaling. Default: 0.55.",
    )
    return parser.parse_args()


def log(message: str) -> None:
    print(message, file=sys.stderr)


def resolve_input_image(path: str) -> Path:
    image_path = Path(path).expanduser().resolve()
    if not image_path.exists():
        raise FileNotFoundError(f"Input image not found: {image_path}")
    if not image_path.is_file():
        raise ValueError(f"Input path is not a file: {image_path}")
    validate_input_for_hunyuan(image_path)
    return image_path


def validate_input_for_hunyuan(path: Path, *, max_bytes: int = HUNYUAN_MAX_BYTES) -> None:
    """Check input format/size before generation (Hunyuan 3D compatible formats)."""
    suffix = path.suffix.lower()
    if suffix not in HUNYUAN_SUPPORTED_SUFFIXES:
        supported = ", ".join(sorted(HUNYUAN_SUPPORTED_SUFFIXES))
        raise ValueError(
            f"Unsupported input format '{suffix}' for Hunyuan 3D: {path}\n"
            f"Supported: {supported}"
        )
    size = path.stat().st_size
    if size > max_bytes:
        raise ValueError(
            f"Input file exceeds {max_bytes // (1024 * 1024)}MB limit: {path} "
            f"({size / (1024 * 1024):.2f} MB)"
        )
    require_pillow()
    with Image.open(path) as image:
        image = ImageOps.exif_transpose(image)
        width, height = image.size
    if width < HUNYUAN_MIN_SIDE or height < HUNYUAN_MIN_SIDE:
        log(
            f"WARNING: Input resolution {width}x{height} is below {HUNYUAN_MIN_SIDE}px; "
            "will upscale outputs to meet Hunyuan minimum."
        )


def parse_input_roots(raw_value: str) -> tuple[Path, ...]:
    if not raw_value:
        return ()
    return tuple(Path(item.strip()).expanduser() for item in raw_value.split(",") if item.strip())


def infer_output_dir(
    input_path: Path,
    output_root: Path,
    input_roots: tuple[Path, ...],
    output_suffix: str,
) -> Path:
    output_root = output_root.expanduser().resolve()

    for input_root in input_roots:
        try:
            relative_path = input_path.resolve().relative_to(input_root.resolve())
            return output_root / relative_path.parent / f"{input_path.stem}{output_suffix}"
        except ValueError:
            continue

    parts = input_path.parts
    if "img_wait_process" in parts:
        index = parts.index("img_wait_process")
        relative_parent = Path(*parts[index + 1 : -1]) if index + 1 < len(parts) - 1 else Path()
        if str(relative_parent) != ".":
            return output_root / relative_parent / f"{input_path.stem}{output_suffix}"

    return output_root / f"{input_path.stem}{output_suffix}"


def load_rgb_image(path: Path) -> Image.Image:
    require_pillow()
    try:
        image = Image.open(path)
        image = ImageOps.exif_transpose(image)
        return image.convert("RGB")
    except Exception as exc:
        raise ValueError(f"Could not open input image: {path}") from exc


def get_view_files(input_path: Path) -> dict[str, str]:
    return {view: f"{input_path.stem}_{view}.png" for view in VIEW_FILES}


def ensure_output_dir(
    path: str,
    overwrite: bool,
    view_files: dict[str, str] | None = None,
) -> Path:
    output_dir = Path(path).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    if not overwrite:
        existing = [output_dir / name for name in (view_files or VIEW_FILES).values()]
        conflicts = [item for item in existing if item.exists()]
        if conflicts:
            joined = ", ".join(str(item) for item in conflicts)
            raise FileExistsError(f"Output files already exist and --no-overwrite was set: {joined}")

    return output_dir


def resolve_device(device_arg: str) -> str:
    try:
        import torch
    except ImportError as exc:
        raise RuntimeError("Missing torch. Install dependencies with: pip install -r requirements.txt") from exc

    if device_arg != "auto":
        if device_arg.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError(f"Requested device '{device_arg}', but CUDA is not available.")
        if device_arg == "cpu":
            log("WARNING: CUDA is not available or CPU was selected. Qwen-Image-Edit CPU inference is very slow.")
        return device_arg

    if torch.cuda.is_available():
        return "cuda"

    log("WARNING: CUDA is not available. Qwen-Image-Edit CPU inference will be extremely slow.")
    return "cpu"


def resolve_dtype(dtype_arg: str, device: str) -> Any:
    import torch

    if dtype_arg == "auto":
        return torch.bfloat16 if device.startswith("cuda") else torch.float32
    return {
        "bfloat16": torch.bfloat16,
        "float16": torch.float16,
        "float32": torch.float32,
    }[dtype_arg]


def read_local_model_class(model_ref: str) -> str | None:
    model_path = Path(model_ref).expanduser()
    index_path = model_path / "model_index.json"
    if not index_path.exists():
        return None
    try:
        data = json.loads(index_path.read_text(encoding="utf-8"))
    except Exception:
        return None
    class_name = data.get("_class_name")
    return class_name if isinstance(class_name, str) else None


def get_pipeline_class(kind: str) -> type[Any]:
    try:
        import diffusers
    except ImportError as exc:
        raise RuntimeError(
            "Missing diffusers. Install dependencies with: pip install -r requirements.txt"
        ) from exc

    class_name = {
        "plus": "QwenImageEditPlusPipeline",
        "base": "QwenImageEditPipeline",
    }[kind]
    pipeline_class = getattr(diffusers, class_name, None)
    if pipeline_class is None:
        raise RuntimeError(
            f"Installed diffusers does not provide {class_name}. "
            "Install the latest diffusers from GitHub as listed in requirements.txt."
        )
    return pipeline_class


def pipeline_order(model_ref: str, requested: str) -> tuple[str, ...]:
    if requested != "auto":
        return (requested,)

    local_class = read_local_model_class(model_ref)
    if local_class == "QwenImageEditPlusPipeline":
        return ("plus", "base")
    if local_class == "QwenImageEditPipeline":
        return ("base", "plus")
    if "2511" in model_ref or "Plus" in model_ref:
        return ("plus", "base")
    return ("base", "plus")


def model_refs(primary: str, fallback: str) -> tuple[str, ...]:
    refs = [primary]
    if fallback and fallback != primary:
        refs.append(fallback)
    return tuple(refs)


def configure_pipeline(pipe: Any, device: str, args: argparse.Namespace) -> None:
    if hasattr(pipe, "set_progress_bar_config"):
        pipe.set_progress_bar_config(disable=not args.progress)

    if args.vae_tiling:
        vae = getattr(pipe, "vae", None)
        if vae is not None and hasattr(vae, "enable_tiling"):
            vae.enable_tiling()

    if args.attention_slicing and hasattr(pipe, "enable_attention_slicing"):
        pipe.enable_attention_slicing()

    if device.startswith("cuda") and args.sequential_cpu_offload:
        if not hasattr(pipe, "enable_sequential_cpu_offload"):
            raise RuntimeError("This pipeline does not support sequential CPU offload.")
        pipe.enable_sequential_cpu_offload()
    elif device.startswith("cuda") and args.cpu_offload:
        if not hasattr(pipe, "enable_model_cpu_offload"):
            raise RuntimeError("This pipeline does not support model CPU offload.")
        pipe.enable_model_cpu_offload()
    else:
        pipe.to(device)


def load_pipeline(args: argparse.Namespace, device: str, dtype: Any) -> LoadedPipeline:
    errors: list[str] = []

    for model_ref in model_refs(args.model, args.fallback_model):
        for kind in pipeline_order(model_ref, args.pipeline):
            try:
                pipeline_class = get_pipeline_class(kind)
                log(f"Loading {kind} pipeline from {model_ref}")
                pipe = pipeline_class.from_pretrained(
                    model_ref,
                    torch_dtype=dtype,
                    local_files_only=args.local_files_only,
                )
                configure_pipeline(pipe, device, args)
                return LoadedPipeline(model_ref=model_ref, pipeline_kind=kind, pipeline=pipe)
            except Exception as exc:
                errors.append(f"{model_ref} [{kind}]: {exc}")
                log(f"Failed to load {model_ref} with {kind} pipeline.")

    raise RuntimeError("Could not load any Qwen-Image-Edit pipeline:\n" + "\n".join(errors))


def accepts_kwarg(signature: inspect.Signature, name: str) -> bool:
    if name in signature.parameters:
        return True
    return any(param.kind == inspect.Parameter.VAR_KEYWORD for param in signature.parameters.values())


def filtered_call(pipe: Any, kwargs: dict[str, Any]) -> Any:
    call = pipe.__call__
    try:
        signature = inspect.signature(call)
    except (TypeError, ValueError):
        return pipe(**kwargs)

    accepted = {key: value for key, value in kwargs.items() if accepts_kwarg(signature, key)}
    return pipe(**accepted)


def fit_to_canvas(image: Image.Image, size: tuple[int, int]) -> Image.Image:
    require_pillow()
    image = image.convert("RGB")
    if image.size == size:
        return image
    fitted = ImageOps.contain(image, size)
    canvas = Image.new("RGB", size, "white")
    x = (size[0] - fitted.width) // 2
    y = (size[1] - fitted.height) // 2
    canvas.paste(fitted, (x, y))
    return canvas


def find_subject_bbox(
    image: Image.Image,
    *,
    white_threshold: int = HUNYUAN_WHITE_THRESHOLD,
) -> tuple[int, int, int, int] | None:
    """Bounding box of non-white pixels (proxy for the main object)."""
    require_pillow()
    gray = image.convert("L")
    mask = gray.point(lambda p: 255 if p < white_threshold else 0)
    return mask.getbbox()


def subject_area_ratio(image: Image.Image) -> float:
    bbox = find_subject_bbox(image)
    if bbox is None:
        return 0.0
    w, h = image.size
    bw = bbox[2] - bbox[0]
    bh = bbox[3] - bbox[1]
    return (bw * bh) / (w * h)


def enforce_min_resolution(image: Image.Image, min_side: int = HUNYUAN_MIN_SIDE) -> Image.Image:
    require_pillow()
    w, h = image.size
    if w >= min_side and h >= min_side:
        return image
    scale = max(min_side / w, min_side / h)
    new_size = (max(min_side, math.ceil(w * scale)), max(min_side, math.ceil(h * scale)))
    log(f"Upscaling to meet Hunyuan min resolution {min_side}px: {image.size} -> {new_size}")
    return image.resize(new_size, Image.Resampling.LANCZOS)


def min_resolution_size(size: tuple[int, int], min_side: int = HUNYUAN_MIN_SIDE) -> tuple[int, int]:
    w, h = size
    if w >= min_side and h >= min_side:
        return size
    scale = max(min_side / w, min_side / h)
    return (max(min_side, math.ceil(w * scale)), max(min_side, math.ceil(h * scale)))


def enforce_subject_presence(
    image: Image.Image,
    *,
    min_ratio: float,
    target_ratio: float,
) -> Image.Image:
    """Scale up a small subject so it occupies enough of the frame for Hunyuan 3D."""
    require_pillow()
    image = image.convert("RGB")
    ratio = subject_area_ratio(image)
    if ratio >= min_ratio:
        return image

    bbox = find_subject_bbox(image)
    if bbox is None:
        log("WARNING: Could not detect subject bbox; subject scale not adjusted.")
        return image

    w, h = image.size
    crop = image.crop(bbox)
    bw, bh = crop.size
    if bw <= 0 or bh <= 0:
        return image

    scale = (target_ratio * w * h / (bw * bh)) ** 0.5
    new_w = max(1, min(w, int(round(bw * scale))))
    new_h = max(1, min(h, int(round(bh * scale))))
    resized = crop.resize((new_w, new_h), Image.Resampling.LANCZOS)
    canvas = Image.new("RGB", (w, h), "white")
    x = (w - new_w) // 2
    y = (h - new_h) // 2
    canvas.paste(resized, (x, y))
    log(
        f"Scaled subject for Hunyuan 3D: bbox area {ratio:.1%} -> "
        f"{subject_area_ratio(canvas):.1%} (target ~{target_ratio:.0%})"
    )
    return canvas


def prepare_hunyuan_view(
    image: Image.Image,
    canvas_size: tuple[int, int],
    args: argparse.Namespace,
) -> Image.Image:
    """Fit to canvas and apply Hunyuan 3D size/subject constraints."""
    if not args.no_hunyuan_constraints:
        canvas_size = min_resolution_size(canvas_size)
    out = fit_to_canvas(image, canvas_size)
    if args.no_hunyuan_constraints:
        return out
    out = enforce_min_resolution(out)
    if out.size != canvas_size:
        out = fit_to_canvas(out, canvas_size)
    out = enforce_subject_presence(
        out,
        min_ratio=args.min_subject_ratio,
        target_ratio=args.target_subject_ratio,
    )
    out = enforce_min_resolution(out)
    return out


def save_png_within_size_limit(path: Path, image: Image.Image, max_bytes: int) -> None:
    """Save PNG, reducing compression / scale until under max_bytes."""
    require_pillow()
    image = image.convert("RGB")
    path.parent.mkdir(parents=True, exist_ok=True)

    for compress_level in (6, 9):
        image.save(path, format="PNG", optimize=True, compress_level=compress_level)
        if path.stat().st_size <= max_bytes:
            return

    current = image
    for _ in range(8):
        w, h = current.size
        new_size = (max(HUNYUAN_MIN_SIDE, int(w * 0.9)), max(HUNYUAN_MIN_SIDE, int(h * 0.9)))
        if new_size == current.size:
            break
        log(f"PNG still >{max_bytes // (1024 * 1024)}MB; downscaling to {new_size}")
        current = current.resize(new_size, Image.Resampling.LANCZOS)
        current.save(path, format="PNG", optimize=True, compress_level=9)
        if path.stat().st_size <= max_bytes:
            return

    size_mb = path.stat().st_size / (1024 * 1024)
    raise RuntimeError(
        f"Could not save {path} under {max_bytes // (1024 * 1024)}MB "
        f"(final size {size_mb:.2f} MB)"
    )


def verify_hunyuan_image(
    path: Path,
    *,
    max_bytes: int = HUNYUAN_MAX_BYTES,
    min_side: int = HUNYUAN_MIN_SIDE,
) -> tuple[int, int]:
    size = verify_image(path)
    file_size = path.stat().st_size
    if file_size > max_bytes:
        raise RuntimeError(
            f"{path} exceeds Hunyuan max file size "
            f"({file_size / (1024 * 1024):.2f} MB > {max_bytes // (1024 * 1024)} MB)"
        )
    if size[0] < min_side or size[1] < min_side:
        raise RuntimeError(
            f"{path} resolution {size[0]}x{size[1]} is below Hunyuan minimum {min_side}px"
        )
    return size


def concat_front_back(front: Image.Image, back: Image.Image) -> Image.Image:
    require_pillow()
    width, height = front.size
    back = fit_to_canvas(back, front.size)
    canvas = Image.new("RGB", (width * 2, height), "white")
    canvas.paste(front, (0, 0))
    canvas.paste(back, (width, 0))
    return canvas


def make_reference(
    loaded: LoadedPipeline,
    refs: list[Image.Image],
    output_dir: Path,
    save_reference: bool,
) -> Image.Image | list[Image.Image]:
    if loaded.pipeline_kind == "plus":
        return refs
    if len(refs) == 1:
        return refs[0]

    reference = concat_front_back(refs[0], refs[1])
    if save_reference:
        reference.save(output_dir / "reference_front_back.png")
    return reference


def generate_one_view(
    loaded: LoadedPipeline,
    refs: list[Image.Image],
    prompt: str,
    args: argparse.Namespace,
    output_dir: Path,
    seed_offset: int,
) -> Image.Image:
    import torch

    reference = make_reference(loaded, refs, output_dir, args.save_reference)
    generator = torch.manual_seed(args.seed + seed_offset)
    kwargs = {
        "image": reference,
        "prompt": prompt,
        "generator": generator,
        "true_cfg_scale": args.true_cfg_scale,
        "negative_prompt": args.negative_prompt,
        "num_inference_steps": args.num_inference_steps,
        "guidance_scale": args.guidance_scale,
        "num_images_per_prompt": 1,
    }
    with torch.inference_mode():
        output = filtered_call(loaded.pipeline, kwargs)

    images = getattr(output, "images", None)
    if not images:
        raise RuntimeError("Pipeline did not return any images.")
    return images[0].convert("RGB")


def save_view(
    path: Path,
    image: Image.Image,
    size: tuple[int, int],
    args: argparse.Namespace,
) -> None:
    prepared = prepare_hunyuan_view(image, size, args)
    max_bytes = int(args.max_output_mb * 1024 * 1024)
    if args.no_hunyuan_constraints:
        prepared.save(path)
        verify_image(path)
        return
    save_png_within_size_limit(path, prepared, max_bytes)
    verify_hunyuan_image(path, max_bytes=max_bytes)


def verify_image(path: Path) -> tuple[int, int]:
    require_pillow()
    if not path.exists():
        raise FileNotFoundError(f"Expected output file was not created: {path}")
    try:
        with Image.open(path) as image:
            image.load()
            return image.size
    except Exception as exc:
        raise RuntimeError(f"Output file exists but cannot be opened: {path}") from exc


def verify_outputs(
    output_dir: Path,
    view_files: dict[str, str],
    args: argparse.Namespace,
) -> None:
    max_bytes = int(args.max_output_mb * 1024 * 1024)
    sizes: dict[str, tuple[int, int]] = {}
    for view, filename in view_files.items():
        path = output_dir / filename
        if args.no_hunyuan_constraints:
            sizes[view] = verify_image(path)
        else:
            sizes[view] = verify_hunyuan_image(path, max_bytes=max_bytes)
    unique_sizes = set(sizes.values())
    if len(unique_sizes) != 1:
        raise RuntimeError(f"Four view images must have identical sizes, got: {sizes}")


def generate_views_for_image(
    input_path: Path,
    output_dir: Path,
    loaded: LoadedPipeline,
    args: argparse.Namespace,
) -> None:
    view_files = get_view_files(input_path)
    output_dir = ensure_output_dir(str(output_dir), args.overwrite, view_files)
    front = load_rgb_image(input_path)
    target_size = front.size if args.no_hunyuan_constraints else min_resolution_size(front.size)

    front_path = output_dir / view_files["front"]
    save_view(front_path, front, target_size, args)

    log(f"Generating back view: {input_path}")
    back = generate_one_view(loaded, [front], BACK_PROMPT, args, output_dir, seed_offset=1)
    save_view(output_dir / view_files["back"], back, target_size, args)

    log(f"Generating left view: {input_path}")
    left = generate_one_view(loaded, [front, back], LEFT_PROMPT, args, output_dir, seed_offset=2)
    save_view(output_dir / view_files["left"], left, target_size, args)

    log(f"Generating right view: {input_path}")
    right = generate_one_view(loaded, [front, back], RIGHT_PROMPT, args, output_dir, seed_offset=3)
    save_view(output_dir / view_files["right"], right, target_size, args)

    verify_outputs(output_dir, view_files, args)


def main() -> int:
    args = parse_args()

    try:
        input_paths = [resolve_input_image(item) for item in args.input]
        output_root = Path(args.output_dir)
        input_roots = parse_input_roots(args.input_root)

        device = resolve_device(args.device)
        dtype = resolve_dtype(args.dtype, device)
        loaded = load_pipeline(args, device, dtype)
        log(f"Using model: {loaded.model_ref}")
        log(f"Using pipeline: {loaded.pipeline_kind}")
        if not args.no_hunyuan_constraints:
            log(
                "Hunyuan 3D constraints enabled: prompts + post-process "
                f"(min {HUNYUAN_MIN_SIDE}px, max {args.max_output_mb}MB, "
                f"min subject ratio {args.min_subject_ratio:.0%})"
            )

        exit_code = 0
        for input_path in input_paths:
            output_dir = (
                output_root.expanduser().resolve()
                if len(input_paths) == 1
                else infer_output_dir(input_path, output_root, input_roots, args.output_suffix)
            )
            try:
                generate_views_for_image(input_path, output_dir, loaded, args)
                print(f"Generated four views in: {output_dir}")
            except Exception as exc:
                print(f"ERROR [{input_path}]: {exc}", file=sys.stderr)
                exit_code = 1

        return exit_code
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
