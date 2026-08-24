#!/usr/bin/env python3
"""
Material classification using LongCat multimodal API.

Analyzes four orthographic views (front/back/left/right) of an object and
classifies its primary material as: rigid_body, glass, or plastic.

Usage:
    python classify_material.py [path_to_views_or_3D_dir]

Default input: assets/book/share/ScreenShot_2026-05-12_164901_551_3D
Output:        <views_dir>/material_classification.json
"""

from __future__ import annotations

import argparse
import base64
import json
import sys
from pathlib import Path

import requests

LONGCAT_API_KEY = "ak_2RR9sg9a30tx0vz9wR8Mo3HF7Oj9H"
LONGCAT_API_URL = "https://api.longcat.chat/openai/v1/chat/completions"
MODEL = "LongCat-Flash-Omni-2603"

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_INPUT = (
    REPO_ROOT / "assets" / "book" / "share" / "ScreenShot_2026-05-12_164901_551_3D"
)

CLASSIFY_PROMPT = """\
You are an expert in material science and computer vision. \
I am providing four orthographic views (front, back, left, right) of the same 3D object.

Carefully analyze all visual characteristics including:
- Surface finish: matte / glossy / transparent / reflective / frosted
- Opacity: fully opaque / semi-transparent / transparent
- Color and sheen patterns
- Surface texture: smooth, grainy, rough, fibrous, printed
- Structural form implying hardness or flexibility
- Manufacturing clues: seams, labels, print marks, embossing

Based on the above, classify the object's primary material into EXACTLY ONE category:
- "rigid_body": hard opaque solids — metal, wood, ceramic, stone, cardboard, fabric, \
hard composite; objects that are clearly non-transparent and non-plastic
- "glass": transparent or strongly translucent glass-like materials — glass bottles, \
crystal vases, transparent containers, glassware
- "plastic": plastic, rubber, or synthetic polymer objects — plastic bottles, \
packaging, soft/hard plastic containers, rubber items

Return ONLY a valid JSON object with NO extra text, NO markdown fences, following this schema exactly:
{
  "material_category": "<rigid_body|glass|plastic>",
  "confidence": <float 0.0–1.0>,
  "material_description": "<one concise sentence describing the observed material>",
  "visual_cues": ["<cue 1>", "<cue 2>", "<cue 3>"],
  "probabilities": {
    "rigid_body": <float 0.0–1.0>,
    "glass": <float 0.0–1.0>,
    "plastic": <float 0.0–1.0>
  }
}
"""


def encode_image_b64(path: Path) -> str:
    with open(path, "rb") as fh:
        return base64.b64encode(fh.read()).decode("utf-8")


def find_views_dir(input_path: Path) -> Path:
    """
    Resolve the *_views directory for a given input path.

    Accepts:
      - A directory ending in _views  → returned as-is
      - A directory ending in _3D     → sibling directory with _views suffix
      - Any other directory           → returned as-is (caller must have views)
    """
    input_path = input_path.expanduser().resolve()
    name = input_path.name

    if name.endswith("_views"):
        return input_path

    if name.endswith("_3D"):
        views_name = name[:-3] + "_views"
        views_path = input_path.parent / views_name
        if views_path.is_dir():
            return views_path
        raise FileNotFoundError(
            f"Expected views directory not found: {views_path}\n"
            f"Run generate_views.py first to create four-view images for this object."
        )

    return input_path


def find_view_images(views_dir: Path) -> dict[str, Path]:
    """
    Locate front / back / left / right PNG images inside views_dir.

    Accepts two naming conventions:
      - Simple:  front.png, back.png, left.png, right.png
      - Prefixed: <stem>_front.png, <stem>_back.png, …
    """
    images: dict[str, Path] = {}
    for view in ("front", "back", "left", "right"):
        candidate_exact = views_dir / f"{view}.png"
        if candidate_exact.exists():
            images[view] = candidate_exact
            continue

        matches = sorted(views_dir.glob(f"*_{view}.png"))
        if len(matches) == 1:
            images[view] = matches[0]
        elif len(matches) > 1:
            raise ValueError(
                f"Multiple '{view}' view images found in {views_dir}: "
                + ", ".join(m.name for m in matches)
            )

    missing = [v for v in ("front", "back", "left", "right") if v not in images]
    if missing:
        available = sorted(p.name for p in views_dir.iterdir() if p.suffix == ".png")
        raise FileNotFoundError(
            f"Missing view images in {views_dir}: {missing}\n"
            f"Available PNG files: {available}"
        )

    return images


def build_api_request(images: dict[str, Path]) -> dict:
    """Assemble the multimodal chat completion request body.

    LongCat-Flash-Omni-2603 uses 'input_image' content blocks, NOT OpenAI's
    'image_url' format.  The image data must be a raw base64 string (no
    'data:image/…;base64,' prefix) wrapped in an array under 'data'.

    References: https://longcat.chat/platform/docs/zh/APIDocs.html
    """
    content: list[dict] = [
        {
            "type": "text",
            "text": (
                "Here are four orthographic views of the same 3D object. "
                "Please analyze them carefully."
            ),
        }
    ]

    for view in ("front", "back", "left", "right"):
        b64 = encode_image_b64(images[view])
        content.append({"type": "text", "text": f"{view.capitalize()} view:"})
        content.append(
            {
                "type": "input_image",
                "input_image": {
                    "type": "base64",
                    "data": [b64],
                },
            }
        )

    content.append({"type": "text", "text": CLASSIFY_PROMPT})

    return {
        "model": MODEL,
        "messages": [{"role": "user", "content": content}],
        "max_tokens": 1024,
        "temperature": 0.1,
        # Omni defaults to stream=true with audio; force text-only non-streaming.
        "stream": False,
        "output_modalities": ["text"],
    }


def call_longcat_api(request_body: dict) -> str:
    """Post the request to LongCat and return the assistant message text."""
    headers = {
        "Authorization": f"Bearer {LONGCAT_API_KEY}",
        "Content-Type": "application/json",
    }
    resp = requests.post(
        LONGCAT_API_URL,
        headers=headers,
        json=request_body,
        timeout=180,
    )
    resp.raise_for_status()
    data = resp.json()
    return data["choices"][0]["message"]["content"]


def parse_json_response(text: str) -> dict:
    """Extract and parse the JSON object from the model response.

    Handles:
    - Pure JSON responses
    - Responses wrapped in ```json … ``` or ``` … ``` fences
    - Responses with prose before/after the JSON object
    """
    text = text.strip()

    # Strip markdown code fences if present
    if text.startswith("```"):
        lines = text.splitlines()
        inner_lines = lines[1:-1] if lines[-1].strip() == "```" else lines[1:]
        text = "\n".join(inner_lines).strip()

    # Try direct parse first
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        pass

    # Fall back: extract the outermost { … } block
    start = text.find("{")
    end = text.rfind("}")
    if start != -1 and end != -1 and end > start:
        try:
            return json.loads(text[start : end + 1])
        except json.JSONDecodeError:
            pass

    raise ValueError(
        f"Could not parse JSON from model response.\n"
        f"Raw response (first 500 chars):\n{text[:500]}"
    )


def classify_object(input_path: Path) -> tuple[dict, Path]:
    """Run the full classification pipeline and return (result_dict, views_dir)."""
    views_dir = find_views_dir(input_path)
    print(f"[classify] Views directory : {views_dir}")

    images = find_view_images(views_dir)
    print(f"[classify] Images found    : {', '.join(f'{k}={v.name}' for k, v in images.items())}")

    request_body = build_api_request(images)
    print(f"[classify] Calling LongCat API ({MODEL}) …")

    raw = call_longcat_api(request_body)
    print(f"[classify] Response received ({len(raw)} chars)")

    result = parse_json_response(raw)

    result["_meta"] = {
        "views_dir": str(views_dir),
        "images": {k: v.name for k, v in images.items()},
        "model": MODEL,
        "api_url": LONGCAT_API_URL,
    }

    return result, views_dir


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Classify object material (rigid_body / glass / plastic) from "
            "four-view images using the LongCat multimodal API."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "input",
        nargs="?",
        default=str(DEFAULT_INPUT),
        help=(
            "Path to a *_3D or *_views directory. "
            "If a *_3D directory is given, the sibling *_views directory is used. "
            f"Default: {DEFAULT_INPUT}"
        ),
    )
    parser.add_argument(
        "--output",
        default=None,
        metavar="FILE",
        help=(
            "Output JSON file path. "
            "Default: <views_dir>/material_classification.json"
        ),
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite existing output file (default: skip if exists).",
    )
    args = parser.parse_args()

    try:
        input_path = Path(args.input).expanduser().resolve()
        if not input_path.exists():
            print(f"ERROR: Input path does not exist: {input_path}", file=sys.stderr)
            return 1

        result, views_dir = classify_object(input_path)

        output_path = (
            Path(args.output).expanduser().resolve()
            if args.output
            else views_dir / "material_classification.json"
        )

        if output_path.exists() and not args.overwrite:
            print(
                f"[classify] Output already exists (use --overwrite to replace): {output_path}"
            )
            return 0

        output_path.write_text(
            json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        print(f"[classify] Saved → {output_path}")
        print(
            f"[classify] Result : {result.get('material_category')} "
            f"(confidence={result.get('confidence'):.2f})"
        )
        return 0

    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
