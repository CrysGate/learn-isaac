import argparse
import base64
import io
import json
import os
import shutil
import sys
import time
import zipfile
from pathlib import Path
from urllib import error, parse, request


PROJECT_ROOT = Path(__file__).resolve().parents[1]
VENDOR_DIR = Path(__file__).resolve().parent / ".vendor"
if VENDOR_DIR.exists():
    sys.path.insert(0, str(VENDOR_DIR))

DEFAULT_BASE_URL = os.getenv("HUNYUAN3D_BASE_URL", "https://api.ai3d.cloud.tencent.com")
DEFAULT_MODEL = os.getenv("HUNYUAN3D_MODEL", "3.1")
DEFAULT_GENERATE_TYPE = os.getenv("HUNYUAN3D_GENERATE_TYPE", "Normal")
DEFAULT_FACE_COUNT = int(os.getenv("HUNYUAN3D_FACE_COUNT", "500000"))
DEFAULT_ENABLE_PBR = os.getenv("HUNYUAN3D_ENABLE_PBR", "").lower() in {"1", "true", "yes", "on"}
DEFAULT_RESULT_FORMAT = os.getenv("HUNYUAN3D_RESULT_FORMAT")
DEFAULT_SUBMIT_TIMEOUT = int(os.getenv("HUNYUAN3D_SUBMIT_TIMEOUT_SECONDS", "300"))
DEFAULT_QUERY_TIMEOUT = int(os.getenv("HUNYUAN3D_QUERY_TIMEOUT_SECONDS", "120"))
DEFAULT_POLL_INTERVAL = float(os.getenv("HUNYUAN3D_POLL_INTERVAL_SECONDS", "5"))
DEFAULT_TIMEOUT = int(os.getenv("HUNYUAN3D_TIMEOUT_SECONDS", "1800"))
DEFAULT_MAX_IMAGE_SIDE = int(os.getenv("HUNYUAN3D_MAX_IMAGE_SIDE", "1024"))
DEFAULT_JPEG_QUALITY = int(os.getenv("HUNYUAN3D_JPEG_QUALITY", "95"))
SUPPORTED_SUFFIXES = (".png", ".jpg", ".jpeg", ".webp")
REQUIRED_VIEWS = ("front", "left", "right", "back")
STATUS_MAP = {
    "queued": "WAIT",
    "in_progress": "RUN",
    "completed": "DONE",
    "failed": "FAIL",
}


class Hunyuan3DAPIError(RuntimeError):
    pass


def load_pillow():
    try:
        from PIL import Image, ImageOps
    except ImportError as exc:
        raise RuntimeError(
            "Pillow is required. Install it with: python -m pip install --target "
            f"{VENDOR_DIR} Pillow"
        ) from exc
    return Image, ImageOps


def compress_image_for_api(path):
    Image, ImageOps = load_pillow()
    with Image.open(path) as image:
        image = ImageOps.exif_transpose(image)
        original_size = image.size
        original_mode = image.mode
        original_bytes = path.stat().st_size

        if max(image.size) > DEFAULT_MAX_IMAGE_SIDE:
            resampling = getattr(Image, "Resampling", Image).LANCZOS
            image.thumbnail((DEFAULT_MAX_IMAGE_SIDE, DEFAULT_MAX_IMAGE_SIDE), resample=resampling)

        if image.mode in {"RGBA", "LA"} or (image.mode == "P" and "transparency" in image.info):
            rgba_image = image.convert("RGBA")
            white_background = Image.new("RGBA", rgba_image.size, (255, 255, 255, 255))
            image = Image.alpha_composite(white_background, rgba_image).convert("RGB")
        elif image.mode != "RGB":
            image = image.convert("RGB")

        compressed_buffer = io.BytesIO()
        image.save(
            compressed_buffer,
            format="JPEG",
            quality=DEFAULT_JPEG_QUALITY,
            optimize=True,
        )
        compressed_bytes = compressed_buffer.getvalue()
        base64_bytes = len(base64.b64encode(compressed_bytes))

        return {
            "base64": base64.b64encode(compressed_bytes).decode("utf-8"),
            "stats": {
                "path": str(path),
                "original_bytes": original_bytes,
                "compressed_bytes": len(compressed_bytes),
                "base64_bytes": base64_bytes,
                "original_size": original_size,
                "compressed_size": image.size,
                "original_mode": original_mode,
                "output_format": "JPEG",
                "jpeg_quality": DEFAULT_JPEG_QUALITY,
            },
        }


def is_relative_to(path, root):
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def to_project_relative(path):
    path = path.resolve()
    if is_relative_to(path, PROJECT_ROOT):
        return path.relative_to(PROJECT_ROOT).as_posix()
    return str(path)


def find_view_image(input_dir, view_name):
    for suffix in SUPPORTED_SUFFIXES:
        candidate = input_dir / f"{view_name}{suffix}"
        if candidate.exists():
            return candidate.resolve()
        prefixed_candidates = sorted(input_dir.glob(f"*_{view_name}{suffix}"))
        if prefixed_candidates:
            return prefixed_candidates[0].resolve()
    return None


def resolve_input_path(input_path):
    input_path = input_path.resolve()
    if not input_path.exists():
        raise FileNotFoundError(f"Input path does not exist: {input_path}")

    if input_path.is_file():
        if input_path.suffix.lower() not in SUPPORTED_SUFFIXES:
            raise FileNotFoundError(
                f"Input file must be an image with one of {SUPPORTED_SUFFIXES}: {input_path}"
            )
        return input_path

    multi_views_dir = input_path / "multi_views"
    if multi_views_dir.is_dir():
        return multi_views_dir.resolve()
    return input_path


def collect_input_views(input_path):
    if input_path.is_file():
        return {"front": input_path.resolve()}

    views = {}
    for view_name in REQUIRED_VIEWS:
        image_path = find_view_image(input_path, view_name)
        if image_path is not None:
            views[view_name] = image_path

    if "front" not in views:
        raise FileNotFoundError(
            f"Input directory must contain front.* or front/left/right/back: {input_path}"
        )

    non_front_views = [view_name for view_name in REQUIRED_VIEWS if view_name != "front" and view_name in views]
    if non_front_views and len(non_front_views) != 3:
        missing = [view_name for view_name in REQUIRED_VIEWS if view_name != "front" and view_name not in views]
        raise FileNotFoundError(
            f"Incomplete multi-view set in {input_path}. Missing required views: {', '.join(missing)}."
        )

    return views


def build_submit_payload(view_paths):
    if DEFAULT_MODEL == "3.1" and DEFAULT_GENERATE_TYPE in {"LowPoly", "Sketch"}:
        raise ValueError("Model 3.1 does not support GenerateType=LowPoly or Sketch.")

    encoded_views = {
        view_name: compress_image_for_api(image_path)
        for view_name, image_path in view_paths.items()
    }
    payload = {
        "Model": DEFAULT_MODEL,
        "GenerateType": DEFAULT_GENERATE_TYPE,
        "ImageBase64": encoded_views["front"]["base64"],
    }

    multi_view_images = [
        {
            "ViewType": view_name,
            "ViewImageBase64": encoded_views[view_name]["base64"],
        }
        for view_name in REQUIRED_VIEWS
        if view_name != "front" and view_name in encoded_views
    ]
    if multi_view_images:
        payload["MultiViewImages"] = multi_view_images

    if DEFAULT_GENERATE_TYPE != "LowPoly":
        payload["FaceCount"] = DEFAULT_FACE_COUNT
    if DEFAULT_GENERATE_TYPE != "Geometry":
        payload["EnablePBR"] = DEFAULT_ENABLE_PBR
    if DEFAULT_RESULT_FORMAT:
        payload["ResultFormat"] = DEFAULT_RESULT_FORMAT
    stats = {view_name: encoded_views[view_name]["stats"] for view_name in encoded_views}
    return payload, stats


def call_hunyuan_3d_api(path, payload, api_key, base_url, timeout):
    endpoint = parse.urljoin(base_url.rstrip("/") + "/", path.lstrip("/"))
    body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    headers = {
        "Authorization": api_key,
        "Content-Type": "application/json",
    }

    req = request.Request(endpoint, data=body, headers=headers, method="POST")
    try:
        with request.urlopen(req, timeout=timeout) as resp:
            raw = resp.read().decode("utf-8")
    except error.HTTPError as exc:
        raw = exc.read().decode("utf-8", errors="replace")
        raise Hunyuan3DAPIError(f"HTTP {exc.code} when calling {endpoint}: {raw}") from exc
    except error.URLError as exc:
        raise Hunyuan3DAPIError(f"Network error when calling {endpoint}: {exc}") from exc

    try:
        return json.loads(raw)
    except json.JSONDecodeError as exc:
        raise Hunyuan3DAPIError(f"Invalid JSON response from {endpoint}: {raw}") from exc


def unwrap_response(data):
    if isinstance(data, dict) and isinstance(data.get("Response"), dict):
        return data["Response"]
    return data


def raise_api_error_if_present(response):
    response = unwrap_response(response)
    error_info = response.get("Error")
    if not isinstance(error_info, dict):
        return response

    error_code = error_info.get("Code") or "UnknownError"
    error_message = error_info.get("Message") or "Unknown error."
    request_id = response.get("RequestId") or error_info.get("RequestId")
    if request_id:
        raise Hunyuan3DAPIError(
            f"{error_code}: {error_message} (RequestId: {request_id})"
        )
    raise Hunyuan3DAPIError(f"{error_code}: {error_message}")


def extract_job_id(response):
    response = raise_api_error_if_present(response)
    job_id = response.get("JobId") or response.get("id")
    if not job_id:
        raise Hunyuan3DAPIError(f"JobId not found in submit response: {response}")
    return job_id


def normalize_status(status):
    if not status:
        return ""
    return STATUS_MAP.get(status, status)


def submit_hunyuan_to_3d_job(payload, api_key, base_url):
    response = call_hunyuan_3d_api(
        "/v1/ai3d/submit",
        payload,
        api_key,
        base_url,
        timeout=DEFAULT_SUBMIT_TIMEOUT,
    )
    return extract_job_id(response)


def query_hunyuan_to_3d_job(job_id, api_key, base_url):
    response = call_hunyuan_3d_api(
        "/v1/ai3d/query",
        {"JobId": job_id},
        api_key,
        base_url,
        timeout=DEFAULT_QUERY_TIMEOUT,
    )
    return raise_api_error_if_present(response)


def wait_for_job(job_id, api_key, base_url):
    deadline = time.time() + DEFAULT_TIMEOUT
    last_status = None

    while time.time() < deadline:
        response = query_hunyuan_to_3d_job(job_id, api_key, base_url)
        status = normalize_status(response.get("Status") or response.get("status"))
        if status != last_status:
            print(f"[hy3d] job {job_id} status: {status}")
            last_status = status

        if status == "DONE":
            return response
        if status == "FAIL":
            error_code = response.get("ErrorCode") or response.get("error_code") or ""
            error_message = response.get("ErrorMessage") or response.get("error_message") or ""
            raise Hunyuan3DAPIError(
                f"Job {job_id} failed: {error_code} {error_message}".strip()
            )
        time.sleep(DEFAULT_POLL_INTERVAL)

    raise TimeoutError(f"Timed out waiting for job {job_id} after {DEFAULT_TIMEOUT} seconds.")


def read_first(item, *keys):
    for key in keys:
        value = item.get(key)
        if value is not None:
            return value
    return None


def resolve_download_name(base_name, file_item):
    file_type = (read_first(file_item, "Type", "type") or "bin").lower()
    file_url = read_first(file_item, "Url", "url") or ""
    url_suffix = Path(parse.urlsplit(file_url).path).suffix.lower()

    if file_type == "glb":
        return f"{base_name}.glb"
    if file_type == "obj" and url_suffix == ".zip":
        return f"{base_name}.obj.zip"
    if url_suffix and url_suffix != f".{file_type}":
        return f"{base_name}.{file_type}{url_suffix}"
    return f"{base_name}.{file_type}"


def download_url(url, output_path):
    req = request.Request(url, method="GET")
    with request.urlopen(req, timeout=300) as resp:
        output_path.write_bytes(resp.read())


def rewrite_obj_mtllib(obj_path, mtl_name):
    lines = obj_path.read_text(encoding="utf-8").splitlines()
    replaced = False
    rewritten = []
    for line in lines:
        if line.startswith("mtllib "):
            rewritten.append(f"mtllib {mtl_name}")
            replaced = True
        else:
            rewritten.append(line)
    if not replaced:
        rewritten.insert(0, f"mtllib {mtl_name}")
    obj_path.write_text("\n".join(rewritten) + "\n", encoding="utf-8")


def rewrite_mtl_texture_paths(mtl_path, texture_name_map):
    texture_keywords = {
        "map_Ka",
        "map_Kd",
        "map_Ks",
        "map_d",
        "map_bump",
        "bump",
        "disp",
        "decal",
        "refl",
    }
    rewritten = []
    for line in mtl_path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            rewritten.append(line)
            continue

        parts = stripped.split()
        keyword = parts[0]
        if keyword in texture_keywords and len(parts) >= 2:
            original_ref = parts[-1].replace("\\", "/")
            texture_ref = texture_name_map.get(original_ref, Path(original_ref).name)
            prefix = line[: len(line) - len(line.lstrip())]
            rewritten.append(f"{prefix}{' '.join(parts[:-1])} {texture_ref}")
        else:
            rewritten.append(line)

    mtl_path.write_text("\n".join(rewritten) + "\n", encoding="utf-8")


def unpack_obj_bundle(zip_path, output_dir):
    temp_dir = output_dir / "_obj_unpack_tmp"
    if temp_dir.exists():
        shutil.rmtree(temp_dir)
    temp_dir.mkdir(parents=True, exist_ok=True)

    with zipfile.ZipFile(zip_path) as zf:
        zf.extractall(temp_dir)

    all_files = [path for path in temp_dir.rglob("*") if path.is_file()]
    obj_files = [path for path in all_files if path.suffix.lower() == ".obj"]
    if not obj_files:
        raise Hunyuan3DAPIError(f"No OBJ file found in {zip_path}")

    obj_source = obj_files[0]
    mtl_source = next((path for path in all_files if path.suffix.lower() == ".mtl"), None)
    texture_sources = [
        path for path in all_files if path.suffix.lower() in SUPPORTED_SUFFIXES and path != obj_source
    ]

    obj_target = output_dir / "3d.obj"
    shutil.move(str(obj_source), str(obj_target))

    mtl_target = None
    if mtl_source is not None:
        mtl_target = output_dir / "3d.mtl"
        shutil.move(str(mtl_source), str(mtl_target))
        rewrite_obj_mtllib(obj_target, mtl_target.name)

    texture_name_map = {}
    texture_targets = []
    for index, texture_source in enumerate(texture_sources):
        texture_ext = texture_source.suffix.lower() or ".png"
        target_name = f"texture{texture_ext}" if index == 0 else f"texture_{index}{texture_ext}"
        target_path = output_dir / target_name
        if texture_source.resolve() != target_path.resolve():
            shutil.move(str(texture_source), str(target_path))
        texture_targets.append(target_path)
        texture_name_map[texture_source.relative_to(temp_dir).as_posix()] = target_name
        texture_name_map[texture_source.name] = target_name

    if mtl_target is not None:
        rewrite_mtl_texture_paths(mtl_target, texture_name_map)

    shutil.rmtree(temp_dir)
    zip_path.unlink(missing_ok=True)

    return {
        "obj": obj_target.name,
        "mtl": mtl_target.name if mtl_target is not None else None,
        "textures": sorted(path.name for path in texture_targets),
    }


def download_job_results(response, output_dir):
    output_dir.mkdir(parents=True, exist_ok=True)
    result_files = response.get("ResultFile3Ds") or response.get("data") or []
    if not result_files:
        raise Hunyuan3DAPIError(f"Job finished but result files are empty: {response}")

    normalized_files = {"obj": None, "mtl": None, "textures": []}
    for file_item in result_files:
        file_url = read_first(file_item, "Url", "url")
        if not file_url:
            continue

        file_type = (read_first(file_item, "Type", "type") or "").upper()
        output_path = output_dir / resolve_download_name("3d_generated", file_item)
        if file_type == "OBJ" and output_path.suffix.lower() == ".zip":
            download_url(file_url, output_path)
            print(f"[hy3d] downloaded {output_path}")
            normalized_files.update(unpack_obj_bundle(output_path, output_dir))

    if normalized_files["obj"] is None:
        raise Hunyuan3DAPIError("No OBJ zip result found in response.")
    return normalized_files


def print_image_stats(stats):
    total_base64_bytes = 0
    for view_name in REQUIRED_VIEWS:
        if view_name not in stats:
            continue
        view_stats = stats[view_name]
        total_base64_bytes += view_stats["base64_bytes"]
        print(
            f"[hy3d] view {view_name}: "
            f"{view_stats['original_size'][0]}x{view_stats['original_size'][1]} "
            f"{view_stats['original_mode']} {view_stats['original_bytes']} bytes -> "
            f"{view_stats['compressed_size'][0]}x{view_stats['compressed_size'][1]} "
            f"{view_stats['output_format']} {view_stats['compressed_bytes']} bytes "
            f"(base64={view_stats['base64_bytes']})"
        )
    print(
        f"[hy3d] total compressed base64 bytes: {total_base64_bytes} "
        f"(max_side={DEFAULT_MAX_IMAGE_SIDE}, jpeg_quality={DEFAULT_JPEG_QUALITY})"
    )


def load_api_key():
    api_key = os.getenv("HUNYUAN3D_API_KEY") or os.getenv("HUNYUAN_API_KEY")
    if not api_key:
        raise EnvironmentError(
            "Set HUNYUAN3D_API_KEY (or HUNYUAN_API_KEY) before running this script."
        )
    return api_key


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate one 3D asset from a 4-view image directory with Tencent Hunyuan 3D API."
    )
    parser.add_argument("--input-path", "--input-dir", dest="input_path", type=Path, required=True)
    parser.add_argument(
        "--output-path",
        "--output-dir",
        "--output-root",
        dest="output_path",
        type=Path,
        required=True,
    )
    parser.add_argument("--dry-run", action="store_true", help="Only validate input/output resolution.")
    return parser.parse_args()


def main():
    args = parse_args()
    input_path = resolve_input_path(args.input_path)
    view_paths = collect_input_views(input_path)
    output_dir = args.output_path.resolve()

    print(f"[hy3d] input path: {to_project_relative(input_path)}")
    for view_name in REQUIRED_VIEWS:
        if view_name in view_paths:
            print(f"[hy3d] view {view_name}: {to_project_relative(view_paths[view_name])}")
    print(f"[hy3d] output dir: {output_dir}")

    if args.dry_run:
        _, stats = build_submit_payload(view_paths)
        print_image_stats(stats)
        return

    api_key = load_api_key()
    payload, stats = build_submit_payload(view_paths)
    print_image_stats(stats)
    payload_size_mb = len(
        json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    ) / (1024 * 1024)
    print(
        f"[hy3d] submitting job with model={DEFAULT_MODEL}, "
        f"generate_type={DEFAULT_GENERATE_TYPE}, payload_size={payload_size_mb:.2f} MB"
    )

    job_id = submit_hunyuan_to_3d_job(payload, api_key, DEFAULT_BASE_URL)
    print(f"[hy3d] submitted job: {job_id}")

    response = wait_for_job(job_id, api_key, DEFAULT_BASE_URL)
    file_info = download_job_results(response, output_dir)

    print(f"[hy3d] finished.")
    print(output_dir / file_info["obj"])
    if file_info["mtl"] is not None:
        print(output_dir / file_info["mtl"])
    for texture_name in file_info["textures"]:
        print(output_dir / texture_name)


if __name__ == "__main__":
    main()
