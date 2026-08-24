from pathlib import Path
from multi_view_api.multi_view import per_img_aibuild_funtion

IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png")
VIEW_NAMES = ("front", "back", "left", "right")


def iter_input_images(img_wait_root_path):
    for image_path in sorted(img_wait_root_path.rglob("*")):
        if image_path.is_file() and image_path.suffix.lower() in IMAGE_SUFFIXES:
            yield image_path


def output_dir_for_image(image_path, img_wait_root_path, assets_output_path):
    relative_path = image_path.relative_to(img_wait_root_path)
    if len(relative_path.parts) < 3:
        raise ValueError(f"Input image should be under <class>/<variant>/file: {image_path}")

    relative_parent = relative_path.parent
    return assets_output_path / relative_parent / f"{image_path.stem}_views"


def output_complete(output_dir, image_stem):
    return all((output_dir / f"{image_stem}_{view}.png").is_file() for view in VIEW_NAMES)


if __name__ == "__main__":
    img_wait_root_path = "./img_wait_process"
    img_wait_root_path = Path(img_wait_root_path)

    assets_output_path = "./assets"
    assets_output_path = Path(assets_output_path)
    assets_output_path.mkdir(parents=True, exist_ok=True)

    images = list(iter_input_images(img_wait_root_path))
    skipped = 0

    for image_path in images:
        output_dir = output_dir_for_image(image_path, img_wait_root_path, assets_output_path)
        if output_complete(output_dir, image_path.stem):
            print(f"[skip] {image_path}  (all views already exist in: {output_dir})")
            skipped += 1
            continue

        print(f"[queued] {image_path}")
        per_img_aibuild_funtion(image_path, output_dir, output_prefix=image_path.stem)

    print(f"Skipped: {skipped} already processed. Processed: {len(images) - skipped}.")
