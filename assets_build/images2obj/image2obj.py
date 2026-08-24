import argparse
import sys
import traceback
from pathlib import Path

from playwright.sync_api import Playwright, sync_playwright

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_VIEWS_DIR = (
    PROJECT_ROOT / "assets" / "book" / "share" / "ScreenShot_2026-05-12_164901_551_views"
)
STATE_FILE = Path(__file__).parent / "tencent_state.json"
SUPPORTED_SUFFIXES = (".png", ".jpg", ".jpeg", ".webp")
VIEW_NAMES = ("front", "back", "left", "right")


def find_view_image(views_dir: Path, view_name: str) -> Path | None:
    for suffix in SUPPORTED_SUFFIXES:
        candidate = views_dir / f"{view_name}{suffix}"
        if candidate.exists():
            return candidate
        prefixed = sorted(views_dir.glob(f"*_{view_name}{suffix}"))
        if prefixed:
            return prefixed[0]
    return None


def collect_views(views_dir: Path) -> dict[str, Path]:
    views = {}
    for name in VIEW_NAMES:
        path = find_view_image(views_dir, name)
        if path:
            views[name] = path
    if "front" not in views:
        raise FileNotFoundError(f"No front image found in: {views_dir}")
    return views


def derive_output_dir(views_dir: Path) -> Path:
    name = views_dir.name
    out_name = name[: -len("_views")] + "_3D" if name.endswith("_views") else name + "_3D"
    return views_dir.parent / out_name


def perform_first_login(playwright: Playwright) -> bool:
    print("未检测到登录状态，准备进行首次手动登录...")
    browser = playwright.chromium.launch(headless=False)
    context = browser.new_context()
    page = context.new_page()

    page.goto("https://3d.hunyuan.tencent.com/")
    print(">>> 请在弹出的浏览器中手动完成扫码/密码登录 <<<")
    print(">>> 登录成功后，脚本会自动保存状态并进入下一步 <<<")

    try:
        page.wait_for_selector('text="开始体验"', timeout=120000)
    except Exception:
        print("等待登录超时或未找到成功标志，请重试。")
        browser.close()
        return False

    context.storage_state(path=str(STATE_FILE))
    print(f"登录状态已成功保存到 {STATE_FILE}！后续运行将自动跳过登录。")
    context.close()
    browser.close()
    return True


def run_automation(playwright: Playwright, views: dict[str, Path], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    front_path = views["front"]
    back_path = views.get("back")
    left_path = views.get("left")
    right_path = views.get("right")

    print("加载登录状态，开始执行自动化任务...")
    for view_name, path in views.items():
        print(f"  {view_name}: {path}")
    print(f"  输出目录: {output_dir}")

    browser = playwright.chromium.launch(headless=False)
    context = browser.new_context(storage_state=str(STATE_FILE))
    page = context.new_page()

    page.goto("https://3d.hunyuan.tencent.com/")
    try:
        page.get_by_role("button", name="开始体验").click(timeout=5000)
    except Exception:
        pass  # already in workspace

    page.goto("https://3d.hunyuan.tencent.com/")
    page.get_by_text("多张图片").click()
    page.get_by_role("button", name="添加多视图（Min2，Max8）").click()

    print(f"上传正图（front）: {front_path.name}")
    with page.expect_file_chooser() as fc:
        page.get_by_text("上传正图").click()
    fc.value.set_files(str(front_path))
    page.wait_for_timeout(2000)

    if back_path is not None:
        print(f"上传背图（back）: {back_path.name}")
        with page.expect_file_chooser() as fc:
            page.get_by_text("上传背图").click()
        fc.value.set_files(str(back_path))
        page.wait_for_timeout(2000)

    if left_path is not None:
        print(f"上传左图（left）: {left_path.name}")
        with page.expect_file_chooser() as fc:
            page.get_by_text("上传左图").click()
        fc.value.set_files(str(left_path))
        page.wait_for_timeout(2000)

    if right_path is not None:
        print(f"上传右图（right）: {right_path.name}")
        with page.expect_file_chooser() as fc:
            page.get_by_text("上传右图").click()
        fc.value.set_files(str(right_path))
        page.wait_for_timeout(2000)

    page.get_by_role("button", name="立即生成").click()
    print("已提交生成请求，等待处理中（最长 10 分钟）...")

    page.get_by_role("button", name="GLB").click(timeout=600000)
    page.get_by_text("OBJ").click()

    with page.expect_download() as download_info:
        page.get_by_role("button", name="下载").click()

    download = download_info.value
    save_path = output_dir / download.suggested_filename
    download.save_as(str(save_path))
    print(f"下载完成，文件已保存为: {save_path}")

    context.close()
    browser.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="从多视图图片目录生成 3D 模型（OBJ），使用腾讯混元 3D 网页端自动化。"
    )
    parser.add_argument(
        "--views-dir",
        type=Path,
        default=DEFAULT_VIEWS_DIR,
        metavar="DIR",
        help=(
            f"视图图片目录，含 *_front.*、*_back.*、*_left.*、*_right.* 等。"
            f"默认: {DEFAULT_VIEWS_DIR}"
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        metavar="DIR",
        help="输出目录，默认与 --views-dir 同级，将 _views 后缀替换为 _3D",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    views_dir = args.views_dir.resolve()

    try:
        if not views_dir.is_dir():
            raise NotADirectoryError(f"views-dir 不存在或不是目录: {views_dir}")

        views = collect_views(views_dir)
        output_dir = args.output_dir.resolve() if args.output_dir else derive_output_dir(views_dir)

        with sync_playwright() as playwright:
            if not STATE_FILE.exists():
                success = perform_first_login(playwright)
                if not success:
                    print("ERROR: 首次登录失败或超时，未保存登录状态。", file=sys.stderr)
                    return 1
            run_automation(playwright, views, output_dir)
        return 0
    except Exception as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        traceback.print_exc(file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
