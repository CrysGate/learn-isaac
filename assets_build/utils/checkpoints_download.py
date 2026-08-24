import os
from pathlib import Path

os.environ["HF_ENDPOINT"] = "https://hf-mirror.com"
from huggingface_hub import snapshot_download

# HF_TOKEN='hf_...'
os.environ["HF_TOKEN"] = ""

# 仓库根目录（本文件位于 assets_build/utils/）
REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_HF_CACHE = REPO_ROOT / "checkpoints_download"
os.environ.setdefault("HF_HOME", str(DEFAULT_HF_CACHE))

LOCAL_MODEL_DIR = DEFAULT_HF_CACHE / "hunyuan3d_v2-1"

# snapshot_download("TianxingChen/RoboTwin2.0", repo_type="dataset",
#     local_dir=str(REPO_ROOT / "benchmark_dataset" / "robotwin2"), resume_download=True)
snapshot_download(
    repo_id="tencent/Hunyuan3D-2.1",
    local_dir=str(LOCAL_MODEL_DIR),
    # repo_type="model",
    resume_download=True,
    max_workers=8,
)
