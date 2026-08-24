# 以下示例假设当前工作目录为仓库根目录（与 `assets_build` 同级）

export HF_TOKEN='<your_hf_token>'
export HF_ENDPOINT='https://hf-mirror.com'
export HF_HOME="${HF_HOME:-$PWD/checkpoints_download}"

conda activate assets_build
cd assets_build

pip install pydantic -i https://mirrors.tencent.com/xxxx --extra-index-url https://mirrors.tencent.com/pypi/simple/
pip install openai -i https://mirrors.tencent.com/xxxx --extra-index-url https://mirrors.tencent.com/pypi/simple/

pip install -r requirements.txt -i https://mirrors.tencent.com/xxxx --extra-index-url https://mirrors.tencent.com/pypi/simple/
pip install -e . -i https://mirrors.tencent.com/xxxx --extra-index-url https://mirrors.tencent.com/pypi/simple/
pip3 uninstall torch torchvision
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu126
# for texture
cd hy3dgen/texgen/custom_rasterizer
python3 setup.py install
cd ../../..
cd hy3dgen/texgen/differentiable_renderer
python3 setup.py install

# Debian/Ubuntu 示例；RHEL 系列请用 dnf 安装对应的 mesa / OpenGL 包
apt install libgl1-mesa-glx
apt install mesa-utils
apt install libopengl0 libglu1-mesa

# 按需检查 pymeshlab 等 native 依赖：ldd <插件 .so 路径>

python Hunyuan3D-2/examples/textured_shape_gen_multiview.py
