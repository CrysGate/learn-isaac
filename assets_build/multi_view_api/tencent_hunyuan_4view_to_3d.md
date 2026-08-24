# 腾讯混元生3D：4 视图生成 3D 模型实现说明

## 1. 需求拆解

你的需求是：

- 输入 4 张图片：`front`、`left`、`right`、`back`
- 调用腾讯混元生3D官方 API
- 生成一个 3D 模型结果

这里先澄清一件事：

- 这个 API 生成的不是“3D 图片”
- 而是 **3D 模型文件**
- 常见返回结果包括 `OBJ zip`、`GLB`，查询结果里还可能带 `PreviewImageUrl`

## 2. 按官方文档，4 视图应该怎么传

这是最关键的一点。

根据官方文档：

- `ImageBase64` / `ImageUrl` / `Prompt` 三选一必填
- `Prompt` 不能和 `ImageBase64` / `ImageUrl` 同时传
- `MultiViewImages` 里支持的视角类型包括：
  - `left`
  - `right`
  - `back`
  - `top`（仅 3.1）
  - `bottom`（仅 3.1）
  - `left_front`（仅 3.1）
  - `right_front`（仅 3.1）

注意：

- **`front` 不在 `MultiViewImages` 的 `ViewType` 取值里**
- 所以 **前视图必须单独放在 `ImageBase64` 或 `ImageUrl`**
- `left`、`right`、`back` 再放到 `MultiViewImages`

也就是说，你的 4 视图映射关系应该是：

```json
{
  "ImageBase64": "front 图的 base64",
  "MultiViewImages": [
    {"ViewType": "left", "ViewImageBase64": "left 图的 base64"},
    {"ViewType": "right", "ViewImageBase64": "right 图的 base64"},
    {"ViewType": "back", "ViewImageBase64": "back 图的 base64"}
  ]
}
```

## 3. 推荐参数

如果你的目标是生成带纹理的正常模型，推荐：

- `Model = "3.1"`
- `GenerateType = "Normal"`
- `EnablePBR = false` 或按需开启
- `FaceCount = 100000` 到 `500000`

说明：

- `Normal`：生成带纹理模型
- `Geometry`：只生成白模
- `LowPoly`：3.1 不支持
- `Sketch`：是线稿/草图场景，不是你这里的主场景

## 4. 请求限制

根据官方文档，和你这个 4 视图需求直接相关的限制有：

- 单张主输入图 `ImageBase64`：
  - 单边分辨率 `128 ~ 5000`
  - base64 后大小建议不超过 `6 MB`
  - 支持 `jpg / png / jpeg / webp`
- `MultiViewImages`：
  - 每个视角最多 1 张
  - 所有多视角图片编码后总大小不超过 `8 MB`
  - 单边分辨率 `128 ~ 5000`
  - 支持 `jpg / png`
- API JSON POST 使用 TC3 签名时，整体请求包支持到 `10 MB`

因此建议：

- 输入图尽量控制在 `1024` 或 `1536` 级别
- 用 `png` 或质量合适的 `jpg`
- 4 张图加起来不要太大

## 5. 官方接口调用流程

按你给的这套官方云 API 文档，实现流程是：

1. 本地读取 `front / left / right / back`
2. 转成 base64
3. 调 `SubmitHunyuanTo3DProJob`
4. 拿到 `JobId`
5. 轮询 `QueryHunyuanTo3DProJob`
6. 等状态变成：
   - `DONE`：成功
   - `FAIL`：失败
7. 从 `ResultFile3Ds` 里拿结果文件 URL
8. 下载 `OBJ zip` 或 `GLB`

## 6. 鉴权方式说明

你给的文档是 **腾讯云 API 3.0（TC3-HMAC-SHA256）** 体系。

这意味着这里不是 OpenAI 兼容接口那种简单的 `API Key` 认证，而是：

- `SecretId`
- `SecretKey`
- `TC3-HMAC-SHA256` 签名

所以：

- 如果你要严格按这篇文档实现，就需要腾讯云 API 3.0 鉴权
- 如果你只想用 `API Key`，那应该走混元生3D的 OpenAI 兼容接口文档，而不是这篇云 API 文档

本文下面的代码，**完全按你给的官方云 API 文档来写**。

## 7. 最小可用请求体示例

```json
{
  "Model": "3.1",
  "GenerateType": "Normal",
  "FaceCount": 200000,
  "EnablePBR": false,
  "ImageBase64": "<front_base64>",
  "MultiViewImages": [
    {
      "ViewType": "left",
      "ViewImageBase64": "<left_base64>"
    },
    {
      "ViewType": "right",
      "ViewImageBase64": "<right_base64>"
    },
    {
      "ViewType": "back",
      "ViewImageBase64": "<back_base64>"
    }
  ]
}
```

## 8. Python 完整示例

下面这份代码直接用 Python 标准库实现：

- 读取 4 张图
- 按 TC3 签名请求
- 提交任务
- 轮询状态
- 下载结果

运行前先设置环境变量：

```bash
export TENCENTCLOUD_SECRET_ID="你的SecretId"
export TENCENTCLOUD_SECRET_KEY="你的SecretKey"
export TENCENTCLOUD_REGION="ap-guangzhou"
```

保存为 `hunyuan_4view_to_3d.py`：

```python
import base64
import hashlib
import hmac
import json
import os
import time
from datetime import datetime
from pathlib import Path
from urllib import error, request


SERVICE = "ai3d"
VERSION = "2025-05-13"
HOST = "ai3d.tencentcloudapi.com"
REGION = os.getenv("TENCENTCLOUD_REGION", "ap-guangzhou")

SECRET_ID = os.getenv("TENCENTCLOUD_SECRET_ID")
SECRET_KEY = os.getenv("TENCENTCLOUD_SECRET_KEY")

if not SECRET_ID or not SECRET_KEY:
    raise RuntimeError("Please set TENCENTCLOUD_SECRET_ID and TENCENTCLOUD_SECRET_KEY")


def read_image_base64(path: str | Path) -> str:
    path = Path(path)
    return base64.b64encode(path.read_bytes()).decode("utf-8")


def sha256_hex(data: bytes | str) -> str:
    if isinstance(data, str):
        data = data.encode("utf-8")
    return hashlib.sha256(data).hexdigest()


def sign(key: bytes, msg: str) -> bytes:
    return hmac.new(key, msg.encode("utf-8"), hashlib.sha256).digest()


def build_authorization(payload: bytes, timestamp: int) -> str:
    content_type = "application/json; charset=utf-8"
    date = datetime.utcfromtimestamp(timestamp).strftime("%Y-%m-%d")

    canonical_headers = f"content-type:{content_type}\nhost:{HOST}\n"
    signed_headers = "content-type;host"
    canonical_request = (
        "POST\n"
        "/\n"
        "\n"
        f"{canonical_headers}\n"
        f"{signed_headers}\n"
        f"{sha256_hex(payload)}"
    )

    credential_scope = f"{date}/{SERVICE}/tc3_request"
    string_to_sign = (
        "TC3-HMAC-SHA256\n"
        f"{timestamp}\n"
        f"{credential_scope}\n"
        f"{sha256_hex(canonical_request)}"
    )

    secret_date = sign(("TC3" + SECRET_KEY).encode("utf-8"), date)
    secret_service = hmac.new(secret_date, SERVICE.encode("utf-8"), hashlib.sha256).digest()
    secret_signing = hmac.new(secret_service, b"tc3_request", hashlib.sha256).digest()
    signature = hmac.new(secret_signing, string_to_sign.encode("utf-8"), hashlib.sha256).hexdigest()

    return (
        "TC3-HMAC-SHA256 "
        f"Credential={SECRET_ID}/{credential_scope}, "
        f"SignedHeaders={signed_headers}, "
        f"Signature={signature}"
    )


def tc3_post(action: str, params: dict) -> dict:
    endpoint = f"https://{HOST}"
    payload = json.dumps(params, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    timestamp = int(time.time())

    headers = {
        "Authorization": build_authorization(payload, timestamp),
        "Content-Type": "application/json; charset=utf-8",
        "Host": HOST,
        "X-TC-Action": action,
        "X-TC-Region": REGION,
        "X-TC-Timestamp": str(timestamp),
        "X-TC-Version": VERSION,
    }

    req = request.Request(endpoint, data=payload, headers=headers, method="POST")
    try:
        with request.urlopen(req, timeout=120) as resp:
            body = resp.read().decode("utf-8")
    except error.HTTPError as exc:
        raise RuntimeError(f"HTTP {exc.code}: {exc.read().decode('utf-8', errors='replace')}") from exc
    except error.URLError as exc:
        raise RuntimeError(f"Network error: {exc}") from exc

    data = json.loads(body)
    response = data.get("Response", {})
    if "Error" in response:
        raise RuntimeError(
            f"{response['Error'].get('Code')}: {response['Error'].get('Message')} "
            f"(RequestId={response.get('RequestId')})"
        )
    return response


def submit_4view_job(front_path, left_path, right_path, back_path) -> str:
    params = {
        "Model": "3.1",
        "GenerateType": "Normal",
        "FaceCount": 200000,
        "EnablePBR": False,
        # front 不能放进 MultiViewImages，必须单独作为主输入
        "ImageBase64": read_image_base64(front_path),
        "MultiViewImages": [
            {
                "ViewType": "left",
                "ViewImageBase64": read_image_base64(left_path),
            },
            {
                "ViewType": "right",
                "ViewImageBase64": read_image_base64(right_path),
            },
            {
                "ViewType": "back",
                "ViewImageBase64": read_image_base64(back_path),
            },
        ],
    }
    response = tc3_post("SubmitHunyuanTo3DProJob", params)
    return response["JobId"]


def query_job(job_id: str) -> dict:
    return tc3_post("QueryHunyuanTo3DProJob", {"JobId": job_id})


def wait_until_done(job_id: str, poll_interval: int = 5, timeout: int = 1800) -> dict:
    deadline = time.time() + timeout
    last_status = None

    while time.time() < deadline:
        result = query_job(job_id)
        status = result.get("Status")
        if status != last_status:
            print(f"job={job_id}, status={status}")
            last_status = status

        if status == "DONE":
            return result
        if status == "FAIL":
            raise RuntimeError(
                f"job failed: {result.get('ErrorCode')} {result.get('ErrorMessage')}"
            )
        time.sleep(poll_interval)

    raise TimeoutError(f"Job {job_id} timed out")


def download_file(url: str, output_path: str | Path):
    output_path = Path(output_path)
    req = request.Request(url, method="GET")
    with request.urlopen(req, timeout=300) as resp:
        output_path.write_bytes(resp.read())


def main():
    front = "front.png"
    left = "left.png"
    right = "right.png"
    back = "back.png"

    job_id = submit_4view_job(front, left, right, back)
    print("submitted job:", job_id)

    result = wait_until_done(job_id)
    print("done:", result.get("RequestId"))

    files = result.get("ResultFile3Ds", [])
    for item in files:
        file_type = item.get("Type")
        url = item.get("Url")
        preview = item.get("PreviewImageUrl")
        print("type=", file_type, "url=", url)

        if file_type == "OBJ":
            download_file(url, "result_obj.zip")
        elif file_type == "GLB":
            download_file(url, "result.glb")

        if preview:
            download_file(preview, "preview.png")


if __name__ == "__main__":
    main()
```

## 9. 如果你想只返回白模

把请求里的：

```json
"GenerateType": "Normal"
```

改成：

```json
"GenerateType": "Geometry"
```

这样生成的是不带纹理的白模。

## 10. 如果你想要带纹理模型

保持：

```json
"GenerateType": "Normal"
```

如果你还想开 PBR，可以再加：

```json
"EnablePBR": true
```

## 11. 关于输出格式

官方文档说明：

- 如果不设置 `ResultFormat`
- 默认会返回文件组，通常包括 `OBJ` 和 `GLB`

如果你想强制只要一种特殊格式，可以设置：

- `STL`
- `USDZ`
- `FBX`

但如果你的目标是：

- 下载 `OBJ + 纹理`
- 或下载 `GLB`

那通常**不要传 `ResultFormat`**，直接吃默认回包更方便。

## 12. 实现注意事项

### 12.1 前视图不要放错地方

错误写法：

```json
"MultiViewImages": [
  {"ViewType": "front", ...}
]
```

这个不符合官方数据结构定义。

正确写法：

- `front` -> `ImageBase64` 或 `ImageUrl`
- `left/right/back` -> `MultiViewImages`

### 12.2 图片总大小要控住

4 张图一起传时，最容易踩的是大小限制。

建议：

- 每张图先压到合适尺寸
- 不要直接上传超大 PNG
- 尽量保证总请求体低于 `10 MB`

### 12.3 轮询一定要做

`SubmitHunyuanTo3DProJob` 只会返回 `JobId`。  
真正的 3D 结果必须通过 `QueryHunyuanTo3DProJob` 查询。

### 12.4 默认并发只有 3

官方文档明确写了：

- 默认提供 `3` 个并发
- 最多同时处理 `3` 个已提交任务

如果你批量提交很多任务，超过的会排队。

## 13. 结论

针对“4 张前后左右视图 -> 调用官方 API -> 生成 3D 模型”，最标准的实现方式是：

1. `front` 作为主输入：`ImageBase64`
2. `left/right/back` 放入 `MultiViewImages`
3. 调 `SubmitHunyuanTo3DProJob`
4. 用 `JobId` 轮询 `QueryHunyuanTo3DProJob`
5. 从 `ResultFile3Ds` 下载 `OBJ zip` 或 `GLB`

这就是和你给的官方文档完全对齐的实现路径。

## 14. 官方文档来源

- 请求结构：https://cloud.tencent.com/document/product/1804/120831
- 提交混元生3D专业版任务：https://cloud.tencent.com/document/product/1804/123447
- 查询混元生3D专业版任务：https://cloud.tencent.com/document/product/1804/123448
- 数据结构（`ViewImage` / `File3D`）：https://cloud.tencent.com/document/product/1804/120828
