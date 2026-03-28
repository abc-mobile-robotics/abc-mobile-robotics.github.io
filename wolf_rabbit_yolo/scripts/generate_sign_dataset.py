import os
import math
import random
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageEnhance


# =========================
# 路徑設定
# =========================
ROOT = Path(r"C:\Users\brian\Desktop\school\mobile-robotics\wolf_rabbit_yolo")
ASSET_DIR = ROOT / "assets"
DATASET_DIR = ROOT / "dataset"

IMG_DIRS = {
    "train": DATASET_DIR / "images" / "train",
    "val": DATASET_DIR / "images" / "val",
    "test": DATASET_DIR / "images" / "test",
}
LBL_DIRS = {
    "train": DATASET_DIR / "labels" / "train",
    "val": DATASET_DIR / "labels" / "val",
    "test": DATASET_DIR / "labels" / "test",
}

for d in list(IMG_DIRS.values()) + list(LBL_DIRS.values()):
    d.mkdir(parents=True, exist_ok=True)


# =========================
# 類別設定
# =========================
CLASS_MAP = {
    "wolf_sign": 0,
    "rabbit_sign": 1,
}

ASSETS = {
    "wolf_sign": ASSET_DIR / "wolf_sign.png",
    "rabbit_sign": ASSET_DIR / "rabbit_sign.png",
}


# =========================
# 數量設定
# =========================
NUM_TRAIN = 2500
NUM_VAL = 300
NUM_TEST = 200

IMAGE_W = 640
IMAGE_H = 640

random.seed(42)
np.random.seed(42)


# =========================
# 背景生成
# =========================
def random_color():
    return tuple(np.random.randint(0, 256, size=3).tolist())


def generate_background(width=640, height=640):
    """
    產生不需要手拍的隨機背景：
    - 純色漸層
    - 矩形 / 圓形 / 線段
    - 輕微模糊與雜訊
    """
    bg = Image.new("RGB", (width, height), random_color())
    draw = ImageDraw.Draw(bg)

    # 畫一些隨機幾何形狀
    for _ in range(random.randint(8, 20)):
        shape_type = random.choice(["rect", "ellipse", "line"])
        color = random_color()

        x1 = random.randint(0, width - 1)
        y1 = random.randint(0, height - 1)
        x2 = random.randint(0, width - 1)
        y2 = random.randint(0, height - 1)

        x_min, x_max = sorted([x1, x2])
        y_min, y_max = sorted([y1, y2])

        if shape_type == "rect":
            draw.rectangle(
                [x_min, y_min, x_max, y_max],
                outline=color,
                width=random.randint(1, 5)
            )
        elif shape_type == "ellipse":
            draw.ellipse(
                [x_min, y_min, x_max, y_max],
                outline=color,
                width=random.randint(1, 5)
            )
        else:
            draw.line(
                [x1, y1, x2, y2],
                fill=color,
                width=random.randint(1, 4)
            )

    # 加一點模糊
    if random.random() < 0.7:
        bg = bg.filter(ImageFilter.GaussianBlur(radius=random.uniform(0.3, 1.5)))

    # 加亮度 / 對比變化
    bg = ImageEnhance.Brightness(bg).enhance(random.uniform(0.8, 1.2))
    bg = ImageEnhance.Contrast(bg).enhance(random.uniform(0.8, 1.2))

    # 加雜訊
    arr = np.array(bg).astype(np.float32)
    noise = np.random.normal(0, random.uniform(2, 10), arr.shape)
    arr = np.clip(arr + noise, 0, 255).astype(np.uint8)

    return Image.fromarray(arr)


# =========================
# 方柱視角近似：透視變換
# =========================
def perspective_warp_rgba(img_rgba: Image.Image):
    """
    模擬看到方柱某一面 / 斜視角
    """
    img = np.array(img_rgba)
    h, w = img.shape[:2]

    src = np.float32([
        [0, 0],
        [w - 1, 0],
        [w - 1, h - 1],
        [0, h - 1]
    ])

    # 隨機讓四角扭曲，模擬視角
    margin_x = w * random.uniform(0.05, 0.35)
    margin_y = h * random.uniform(0.02, 0.20)

    dst = np.float32([
        [random.uniform(0, margin_x), random.uniform(0, margin_y)],
        [w - 1 - random.uniform(0, margin_x), random.uniform(0, margin_y)],
        [w - 1 - random.uniform(0, margin_x), h - 1 - random.uniform(0, margin_y)],
        [random.uniform(0, margin_x), h - 1 - random.uniform(0, margin_y)],
    ])

    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(
        img,
        M,
        (w, h),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0, 0, 0, 0)
    )

    return Image.fromarray(warped)


def random_sign_transform(sign_path: Path):
    """
    載入 sign，做縮放、旋轉、透視、亮度變化、輕微模糊
    """
    img = Image.open(sign_path).convert("RGBA")

    # 縮放
    target_w = random.randint(50, 240)
    scale = target_w / img.size[0]
    target_h = max(20, int(img.size[1] * scale))
    img = img.resize((target_w, target_h), Image.Resampling.LANCZOS)

    # 旋轉
    angle = random.uniform(-20, 20)
    img = img.rotate(angle, expand=True, resample=Image.Resampling.BICUBIC)

    # 透視
    if random.random() < 0.85:
        img = perspective_warp_rgba(img)

    # 顏色 / 亮度
    rgb = img.convert("RGB")
    alpha = img.getchannel("A")

    rgb = ImageEnhance.Brightness(rgb).enhance(random.uniform(0.8, 1.2))
    rgb = ImageEnhance.Contrast(rgb).enhance(random.uniform(0.8, 1.25))
    rgb = ImageEnhance.Color(rgb).enhance(random.uniform(0.8, 1.15))

    img = rgb.convert("RGBA")
    img.putalpha(alpha)

    # 模糊
    if random.random() < 0.4:
        img = img.filter(ImageFilter.GaussianBlur(radius=random.uniform(0.2, 1.2)))

    return img


# =========================
# 貼圖與 bbox 計算
# =========================
def paste_rgba_with_bbox(background: Image.Image, overlay: Image.Image, x: int, y: int):
    """
    把 overlay 貼到 background 上，並回傳可見 alpha 區域 bbox
    """
    bg = background.copy()
    bg.paste(overlay, (x, y), overlay)

    alpha = np.array(overlay.getchannel("A"))
    ys, xs = np.where(alpha > 10)

    if len(xs) == 0 or len(ys) == 0:
        return bg, None

    x1 = x + int(xs.min())
    y1 = y + int(ys.min())
    x2 = x + int(xs.max())
    y2 = y + int(ys.max())

    # 裁到畫面內
    x1 = max(0, min(background.width - 1, x1))
    y1 = max(0, min(background.height - 1, y1))
    x2 = max(0, min(background.width - 1, x2))
    y2 = max(0, min(background.height - 1, y2))

    if x2 <= x1 or y2 <= y1:
        return bg, None

    return bg, (x1, y1, x2, y2)


def bbox_to_yolo(x1, y1, x2, y2, img_w, img_h):
    xc = ((x1 + x2) / 2.0) / img_w
    yc = ((y1 + y2) / 2.0) / img_h
    bw = (x2 - x1) / img_w
    bh = (y2 - y1) / img_h
    return xc, yc, bw, bh


# =========================
# 生成單張
# =========================
def generate_one(index: int, split: str):
    cls_name = random.choice(list(CLASS_MAP.keys()))
    cls_id = CLASS_MAP[cls_name]

    bg = generate_background(IMAGE_W, IMAGE_H)
    sign = random_sign_transform(ASSETS[cls_name])

    # 隨機位置（可部分出界，增加邊界情況）
    max_x = IMAGE_W - 10
    max_y = IMAGE_H - 10
    x = random.randint(-sign.width // 4, max_x - sign.width // 2)
    y = random.randint(-sign.height // 4, max_y - sign.height // 2)

    composed, bbox = paste_rgba_with_bbox(bg, sign, x, y)

    # 可能加入第二個同類 / 異類，增加干擾
    labels = []
    if bbox is not None:
        labels.append((cls_id, *bbox_to_yolo(*bbox, IMAGE_W, IMAGE_H)))

    if random.random() < 0.25:
        cls_name2 = random.choice(list(CLASS_MAP.keys()))
        cls_id2 = CLASS_MAP[cls_name2]
        sign2 = random_sign_transform(ASSETS[cls_name2])
        x2 = random.randint(-sign2.width // 4, max_x - sign2.width // 2)
        y2 = random.randint(-sign2.height // 4, max_y - sign2.height // 2)
        composed, bbox2 = paste_rgba_with_bbox(composed, sign2, x2, y2)
        if bbox2 is not None:
            labels.append((cls_id2, *bbox_to_yolo(*bbox2, IMAGE_W, IMAGE_H)))

    img_name = f"{split}_{index:05d}.jpg"
    lbl_name = f"{split}_{index:05d}.txt"

    composed = composed.convert("RGB")
    composed.save(IMG_DIRS[split] / img_name, quality=95)

    with open(LBL_DIRS[split] / lbl_name, "w", encoding="utf-8") as f:
        for row in labels:
            cls_id, xc, yc, bw, bh = row
            f.write(f"{cls_id} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}\n")


# =========================
# 主程式
# =========================
def main():
    for i in range(NUM_TRAIN):
        generate_one(i, "train")
    for i in range(NUM_VAL):
        generate_one(i, "val")
    for i in range(NUM_TEST):
        generate_one(i, "test")

    print("Synthetic dataset generation completed.")
    print(f"Train: {NUM_TRAIN}, Val: {NUM_VAL}, Test: {NUM_TEST}")


if __name__ == "__main__":
    main()