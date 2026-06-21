# -*- coding: utf-8 -*-
"""
独立测试脚本：验证"多行采样 + 直线拟合"的黄色边线检测效果。

用法：
    python test_yellow_edge.py <图片路径>

逻辑和 yellow_track_opencv.py 中的 _find_edge 保持一致，方便离线
用图片调参（HSV阈值、ROI范围、采样行数、离群点阈值等）。
"""

import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# 参数（与 yellow_track_opencv.py 中的默认值保持一致，可按需调整）
# ============================================================
H_MIN, H_MAX = 0, 35
S_MIN, S_MAX = 15, 78
V_MIN, V_MAX = 121, 220

ROI_TOP = 0.35
ROI_BOTTOM = 0.60

TARGET_X = 576.0

N_SAMPLES = 15
OUTLIER_THRESH_RATIO = 0.15
EVAL_OFFSET = 50


# ============================================================
# 边线检测：多行采样 + 直线拟合（与节点中 _find_edge 逻辑一致）
# ============================================================
def find_edge(mask, side, n_samples=N_SAMPLES, outlier_thresh_ratio=OUTLIER_THRESH_RATIO,
               eval_offset=EVAL_OFFSET):
    """
    在mask内多行采样查找黄色区域边界，并用直线拟合得到鲁棒的边界位置。

    side: 'right' 每行从最右往左扫，取第一个非零像素；
          'left'  每行从最左往右扫，取第一个非零像素。

    eval_offset: 取拟合直线在"ROI底部往上数eval_offset像素"那一行的值
                  作为最终边界位置（避免ROI最底部几行因贴图像边缘饱和导致外推失真）。

    返回:
        edge_x:   最终边界x坐标（ROI内 y=h-1-eval_offset 处对应位置），找不到返回 None
        sample_pts: 实际采样到的边界点列表 [(x, y), ...]，y为ROI内的行号，用于可视化
        fit_params: (a, b) 拟合直线 x = a*y + b，点数不足时为 None
        eval_y:   实际用于求值的ROI内行号（用于可视化标记），点数不足时为 None
    """
    h, w = mask.shape
    rows = np.linspace(0, h - 1, n_samples).astype(int)

    sample_pts = []
    for y in rows:
        row = mask[y, :]
        nz = np.nonzero(row)[0]
        if nz.size == 0:
            continue
        x = nz[-1] if side == 'right' else nz[0]
        sample_pts.append((int(x), int(y)))

    if not sample_pts:
        return None, [], None, None

    xs = np.array([p[0] for p in sample_pts], dtype=np.float64)
    ys = np.array([p[1] for p in sample_pts], dtype=np.float64)

    if len(xs) < 3:
        return float(np.median(xs)), sample_pts, None, None

    # 用中位数剔除离群点
    median_x = np.median(xs)
    inliers = np.abs(xs - median_x) < (w * outlier_thresh_ratio)
    if np.count_nonzero(inliers) < 3:
        return float(median_x), sample_pts, None, None

    # 一次线性拟合 x = a*y + b
    a, b = np.polyfit(ys[inliers], xs[inliers], 1)

    # 用ROI底部往上偏移eval_offset像素那一行的拟合值作为边界位置
    eval_y = max(h - 1 - eval_offset, 0)
    edge_x = a * eval_y + b
    edge_x = float(np.clip(edge_x, 0, w - 1))
    return edge_x, sample_pts, (a, b), eval_y


# ============================================================
# 图像处理：HSV分割 + ROI裁剪
# ============================================================
def process_image(image_path):
    bgr = cv2.imread(image_path)
    if bgr is None:
        raise FileNotFoundError(f"无法读取图像: {image_path}")

    height, width = bgr.shape[:2]

    top = int(height * ROI_TOP)
    bottom = int(height * ROI_BOTTOM)
    roi = bgr[top:bottom, :]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower = np.array([H_MIN, S_MIN, V_MIN])
    upper = np.array([H_MAX, S_MAX, V_MAX])
    mask = cv2.inRange(hsv, lower, upper)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    right_x, right_pts, right_fit, right_eval_y = find_edge(mask, 'right')
    left_x, left_pts, left_fit, left_eval_y = find_edge(mask, 'left')

    vis = draw_visualization(
        bgr, mask, top, bottom, width,
        right_x, right_pts, right_fit, right_eval_y,
        left_x, left_pts, left_fit, left_eval_y,
    )
    return vis, mask, left_x, right_x


# ============================================================
# 可视化：在原图上画ROI框、mask叠加、采样点、拟合线、边界结果
# ============================================================
def draw_visualization(bgr, mask, top, bottom, width,
                        right_x, right_pts, right_fit, right_eval_y,
                        left_x, left_pts, left_fit, left_eval_y):
    vis = bgr.copy()
    h_roi = bottom - top

    # ROI框（蓝色）
    cv2.rectangle(vis, (0, top), (width - 1, bottom - 1), (255, 0, 0), 2)

    # mask叠加（绿色半透明）
    mask_color = np.zeros((h_roi, width, 3), dtype=np.uint8)
    mask_color[mask > 0] = (0, 200, 0)
    vis[top:bottom, :] = cv2.addWeighted(vis[top:bottom, :], 0.7, mask_color, 0.3, 0)

    # 目标位置参考线（红色）
    tx = int(np.clip(TARGET_X, 0, width - 1))
    cv2.line(vis, (tx, top), (tx, bottom), (0, 0, 255), 1)

    # 取值行参考线（白色虚线效果，标出 y = h_roi-1-EVAL_OFFSET 这一行）
    eval_row = max(h_roi - 1 - EVAL_OFFSET, 0)
    cv2.line(vis, (0, top + eval_row), (width - 1, top + eval_row), (255, 255, 255), 1)

    # ---- 右边界：采样点(黄)、拟合线(青)、最终结果(绿) ----
    for (x, y) in right_pts:
        cv2.circle(vis, (x, top + y), 3, (0, 255, 255), -1)
    if right_fit is not None:
        a, b = right_fit
        y0, y1 = 0, h_roi - 1
        x0, x1 = a * y0 + b, a * y1 + b
        cv2.line(vis, (int(x0), top + y0), (int(x1), top + y1), (255, 255, 0), 2)
    if right_x is not None:
        rx = int(right_x)
        ry = top + (right_eval_y if right_eval_y is not None else h_roi - 1)
        cv2.circle(vis, (rx, ry), 6, (0, 255, 0), -1)
        cv2.putText(vis, f'R={right_x:.1f}', (rx + 8, ry),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # ---- 左边界：采样点(品红)、拟合线(紫)、最终结果(绿) ----
    for (x, y) in left_pts:
        cv2.circle(vis, (x, top + y), 3, (255, 0, 255), -1)
    if left_fit is not None:
        a, b = left_fit
        y0, y1 = 0, h_roi - 1
        x0, x1 = a * y0 + b, a * y1 + b
        cv2.line(vis, (int(x0), top + y0), (int(x1), top + y1), (255, 0, 200), 2)
    if left_x is not None:
        lx = int(left_x)
        ly = top + (left_eval_y if left_eval_y is not None else h_roi - 1)
        cv2.circle(vis, (lx, ly), 6, (0, 255, 0), -1)
        cv2.putText(vis, f'L={left_x:.1f}', (lx + 8, ly),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return vis


# ============================================================
# 主函数
# ============================================================
def main():
    if len(sys.argv) < 2:
        print("用法: python test_yellow_edge.py <图片路径>")
        sys.exit(1)

    image_path = sys.argv[1]
    vis, mask, left_x, right_x = process_image(image_path)

    print(f"left_x  = {left_x}")
    print(f"right_x = {right_x}")

    vis_rgb = cv2.cvtColor(vis, cv2.COLOR_BGR2RGB)

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    axes[0].imshow(vis_rgb)
    axes[0].set_title("Visualization (ROI / sample points / fit line / edge result)")
    axes[0].axis('off')

    axes[1].imshow(mask, cmap='gray')
    axes[1].set_title("Yellow Mask (ROI only)")
    axes[1].axis('off')

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
