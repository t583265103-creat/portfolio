"""
visualize_pipeline.py
---------------------
קורא את קבצי ה-PCD שנשמרו ע"י LidarCloud::enablePipelineCapture()
ויוצר סרטון קצר (MP4) המציג את שלבי עיבוד ענן הנקודות.

שימוש:
    python visualize_pipeline.py [--capture_dir PATH] [--out OUTPUT.mp4] [--fps N]

דרישות:
    pip install numpy matplotlib imageio[ffmpeg]
"""

import argparse
import os
import re
import glob
import numpy as np
import matplotlib
matplotlib.use("Agg")               # ללא חלון GUI — לשמירה לקובץ
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D   # noqa: F401
import imageio


# ───────────────────────────────────────────────────────────────────────────────
# PCD READER  (ASCII format, PointXYZI)
# ───────────────────────────────────────────────────────────────────────────────

def read_pcd(path: str) -> np.ndarray:
    """מחזיר מערך N×4 [x, y, z, intensity] או None."""
    if not os.path.isfile(path):
        return None
    points = []
    in_data = False
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if line.upper() == "DATA ASCII":
                in_data = True
                continue
            if in_data and line:
                parts = line.split()
                if len(parts) >= 4:
                    try:
                        points.append([float(p) for p in parts[:4]])
                    except ValueError:
                        pass
    return np.array(points, dtype=np.float32) if points else None


def read_params(path: str) -> dict:
    """קורא key=value מ-pipeline_params.txt."""
    params = {}
    if not os.path.isfile(path):
        return params
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if "=" in line:
                k, _, v = line.partition("=")
                params[k.strip()] = v.strip()
    return params


# ───────────────────────────────────────────────────────────────────────────────
# FRAME RENDERING
# ───────────────────────────────────────────────────────────────────────────────

COLORS = [
    "#e6194b","#3cb44b","#ffe119","#4363d8","#f58231",
    "#911eb4","#42d4f4","#f032e6","#bfef45","#fabed4",
]

def _scatter3(ax, pts, color, size=1.5, alpha=0.6, label=None):
    if pts is None or len(pts) == 0:
        return
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2],
               c=color, s=size, alpha=alpha, linewidths=0, label=label)


def _style_ax(ax, title, subtitle=""):
    ax.set_title(title, fontsize=11, fontweight="bold", pad=4, color="white")
    if subtitle:
        ax.text2D(0.5, -0.04, subtitle, transform=ax.transAxes,
                  ha="center", fontsize=7, color="#aaaaaa", wrap=True)
    ax.set_facecolor("#1a1a2e")
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.tick_params(colors="#555555", labelsize=6)
    ax.set_xlabel("X [m]", fontsize=6, color="#666666", labelpad=2)
    ax.set_ylabel("Y [m]", fontsize=6, color="#666666", labelpad=2)
    ax.set_zlabel("Z [m]", fontsize=6, color="#666666", labelpad=2)


def render_frame(step_idx: int, step_data: list, params: dict,
                 w_px=1920, h_px=1080, dpi=120) -> np.ndarray:
    """
    מרנדר פריים אחד לתמונת numpy RGB.
    step_data: רשימה של dict {pts, color, label} לציור.
    """
    fig = plt.figure(figsize=(w_px / dpi, h_px / dpi), dpi=dpi,
                     facecolor="#0d0d1a")

    # פאנל 3D עיקרי (שמאל)
    ax = fig.add_axes([0.02, 0.08, 0.60, 0.85], projection="3d")
    ax.set_facecolor("#1a1a2e")

    for item in step_data:
        pts   = item["pts"]
        color = item["color"]
        label = item.get("label", None)
        if pts is not None and len(pts):
            _scatter3(ax, pts, color, size=2.0, alpha=0.75, label=label)

    _style_ax(ax, step_data[0]["title"] if step_data else "")

    if any(d.get("label") for d in step_data):
        ax.legend(loc="upper left", fontsize=7, framealpha=0.3,
                  labelcolor="white", markerscale=3)

    # פאנל טקסט (ימין) — פרמטרים
    tax = fig.add_axes([0.64, 0.05, 0.34, 0.90])
    tax.axis("off")
    tax.set_facecolor("#0d0d1a")

    info_lines = _build_info_text(step_idx, params)
    y = 0.97
    for line in info_lines:
        color = "#00e5ff" if line.startswith("##") else (
                "#ffffff" if line.startswith("#") else "#cccccc")
        text  = line.lstrip("#").strip()
        size  = 13 if line.startswith("##") else (10 if line.startswith("#") else 8.5)
        tax.text(0.02, y, text, transform=tax.transAxes,
                 fontsize=size, color=color, fontfamily="monospace",
                 va="top", wrap=True)
        y -= 0.038 if size > 9 else 0.030

    fig.canvas.draw()
    buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
    buf = buf.reshape(fig.canvas.get_width_height()[::-1] + (4,))
    buf = buf[:, :, :3]   # RGBA -> RGB
    plt.close(fig)
    return buf


def _build_info_text(step_idx: int, p: dict) -> list:
    """בונה רשימת שורות טקסט לתצוגת הפרמטרים."""
    steps_info = [
        # שלב 0 — raw merged
        [
            "## 1. מיזוג עננים גולמיים",
            "# Pipeline Step 1/5",
            "",
            "# תיאור:",
            f"  נקודות ממוזגות: {p.get('step1_merged_pts','?')}",
            f"  חיישן 0:  {p.get('step1_sensor0_pts','?')} נקודות",
            f"  חיישן 1:  {p.get('step1_sensor1_pts','?')} נקודות",
            f"  חיישן 2:  {p.get('step1_sensor2_pts','?')} נקודות",
            "",
            "# תיאור:",
            "  שלושה לידארים (קדמי, שמאלי, ימני)",
            "  עוברים כיול בסיסי (Extrinsics) ו-Voxel",
            "  Grid לפני המיזוג.",
        ],
        # שלב 1 — cropbox
        [
            "## 2. CropBox + PassThrough",
            "# Pipeline Step 2/5",
            "",
            f"  נקודות לפני: {p.get('step1_merged_pts','?')}",
            f"  נקודות אחרי: {p.get('step2_cropbox_pts','?')}",
            "",
            "# פרמטרים:",
            f"  ROI min XYZ: {p.get('step2_roi_min','?')}",
            f"  ROI max XYZ: {p.get('step2_roi_max','?')}",
            f"  min intensity: {p.get('step2_min_intensity','?')}",
            "",
            "# תיאור:",
            "  CropBox חותך לתחום מרחבי [m]",
            "  PassThrough מסנן לפי intensity",
        ],
        # שלב 2 — voxel
        [
            "## 3. Voxel Grid Downsample",
            "# Pipeline Step 3/5",
            "",
            f"  נקודות לפני: {p.get('step2_cropbox_pts','?')}",
            f"  נקודות אחרי: {p.get('step3_voxel_pts','?')}",
            "",
            "# פרמטרים:",
            f"  voxel leaf size: {p.get('step3_voxel_size','?')} m",
            "",
            "# תיאור:",
            "  ApproximateVoxelGrid מקטין צפיפות",
            "  ומאפשר עיבוד מהיר בזמן אמת.",
        ],
        # שלב 3 — outliers
        [
            "## 4. Statistical Outlier Removal",
            "# Pipeline Step 4/5",
            "",
            f"  נקודות לפני: {p.get('step3_voxel_pts','?')}",
            f"  נקודות אחרי: {p.get('step4_outlier_pts','?')}",
            f"  נקודות הוסרו: "
                f"{int(p.get('step3_voxel_pts',0) or 0) - int(p.get('step4_outlier_pts',0) or 0)}",
            "",
            "# פרמטרים:",
            f"  meanK (min neighbors): {p.get('step4_min_neighbors','?')}",
            f"  stddev threshold:  {p.get('step4_std_thresh','?')}",
            "",
            "# תיאור:",
            "  מסיר נקודות 'מרחפות' בודדות",
            "  שאינן חלק מאובייקטים אמיתיים.",
        ],
        # שלב 4 — clustering
        [
            "## 5. Euclidean Clustering",
            "# Pipeline Step 5/5",
            "",
            f"  אשכולות שנמצאו: {p.get('step5_num_clusters','?')}",
            "",
            "# פרמטרים:",
            f"  cluster tolerance: {p.get('step5_cluster_tolerance','?')} m",
            f"  min cluster size:  {p.get('step5_min_cluster','?')} נקודות",
            f"  max cluster size:  {p.get('step5_max_cluster','?')} נקודות",
            "",
            "# אשכולות:",
        ] + [
            f"  #{c}: {p.get(f'step5_cluster_{c}_pts','?')} נקודות"
            for c in range(int(p.get("step5_num_clusters", 0) or 0))
        ],
    ]
    if step_idx < len(steps_info):
        return steps_info[step_idx]
    return []


# ───────────────────────────────────────────────────────────────────────────────
# MAIN
# ───────────────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Lidar pipeline visualizer")
    ap.add_argument("--capture_dir", default="pipeline_capture",
                    help="ספריית הקבצים ש-LidarCloud שמר")
    ap.add_argument("--out", default="pipeline_video.mp4",
                    help="נתיב לקובץ הוידאו שיצא")
    ap.add_argument("--fps", type=int, default=1,
                    help="FPS (1 = שנייה לשלב)")
    ap.add_argument("--hold", type=int, default=3,
                    help="כמה שניות להחזיק כל שלב")
    args = ap.parse_args()

    cap_dir   = args.capture_dir
    params    = read_params(os.path.join(cap_dir, "pipeline_params.txt"))

    # טעינת כל קבצי PCD
    step1 = read_pcd(os.path.join(cap_dir, "step1_merged.pcd"))
    step2 = read_pcd(os.path.join(cap_dir, "step2_cropbox.pcd"))
    step3 = read_pcd(os.path.join(cap_dir, "step3_voxel.pcd"))
    step4 = read_pcd(os.path.join(cap_dir, "step4_outliers.pcd"))

    n_clusters = int(params.get("step5_num_clusters", 0) or 0)
    clusters   = []
    for c in range(n_clusters):
        pts = read_pcd(os.path.join(cap_dir, f"step5_cluster_{c}.pcd"))
        clusters.append(pts)

    # ─── הגדרת שלבים ───────────────────────────────────────────────────────
    stages = [
        # שלב 1
        [{"pts": step1, "color": "#00bcd4", "title": "שלב 1 — ענן גולמי ממוזג"}],

        # שלב 2
        [{"pts": step2, "color": "#26c6da", "title": "שלב 2 — לאחר CropBox"}],

        # שלב 3
        [{"pts": step3, "color": "#66bb6a", "title": "שלב 3 — לאחר Voxel Grid"}],

        # שלב 4
        [{"pts": step4, "color": "#ffa726", "title": "שלב 4 — לאחר Outlier Removal"}],

        # שלב 5 — כל אשכול בצבע שונה
        [{"pts": clusters[c],
          "color": COLORS[c % len(COLORS)],
          "title": f"שלב 5 — חלוקה ל-{n_clusters} אובייקטים",
          "label": f"אובייקט #{c}  ({len(clusters[c]) if clusters[c] is not None else 0} נק')" }
         for c in range(n_clusters)] or
        [{"pts": None, "color": "white", "title": "שלב 5 — אין אשכולות"}],
    ]

    frames = []
    for step_idx, step_data in enumerate(stages):
        print(f"  rendering step {step_idx + 1}/{len(stages)} ...")
        img = render_frame(step_idx, step_data, params)
        # כפול hold פריימים
        for _ in range(args.hold * args.fps):
            frames.append(img)

    if not frames:
        print("[ERROR] אין פריימים לכתוב.")
        return

    print(f"  writing {args.out}  ({len(frames)} frames @ {args.fps} fps)...")
    writer = imageio.get_writer(args.out, fps=args.fps, codec="libx264",
                                quality=8, pixelformat="yuv420p")
    for f in frames:
        writer.append_data(f)
    writer.close()
    print(f"[DONE] {args.out}")


if __name__ == "__main__":
    main()
