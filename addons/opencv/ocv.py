#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Shared OpenCV-side photo -> measurement -> CSV collector.

Primary use:
- read still images
- extract 2D chain points / angles
- keep rows in memory
- export rows to CSV

func_finder is expected to consume CSV, not raw images.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np

EXPECTED_RED_CHAIN_POINTS = 11


def calculate_angle_between_vectors(v1: tuple[float, float], v2: tuple[float, float]) -> float:
    dot_product = float(v1[0] * v2[0] + v1[1] * v2[1])
    determinant = float(v1[0] * v2[1] - v1[1] * v2[0])
    return float(math.degrees(math.atan2(determinant, dot_product)))


@dataclass(frozen=True)
class Measurement2D:
    image_path: str
    roll_deg: float
    theta1_deg: float
    theta2_deg: float
    start_red_xy: tuple[int, int]
    yellow_anchor_xy: tuple[int, int]
    joint_xy: np.ndarray
    segment_angles_deg: np.ndarray


def _normalized_joint_xy(joint_xy: np.ndarray) -> np.ndarray:
    points = np.asarray(joint_xy, dtype=float)
    if len(points) == 0:
        return points.copy()
    out = points - points[0]
    out[:, 1] *= -1.0
    return out


def _compute_segment_angles_deg(joint_xy: np.ndarray) -> np.ndarray:
    points = np.asarray(joint_xy, dtype=float)
    if len(points) < 3:
        return np.zeros((0,), dtype=float)
    angles: list[float] = []
    prev_vector = (
        float(points[1][0] - points[0][0]),
        float(points[1][1] - points[0][1]),
    )
    for idx in range(1, len(points) - 1):
        pt1 = points[idx]
        pt2 = points[idx + 1]
        current_vector = (float(pt2[0] - pt1[0]), float(pt2[1] - pt1[1]))
        angles.append(calculate_angle_between_vectors(prev_vector, current_vector))
        prev_vector = current_vector
    return np.array(angles, dtype=float)


def _render_measurement_preview(frame: np.ndarray, measurement: Measurement2D) -> np.ndarray:
    result_img = frame.copy()
    yellow_anchor = tuple(int(v) for v in measurement.yellow_anchor_xy)
    ordered_points = [tuple(int(v) for v in pt) for pt in np.asarray(measurement.joint_xy, dtype=float)]
    for idx, pt in enumerate(ordered_points):
        if idx == 0:
            color = (0, 255, 255)
            label = "Y0"
            radius = 8
        elif idx == len(ordered_points) - 1:
            color = (255, 0, 255)
            label = "End"
            radius = 6
        else:
            color = (0, 0, 255)
            label = f"R{idx}"
            radius = 6
        cv2.circle(result_img, pt, radius, color, -1)
        cv2.putText(result_img, label, (pt[0] + 12, pt[1] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    if len(ordered_points) >= 2:
        cv2.line(result_img, ordered_points[0], ordered_points[1], (0, 0, 255), 3)
    for idx in range(1, len(ordered_points) - 1):
        pt1 = ordered_points[idx]
        pt2 = ordered_points[idx + 1]
        line_color = (200, 100, 255) if idx == len(ordered_points) - 2 else (0, 255, 0)
        cv2.line(result_img, pt1, pt2, line_color, 2)
        angle = float(measurement.segment_angles_deg[idx - 1])
        cv2.putText(
            result_img,
            f"A{idx}: {angle:.1f}",
            (pt1[0] + 15, pt1[1] + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )

    cv2.putText(result_img, "[ yellow -> nearest left red = start ]", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    return result_img


def _with_corrected_point(measurement: Measurement2D, point_index: int, xy: tuple[int, int]) -> Measurement2D:
    points = np.asarray(measurement.joint_xy, dtype=float).copy()
    points[point_index] = np.array([float(xy[0]), float(xy[1])], dtype=float)
    angles = _compute_segment_angles_deg(points)
    start_red_xy = measurement.start_red_xy
    yellow_anchor_xy = measurement.yellow_anchor_xy
    if point_index == 0:
        yellow_anchor_xy = (int(xy[0]), int(xy[1]))
    if point_index == 1:
        start_red_xy = (int(xy[0]), int(xy[1]))
    return Measurement2D(
        image_path=measurement.image_path,
        roll_deg=measurement.roll_deg,
        theta1_deg=measurement.theta1_deg,
        theta2_deg=measurement.theta2_deg,
        start_red_xy=start_red_xy,
        yellow_anchor_xy=yellow_anchor_xy,
        joint_xy=points,
        segment_angles_deg=angles,
    )


def _get_contour_center(mask: np.ndarray) -> Optional[tuple[int, int]]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest)
    if float(M["m00"]) == 0.0:
        return None
    return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))


def _collect_red_points(mask: np.ndarray) -> list[tuple[int, int]]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    points: list[tuple[int, int]] = []
    for cnt in contours:
        area = float(cv2.contourArea(cnt))
        if area < 120.0 or area > 2500.0:
            continue
        perimeter = float(cv2.arcLength(cnt, True))
        if perimeter <= 0.0:
            continue
        circularity = float(4.0 * math.pi * area / (perimeter * perimeter))
        if circularity < 0.55:
            continue
        M = cv2.moments(cnt)
        if float(M["m00"]) == 0.0:
            continue
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        points.append(center)
    return points


def _choose_start_red(
    red_points: list[tuple[int, int]],
    yellow_anchor: tuple[int, int],
) -> tuple[int, int]:
    left_side = [pt for pt in red_points if pt[0] < yellow_anchor[0]]
    candidates = left_side if left_side else red_points
    if not candidates:
        raise ValueError("failed to detect red chain points")
    return min(
        candidates,
        key=lambda pt: math.hypot(float(pt[0] - yellow_anchor[0]), float(pt[1] - yellow_anchor[1])),
    )


def _order_chain_points(
    red_points: list[tuple[int, int]],
    start_red: tuple[int, int],
    yellow_anchor: tuple[int, int],
    *,
    expected_count: int = EXPECTED_RED_CHAIN_POINTS,
) -> list[tuple[int, int]]:
    ordered: list[tuple[int, int]] = [start_red]
    remaining = [pt for pt in red_points if pt != start_red]
    current = start_red
    prev_dir = np.array(
        [float(start_red[0] - yellow_anchor[0]), float(start_red[1] - yellow_anchor[1])],
        dtype=float,
    )
    while remaining and len(ordered) < expected_count:
        if float(np.linalg.norm(prev_dir)) < 1e-6:
            prev_dir = np.array([1.0, 0.0], dtype=float)

        def _score(pt: tuple[int, int]) -> float:
            step = np.array([float(pt[0] - current[0]), float(pt[1] - current[1])], dtype=float)
            dist = float(np.linalg.norm(step))
            if dist < 1e-6:
                return 1e12
            step_dir = step / dist
            prev_unit = prev_dir / float(np.linalg.norm(prev_dir))
            align = float(np.dot(step_dir, prev_unit))
            # Prefer continuation along the same direction and strongly reject reversals.
            penalty = 1.0 - align
            if align < 0.0:
                penalty += 2.0
            return dist * (1.0 + penalty)

        closest = min(remaining, key=_score)
        ordered.append(closest)
        remaining.remove(closest)
        prev_dir = np.array([float(closest[0] - current[0]), float(closest[1] - current[1])], dtype=float)
        current = closest
    if len(ordered) < expected_count:
        raise ValueError(
            f"failed to trace enough red chain points: expected {expected_count}, got {len(ordered)}"
        )
    return ordered


def extract_from_frame(
    frame: np.ndarray,
    *,
    image_path: str,
    roll_deg: float,
    theta1_deg: float,
    theta2_deg: float,
) -> tuple[Measurement2D, np.ndarray]:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask_red1 = cv2.inRange(hsv, np.array([0, 90, 70]), np.array([12, 255, 255]))
    mask_red2 = cv2.inRange(hsv, np.array([168, 90, 70]), np.array([180, 255, 255]))
    red_mask = cv2.bitwise_or(mask_red1, mask_red2)
    yellow_mask = cv2.inRange(hsv, np.array([15, 100, 100]), np.array([45, 255, 255]))

    kernel = np.ones((3, 3), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
    yellow_anchor = _get_contour_center(yellow_mask)
    red_points = _collect_red_points(red_mask)
    if yellow_anchor is None:
        raise ValueError("failed to detect yellow anchor point")
    if len(red_points) < EXPECTED_RED_CHAIN_POINTS:
        raise ValueError(
            f"failed to detect enough red chain points: expected at least {EXPECTED_RED_CHAIN_POINTS}, got {len(red_points)}"
        )

    start_red = _choose_start_red(red_points, yellow_anchor)
    ordered_red_points = _order_chain_points(red_points, start_red, yellow_anchor)
    ordered_points = [yellow_anchor, *ordered_red_points]

    measurement = Measurement2D(
        image_path=str(image_path),
        roll_deg=float(roll_deg),
        theta1_deg=float(theta1_deg),
        theta2_deg=float(theta2_deg),
        start_red_xy=(int(start_red[0]), int(start_red[1])),
        yellow_anchor_xy=(int(yellow_anchor[0]), int(yellow_anchor[1])),
        joint_xy=np.array(ordered_points, dtype=float),
        segment_angles_deg=_compute_segment_angles_deg(np.array(ordered_points, dtype=float)),
    )
    return measurement, _render_measurement_preview(frame, measurement)


def extract_from_image(
    image_path: str,
    *,
    roll_deg: float,
    theta1_deg: float,
    theta2_deg: float,
) -> tuple[Measurement2D, np.ndarray]:
    img = cv2.imread(str(image_path))
    if img is None:
        raise FileNotFoundError(f"failed to read image: {image_path}")
    return extract_from_frame(
        img,
        image_path=str(image_path),
        roll_deg=roll_deg,
        theta1_deg=theta1_deg,
        theta2_deg=theta2_deg,
    )


def measurement_to_row(measurement: Measurement2D, *, row_no: int) -> dict[str, Any]:
    normalized = _normalized_joint_xy(measurement.joint_xy)
    row: dict[str, Any] = {
        "number": int(row_no),
        "roll_deg": float(measurement.roll_deg),
        "theta1_deg": float(measurement.theta1_deg),
        "theta2_deg": float(measurement.theta2_deg),
    }
    for i, (x, y) in enumerate(normalized, start=1):
        row[f"{i}_x"] = float(x)
        row[f"{i}_y"] = float(y)
    for i, angle in enumerate(np.asarray(measurement.segment_angles_deg, dtype=float)):
        row[f"angle_deg_{i}"] = float(-angle)
    return row


class SampleCollector:
    def __init__(self) -> None:
        self.rows: dict[int, dict[str, Any]] = {}

    def clear(self) -> None:
        self.rows.clear()

    def put_row(self, row_no: int, measurement: Measurement2D) -> dict[str, Any]:
        row = measurement_to_row(measurement, row_no=row_no)
        self.rows[int(row_no)] = row
        return row

    def export_csv(self, out_path: str) -> None:
        if not self.rows:
            raise RuntimeError("no rows collected")
        out = Path(out_path)
        out.parent.mkdir(parents=True, exist_ok=True)
        ordered_rows = [self.rows[k] for k in sorted(self.rows.keys())]
        fieldnames = list(ordered_rows[0].keys())
        with out.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(ordered_rows)


class CsvGui:
    def __init__(self) -> None:
        import tkinter as tk

        self._tk = tk
        self.collector = SampleCollector()
        self.root = tk.Tk()
        self.root.title("ocv csv tool")
        self._preview_window_name = "ocv preview"

        self.image_path_var = tk.StringVar()
        self.roll_var = tk.StringVar(value="0")
        self.theta1_var = tk.StringVar(value="0")
        self.theta2_var = tk.StringVar(value="0")
        self.row_no_var = tk.StringVar(value="1")
        self.correct_index_var = tk.StringVar(value="0")
        self.status_var = tk.StringVar(value="ready")
        self._current_measurement: Optional[Measurement2D] = None
        self._current_frame: Optional[np.ndarray] = None
        self._current_row_no: Optional[int] = None
        self._pending_correct_index: Optional[int] = None
        self._pending_click_xy: Optional[tuple[int, int]] = None

        self._build()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(30, self._pump_cv_events)

    def _build(self) -> None:
        tk = self._tk
        pad = {"padx": 6, "pady": 4}

        tk.Label(self.root, text="image").grid(row=0, column=0, sticky="w", **pad)
        tk.Entry(self.root, textvariable=self.image_path_var, width=60).grid(row=0, column=1, sticky="we", **pad)
        tk.Button(self.root, text="Browse", command=self._browse_image).grid(row=0, column=2, **pad)

        tk.Label(self.root, text="roll_deg").grid(row=1, column=0, sticky="w", **pad)
        tk.Entry(self.root, textvariable=self.roll_var, width=12).grid(row=1, column=1, sticky="w", **pad)

        tk.Label(self.root, text="theta1_deg").grid(row=2, column=0, sticky="w", **pad)
        tk.Entry(self.root, textvariable=self.theta1_var, width=12).grid(row=2, column=1, sticky="w", **pad)

        tk.Label(self.root, text="theta2_deg").grid(row=3, column=0, sticky="w", **pad)
        tk.Entry(self.root, textvariable=self.theta2_var, width=12).grid(row=3, column=1, sticky="w", **pad)

        tk.Label(self.root, text="row_no").grid(row=4, column=0, sticky="w", **pad)
        tk.Entry(self.root, textvariable=self.row_no_var, width=12).grid(row=4, column=1, sticky="w", **pad)

        tk.Button(self.root, text="Load Image", command=self._load_image).grid(row=5, column=0, sticky="we", **pad)
        tk.Button(self.root, text="Put Row", command=self._put_row).grid(row=5, column=1, sticky="we", **pad)
        tk.Button(self.root, text="Clear", command=self._clear_rows).grid(row=5, column=2, sticky="we", **pad)

        tk.Button(self.root, text="Export CSV", command=self._export_csv).grid(row=6, column=0, sticky="we", **pad)

        tk.Label(self.root, text="correct_index").grid(row=7, column=0, sticky="w", **pad)
        tk.Entry(self.root, textvariable=self.correct_index_var, width=12).grid(row=7, column=1, sticky="w", **pad)
        tk.Button(self.root, text="Correct Point", command=self._start_correct_point).grid(row=7, column=2, sticky="we", **pad)

        self.rows_label = tk.Label(self.root, text="rows: 0")
        self.rows_label.grid(row=8, column=0, sticky="w", **pad)

        tk.Label(self.root, textvariable=self.status_var, anchor="w", justify="left").grid(row=9, column=0, columnspan=3, sticky="we", **pad)

        self.root.grid_columnconfigure(1, weight=1)

    def _browse_image(self) -> None:
        from tkinter import filedialog

        path = filedialog.askopenfilename(
            title="Select image",
            filetypes=[("Image files", "*.png *.jpg *.jpeg *.bmp"), ("All files", "*.*")],
        )
        if path:
            self.image_path_var.set(path)
            self._maybe_fill_angles_from_filename(path)

    def _maybe_fill_angles_from_filename(self, image_path: str) -> None:
        stem = Path(image_path).stem
        parts = [p.strip() for p in stem.split("-")]
        if len(parts) != 3:
            return
        try:
            roll_deg = float(parts[0])
            theta1_deg = float(parts[1])
            theta2_deg = float(parts[2])
        except Exception:
            return
        self.roll_var.set(str(roll_deg))
        self.theta1_var.set(str(theta1_deg))
        self.theta2_var.set(str(theta2_deg))

    def _show_preview(self, img: np.ndarray) -> None:
        cv2.namedWindow(self._preview_window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self._preview_window_name, 900, 1000)
        cv2.setMouseCallback(self._preview_window_name, self._on_preview_mouse)
        cv2.imshow(self._preview_window_name, img)
        cv2.waitKey(1)

    def _parse_float(self, text: str, *, name: str) -> float:
        try:
            return float(text)
        except Exception as exc:
            raise ValueError(f"invalid {name}: {text}") from exc

    def _load_image(self) -> None:
        from tkinter import messagebox

        image_path = self.image_path_var.get().strip()
        if not image_path:
            messagebox.showerror("ocv", "image path is empty")
            return
        try:
            roll_deg = self._parse_float(self.roll_var.get().strip(), name="roll_deg")
            theta1_deg = self._parse_float(self.theta1_var.get().strip(), name="theta1_deg")
            theta2_deg = self._parse_float(self.theta2_var.get().strip(), name="theta2_deg")
            measurement, preview_img = extract_from_image(
                image_path,
                roll_deg=roll_deg,
                theta1_deg=theta1_deg,
                theta2_deg=theta2_deg,
            )
            self._current_measurement = measurement
            self._current_frame = cv2.imread(str(image_path))
            self._current_row_no = None
            self._pending_correct_index = None
            self._pending_click_xy = None
            self._show_preview(preview_img)
            self.status_var.set(
                f"loaded image: {image_path}\n"
                f"start_red={measurement.start_red_xy} yellow={measurement.yellow_anchor_xy}\n"
                f"preview updated; ready for correction or Put Row"
            )
        except Exception as exc:
            messagebox.showerror("ocv", str(exc))

    def _put_row(self) -> None:
        from tkinter import messagebox

        if self._current_measurement is None:
            messagebox.showerror("ocv", "no loaded image/measurement")
            return
        try:
            row_no = int(self.row_no_var.get().strip())
            if row_no < 1:
                raise ValueError(f"invalid row_no: {row_no}")
            row = self.collector.put_row(row_no, self._current_measurement)
            self._current_row_no = row_no
            self.rows_label.config(text=f"rows: {len(self.collector.rows)}")
            self.status_var.set(
                f"stored row: {row['number']}\n"
                f"1_x={row['1_x']:.1f}, 1_y={row['1_y']:.1f} (origin)\n"
                f"2_x={row['2_x']:.1f}, 2_y={row['2_y']:.1f}"
            )
        except Exception as exc:
            messagebox.showerror("ocv", str(exc))

    def _start_correct_point(self) -> None:
        from tkinter import messagebox

        if self._current_measurement is None or self._current_frame is None:
            messagebox.showerror("ocv", "no loaded image to correct")
            return
        try:
            point_index = int(self.correct_index_var.get().strip())
        except Exception:
            messagebox.showerror("ocv", f"invalid point index: {self.correct_index_var.get().strip()}")
            return
        if point_index < 0 or point_index >= len(self._current_measurement.joint_xy):
            messagebox.showerror("ocv", f"point index out of range: 0..{len(self._current_measurement.joint_xy) - 1}")
            return
        self._pending_correct_index = point_index
        self.status_var.set(
            f"pending correction: point {point_index}\n"
            f"click a new location in the preview window"
        )

    def _on_preview_mouse(self, event: int, x: int, y: int, flags: int, param: Any) -> None:
        del flags, param
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self._pending_correct_index is None:
            return
        if self._pending_click_xy is not None:
            return
        self._pending_click_xy = (int(x), int(y))
        self.root.after_idle(self._apply_pending_correction)

    def _apply_pending_correction(self) -> None:
        if self._pending_correct_index is None or self._pending_click_xy is None:
            return
        if self._current_measurement is None or self._current_frame is None:
            self._pending_click_xy = None
            self._pending_correct_index = None
            return
        corrected_index = int(self._pending_correct_index)
        click_xy = tuple(self._pending_click_xy)
        self._pending_correct_index = None
        self._pending_click_xy = None

        corrected = _with_corrected_point(self._current_measurement, corrected_index, click_xy)
        self._current_measurement = corrected
        if self._current_row_no is not None and self._current_row_no in self.collector.rows:
            self.collector.rows[self._current_row_no] = measurement_to_row(corrected, row_no=self._current_row_no)
        preview_img = _render_measurement_preview(self._current_frame, corrected)
        self._show_preview(preview_img)
        if self._current_row_no is None:
            self.status_var.set(
                f"corrected point {corrected_index} -> ({click_xy[0]}, {click_xy[1]})\n"
                f"loaded measurement updated; not yet stored to any row"
            )
        else:
            self.status_var.set(
                f"corrected point {corrected_index} -> ({click_xy[0]}, {click_xy[1]})\n"
                f"row {self._current_row_no} updated in memory"
            )

    def _export_csv(self) -> None:
        from tkinter import filedialog, messagebox

        if not self.collector.rows:
            messagebox.showerror("ocv", "no rows collected")
            return
        out_path = filedialog.asksaveasfilename(
            title="Export CSV",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not out_path:
            return
        try:
            self.collector.export_csv(out_path)
            self.status_var.set(f"exported: {out_path}\nrows: {len(self.collector.rows)}")
        except Exception as exc:
            messagebox.showerror("ocv", str(exc))

    def _clear_rows(self) -> None:
        self.collector.clear()
        self._current_measurement = None
        self._current_frame = None
        self._current_row_no = None
        self._pending_correct_index = None
        self._pending_click_xy = None
        self.rows_label.config(text="rows: 0")
        self.status_var.set("cleared")

    def _pump_cv_events(self) -> None:
        try:
            cv2.waitKey(1)
        finally:
            if self.root.winfo_exists():
                self.root.after(30, self._pump_cv_events)

    def _on_close(self) -> None:
        try:
            cv2.destroyAllWindows()
        finally:
            self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    CsvGui().run()


if __name__ == "__main__":
    main()
