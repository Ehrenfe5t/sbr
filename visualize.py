#!/usr/bin/env python3
"""
SBR Geometric Pathfinding — 3D Interactive Visualizer (PyQt5 + PyVista)
PyCharm + Anaconda: pip install pyvista PyQt5 trimesh

Usage: python visualize.py [sbr_result.json]
"""
import json, sys, os, glob
import numpy as np
from collections import defaultdict

# ═══════════════════════════════════════════════════════════
# Data loading
# ═══════════════════════════════════════════════════════════
def load_result(path: str) -> dict:
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)

def load_obj_mesh(obj_path: str):
    try:
        import trimesh
        import pyvista as pv
        mesh = trimesh.load(obj_path, process=False, skip_materials=True)
        return pv.wrap(mesh)
    except:
        try:
            import pyvista as pv
            return pv.read(obj_path)
        except:
            return None

# ═══════════════════════════════════════════════════════════
# PyQt5 + PyVista Interactive Window
# ═══════════════════════════════════════════════════════════
try:
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QLabel, QComboBox, QPushButton, QFileDialog, QSpinBox,
        QGroupBox, QSplitter, QTextEdit
    )
    from PyQt5.QtCore import Qt
    import pyvista as pv
    from pyvistaqt import QtInteractor
    HAS_GUI = True
except ImportError:
    HAS_GUI = False

if HAS_GUI:
    class SbrVisualizer(QMainWindow):
        def __init__(self, result_path=None):
            super().__init__()
            self.setWindowTitle("SBR Geometric Pathfinding — 3D Visualizer")
            self.setGeometry(100, 50, 1800, 1100)

            self.result_data = None
            self.current_rx = 0
            self.current_path = 0
            self.show_scene = True
            self.show_all_rx = True
            self.show_all_paths = False
            self.max_paths_all = 200
            self.scene_mesh = None
            self.scene_file = ""
            self._camera_pos = None       # ★ 保存视角
            self._first_render = True     # ★ 首次渲染标志
            self._seq_filter = ""          # ★ 按交互序列类型筛选
            self._seq_paths = []           # ★ 当前序列类型下的路径索引列表

            # Colors for paths
            self.PATH_COLORS = [
                [0.10, 0.55, 0.95], [0.95, 0.35, 0.05], [0.05, 0.75, 0.35],
                [0.85, 0.15, 0.55], [0.55, 0.35, 0.05], [0.05, 0.55, 0.75],
                [0.75, 0.05, 0.15], [0.35, 0.35, 0.75],
            ]
            self.INTERACTION_COLORS = {'r': 'orange', 't': 'cyan', 'd': 'magenta'}

            self._init_ui()
            if result_path:
                self._load_file(result_path)

        def _init_ui(self):
            central = QWidget()
            self.setCentralWidget(central)
            main_layout = QVBoxLayout(central)
            main_layout.setContentsMargins(8, 8, 8, 8)

            # ── Top control bar ──
            ctrl = QWidget()
            ctrl_layout = QHBoxLayout(ctrl)
            ctrl_layout.setContentsMargins(0, 0, 0, 0)

            # File
            file_grp = QGroupBox("Result File")
            fl = QHBoxLayout(file_grp)
            self.file_label = QLabel("(none)")
            self.file_label.setMaximumWidth(300)
            fl.addWidget(self.file_label)
            btn_open = QPushButton("Open JSON...")
            btn_open.clicked.connect(self._on_open)
            fl.addWidget(btn_open)
            btn_refresh = QPushButton("Refresh")
            btn_refresh.clicked.connect(self._on_refresh)
            fl.addWidget(btn_refresh)
            ctrl_layout.addWidget(file_grp)

            # Rx selection
            rx_grp = QGroupBox("Rx Selection")
            rl = QHBoxLayout(rx_grp)
            rl.addWidget(QLabel("Rx:"))
            self.rx_combo = QComboBox()
            self.rx_combo.setMinimumWidth(250)
            self.rx_combo.currentIndexChanged.connect(self._on_rx_change)
            rl.addWidget(self.rx_combo)
            ctrl_layout.addWidget(rx_grp)

            # Path selection
            path_grp = QGroupBox("Path Selection")
            pl = QHBoxLayout(path_grp)
            pl.addWidget(QLabel("Path:"))
            self.path_spin = QSpinBox()
            self.path_spin.setMinimum(0)
            self.path_spin.setMaximum(99999)
            self.path_spin.setValue(0)
            self.path_spin.valueChanged.connect(self._on_path_change)
            pl.addWidget(self.path_spin)
            pl.addWidget(QLabel("/"))
            self.path_total = QLabel("0")
            pl.addWidget(self.path_total)
            ctrl_layout.addWidget(path_grp)

            # Sequence type filter
            seq_grp = QGroupBox("Sequence Filter")
            sl = QHBoxLayout(seq_grp)
            sl.addWidget(QLabel("Type:"))
            self.seq_combo = QComboBox()
            self.seq_combo.setMinimumWidth(120)
            self.seq_combo.currentIndexChanged.connect(self._on_seq_change)
            sl.addWidget(self.seq_combo)
            ctrl_layout.addWidget(seq_grp)

            # Display toggles
            disp_grp = QGroupBox("Display")
            dl = QHBoxLayout(disp_grp)
            from PyQt5.QtWidgets import QCheckBox
            self.chk_scene = QCheckBox("Scene")
            self.chk_scene.setChecked(True)
            self.chk_scene.stateChanged.connect(self._update_render)
            dl.addWidget(self.chk_scene)
            self.chk_all_rx = QCheckBox("All Rx")
            self.chk_all_rx.setChecked(True)
            self.chk_all_rx.stateChanged.connect(self._update_render)
            dl.addWidget(self.chk_all_rx)
            self.chk_all_paths = QCheckBox("All Paths")
            self.chk_all_paths.setChecked(False)
            self.chk_all_paths.stateChanged.connect(self._update_render)
            dl.addWidget(self.chk_all_paths)
            ctrl_layout.addWidget(disp_grp)

            main_layout.addWidget(ctrl)

            # ── Splitter: 3D view + Info panel ──
            splitter = QSplitter(Qt.Horizontal)

            # 3D view
            self.plotter = QtInteractor(self)
            self.plotter.set_background("white")
            self.plotter.enable_anti_aliasing()
            splitter.addWidget(self.plotter)

            # Info panel
            info_widget = QWidget()
            info_widget.setMaximumWidth(350)
            info_layout = QVBoxLayout(info_widget)
            info_layout.setContentsMargins(4, 4, 4, 4)

            info_layout.addWidget(QLabel("Path Info"))
            self.info_text = QTextEdit()
            self.info_text.setReadOnly(True)
            self.info_text.setMaximumHeight(200)
            info_layout.addWidget(self.info_text)

            info_layout.addWidget(QLabel("Summary"))
            self.summary_text = QTextEdit()
            self.summary_text.setReadOnly(True)
            info_layout.addWidget(self.summary_text)

            splitter.addWidget(info_widget)
            splitter.setSizes([1400, 350])
            main_layout.addWidget(splitter, stretch=1)

        def _load_file(self, path):
            try:
                self.result_data = load_result(path)
                self.file_label.setText(os.path.basename(path))
                self.scene_file = self.result_data.get('scene_file', '')

                # Load scene mesh
                if self.scene_file and os.path.exists(self.scene_file):
                    try:
                        import trimesh
                        raw = trimesh.load(self.scene_file, process=False, skip_materials=True)
                        self.scene_mesh = pv.wrap(raw)
                    except:
                        self.scene_mesh = None

                # Populate Rx combo
                self.rx_combo.blockSignals(True)
                self.rx_combo.clear()
                records = self.result_data.get('rx_records', [])
                for rec in records:
                    p = rec.get('position', [0,0,0])
                    pw = rec.get('total_power_dBm', 0)
                    n = rec.get('hit_count', 0)
                    self.rx_combo.addItem(f"Rx[{rec['rx_index']}] ({p[0]:.1f},{p[1]:.1f},{p[2]:.1f}) P={pw:.1f}dBm {n}paths")
                self.rx_combo.blockSignals(False)

                # Update summary
                s = self.result_data.get('stats', {})
                records = self.result_data.get('rx_records', [])
                total_p = sum(r.get('hit_count', 0) for r in records)
                rx_hit = sum(1 for r in records if r.get('hit_count', 0) > 0)
                counts = defaultdict(int)
                for r in records:
                    for p in r.get('paths', []):
                        for ch in p.get('sequence', ''):
                            if ch == 'r': counts['R'] += 1
                            elif ch == 't': counts['T'] += 1
                            elif ch == 'd': counts['D'] += 1
                self.summary_text.setPlainText(
                    f"Faces: {s.get('total_faces',0)}  Wedges: {s.get('total_wedges',0)}\n"
                    f"Rays: {s.get('total_rays',0):,}\n"
                    f"BVH: {s.get('bvh_build_ms',0):.0f}ms  Trace: {s.get('sbr_trace_ms',0):.0f}ms\n"
                    f"Rx hit: {rx_hit}/{len(records)}\n"
                    f"Total paths: {total_p}\n"
                    f"R={counts['R']} T={counts['T']} D={counts['D']}"
                )

                self.current_rx = 0
                self.current_path = 0
                self._first_render = True     # ★ 新文件重置视角
                self._camera_pos = None
                self._on_rx_change(0)          # 填充 seq combo + path spin
                self._update_render()

            except Exception as e:
                self.info_text.setPlainText(f"Error: {e}")

        def _on_open(self):
            path, _ = QFileDialog.getOpenFileName(self, "Open SBR Result", "", "JSON Files (*.json);;All (*)")
            if path:
                self._load_file(path)

        def _on_refresh(self):
            if self.scene_file:
                self._load_file(self.scene_file)
            elif hasattr(self, '_last_path'):
                self._load_file(self._last_path)

        def _on_rx_change(self, idx):
            if idx < 0 or not self.result_data: return
            self.current_rx = idx
            records = self.result_data.get('rx_records', [])
            if idx < len(records):
                paths = records[idx].get('paths', [])
                # ── 按序列类型分组 ──
                seq_map = {}
                for pi, p in enumerate(paths):
                    sq = p.get('sequence', '?')
                    seq_map.setdefault(sq, []).append(pi)
                sorted_seqs = sorted(seq_map.keys(),
                    key=lambda s: (-len(s), s))  # complex first
                # Populate seq combo
                self.seq_combo.blockSignals(True)
                self.seq_combo.clear()
                self.seq_combo.addItem(f"(all) [{len(paths)}]", "")
                for sq in sorted_seqs:
                    self.seq_combo.addItem(f"{sq} [{len(seq_map[sq])}]", sq)
                self.seq_combo.blockSignals(False)
                self._apply_seq_filter(paths)

        def _on_seq_change(self, _idx):
            if not self.result_data: return
            records = self.result_data.get('rx_records', [])
            if self.current_rx < len(records):
                self._apply_seq_filter(records[self.current_rx].get('paths', []))
            self._update_render()

        def _apply_seq_filter(self, paths):
            sq = self.seq_combo.currentData()
            if sq:
                self._seq_filter = sq
                self._seq_paths = [pi for pi, p in enumerate(paths) if p.get('sequence','') == sq]
            else:
                self._seq_filter = ""
                self._seq_paths = list(range(len(paths)))
            self.current_path = 0
            self.path_spin.blockSignals(True)
            self.path_spin.setMaximum(max(0, len(self._seq_paths) - 1))
            self.path_spin.setValue(0)
            self.path_total.setText(str(len(self._seq_paths)))
            self.path_spin.blockSignals(False)

        def _on_path_change(self, val):
            self.current_path = val
            self._update_render()

        def _update_render(self):
            # ★ 保存当前视角 (首次渲染除外)
            if not self._first_render and self.plotter.renderer.camera:
                self._camera_pos = self.plotter.camera_position

            self.plotter.clear()
            if not self.result_data: return

            records = self.result_data.get('rx_records', [])

            # Scene mesh
            if self.chk_scene.isChecked() and self.scene_mesh:
                self.plotter.add_mesh(self.scene_mesh, color='lightgray', opacity=0.35,
                                      style='surface', show_edges=True, edge_color='gray', line_width=0.5)

            tx = self.result_data.get('tx', {})
            tx_pt = np.array([[tx.get('x',0), tx.get('y',0), tx.get('z',0)]], dtype=np.float32)
            self.plotter.add_points(tx_pt, color='red', point_size=35, render_points_as_spheres=True)

            # All Rx
            if self.chk_all_rx.isChecked():
                rx_pts, rx_pwr = [], []
                for rec in records:
                    pos = rec.get('position', [0,0,0])
                    rx_pts.append(pos)
                    rx_pwr.append(max(-160, rec.get('total_power_dBm', -160)))
                if rx_pts:
                    self.plotter.add_points(np.array(rx_pts, dtype=np.float32),
                        scalars=np.array(rx_pwr, dtype=np.float32),
                        cmap='hot', point_size=22, render_points_as_spheres=True,
                        scalar_bar_args={'title': 'Power (dBm)', 'vertical': True,
                                         'position_x': 0.02, 'position_y': 0.05})

            # Highlight current Rx
            if self.current_rx < len(records):
                pos = records[self.current_rx].get('position', [0,0,0])
                self.plotter.add_points(np.array([pos], dtype=np.float32),
                    color='lime', point_size=30, render_points_as_spheres=True)

            # ── Paths ──
            if self.current_rx < len(records):
                paths = records[self.current_rx].get('paths', [])
                if self.chk_all_paths.isChecked():
                    # 显示所选序列类型的所有路径 (最多 max_paths_all)
                    for i, pi in enumerate(self._seq_paths):
                        if i >= self.max_paths_all: break
                        self._draw_path(paths[pi], pi, highlight=(i == self.current_path))
                else:
                    # ★ 仅显示当前所选路径 (别无其他)
                    if self.current_path < len(self._seq_paths):
                        pi = self._seq_paths[self.current_path]
                        self._draw_path(paths[pi], pi, highlight=True)

            self.plotter.show_grid(color='gray', font_size=8)

            # ★ 恢复视角
            if self._camera_pos is not None:
                self.plotter.camera_position = self._camera_pos
            elif self._first_render:
                self.plotter.camera_position = 'iso'
            self._first_render = False

            # Update path info
            if self.current_rx < len(records):
                paths = records[self.current_rx].get('paths', [])
                if self.current_path < len(self._seq_paths):
                    pi = self._seq_paths[self.current_path]
                    p = paths[pi]
                    self.info_text.setPlainText(
                        f"Rx[{self.current_rx}] Path #{pi} (filtered #{self.current_path})\n"
                        f"Length: {p.get('len',0):.3f}m\n"
                        f"Nodes: {p.get('nodes',0)}\n"
                        f"LoS: {p.get('los',False)}\n"
                        f"Has Tx: {p.get('has_tx',False)}\n"
                        f"Sequence: {p.get('sequence','?')}\n"
                        f"Points: {len(p.get('points',[]))}"
                    )

        def _draw_path(self, p, pi, highlight=True):
            pts = p.get('points', [])
            if len(pts) < 2: return
            arr = np.array(pts, dtype=np.float32)
            seq = p.get('sequence', '')
            is_los = p.get('los', False)
            seq_colors = {'T': [1.0, 0.2, 0.2], 'R': [0.2, 0.6, 1.0],  # T=Tx(red), R=Rx(blue)
                          'r': [1.0, 0.5, 0.0], 't': [0.0, 0.7, 0.7],  # r=reflection(orange), t=transmission(cyan)
                          'd': [0.8, 0.0, 0.8]}                         # d=diffraction(magenta)

            if highlight:
                # ★ 逐段着色: 每段颜色 = 该段终点节点的交互类型
                for si in range(len(pts) - 1):
                    ntype = seq[si + 1] if (si + 1) < len(seq) else 'R'
                    seg_color = seq_colors.get(ntype, [0.5, 0.5, 0.5])
                    seg = np.array([pts[si], pts[si + 1]], dtype=np.float32)
                    self.plotter.add_lines(seg, color=seg_color, width=4, connected=True)
                # 交互节点 (大球 + 颜色标记)
                for ni in range(1, len(pts) - 1):
                    ntype = seq[ni] if ni < len(seq) else '?'
                    nc = self.INTERACTION_COLORS.get(ntype, 'gray')
                    self.plotter.add_points(arr[ni:ni+1], color=nc, point_size=14,
                                            render_points_as_spheres=True)
            else:
                width = 1.2 if is_los else 0.6
                color = self.PATH_COLORS[pi % len(self.PATH_COLORS)]
                alpha_color = [c * (0.6 if is_los else 0.15) for c in color]
                self.plotter.add_lines(arr, color=alpha_color, width=width, connected=True)
                # 小节点
                for ni in range(1, len(pts) - 1):
                    ntype = seq[ni] if ni < len(seq) else '?'
                    nc = self.INTERACTION_COLORS.get(ntype, 'gray')
                    self.plotter.add_points(arr[ni:ni+1], color=nc, point_size=5,
                                            render_points_as_spheres=True)


def run_headless(result_path):
    """Fallback: console summary when GUI not available."""
    r = load_result(result_path)
    s = r.get('stats', {})
    records = r.get('rx_records', [])
    total_p = sum(rec.get('hit_count', 0) for rec in records)
    rx_hit = sum(1 for rec in records if rec.get('hit_count', 0) > 0)
    counts = defaultdict(int)
    for rec in records:
        for p in rec.get('paths', []):
            for ch in p.get('sequence', ''):
                if ch == 'r': counts['R'] += 1
                elif ch == 't': counts['T'] += 1
                elif ch == 'd': counts['D'] += 1
    print("=" * 65)
    print(f"  SBR Results  |  Faces: {s.get('total_faces',0)}  Wedges: {s.get('total_wedges',0)}")
    print(f"  Rays: {s.get('total_rays',0):,}  |  BVH: {s.get('bvh_build_ms',0):.0f}ms  Trace: {s.get('sbr_trace_ms',0):.0f}ms")
    print(f"  Rx: {rx_hit}/{len(records)} hit  |  Paths: {total_p}  |  R={counts['R']} T={counts['T']} D={counts['D']}")
    print("=" * 65)
    for rec in records:
        paths = rec.get('paths', [])
        if not paths: continue
        pos = rec.get('position', [0,0,0])
        print(f"\n  Rx[{rec['rx_index']}] ({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f})  P={rec.get('total_power_dBm',0):.1f}dBm  [{len(paths)} paths]")
        for i, p in enumerate(paths[:3]):
            print(f"    #{i}: {p['len']:.2f}m  {p['nodes']}nodes  {'LoS' if p.get('los') else 'NLoS'}  {p.get('sequence','?')}")
    print("\n[NOTE] Install PyQt5 + pyvista for interactive GUI: pip install PyQt5 pyvista pyvistaqt trimesh")


# ═══════════════════════════════════════════════════════════
if __name__ == '__main__':
    if not HAS_GUI:
        if len(sys.argv) > 1:
            run_headless(sys.argv[1])
        else:
            print("Usage: python visualize.py <sbr_result.json>")
            print("Install GUI deps: pip install PyQt5 pyvista pyvistaqt trimesh")
        sys.exit(0)

    app = QApplication(sys.argv)
    init_path = sys.argv[1] if len(sys.argv) > 1 else None
    window = SbrVisualizer(init_path)
    window.show()
    sys.exit(app.exec_())
