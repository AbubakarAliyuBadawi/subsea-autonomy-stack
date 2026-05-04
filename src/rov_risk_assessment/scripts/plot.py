#!/usr/bin/env python3
"""
Docking BN Validation Visualizer
=================================
Reads rosbag2 SQLite database and produces a multi-panel figure showing
how BN evidence evolves throughout the docking approach, with the
DockingModeReliability output correlated against distance, ArUco visibility,
and operator cognitive state.

Usage:
    python3 visualize_docking_bn.py \
        --bag /home/badawi/Desktop/PhD_Autumn_2025/subsea-autonomy-stack/rosbag2_2026_01_16-14_26_19/rosbag2_2026_01_16-14_26_19_0.db3 \
        --output docking_bn_validation.png

Requirements:
    pip install sqlite3 numpy pandas matplotlib scipy
    (all standard — no ROS needed)
"""

import sqlite3
import struct
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as gridspec
from matplotlib.lines import Line2D
from scipy.ndimage import uniform_filter1d
import warnings
warnings.filterwarnings('ignore')

# ─── CDR deserializers (no ROS needed) ───────────────────────────────────────

def read_float32(data):
    """std_msgs/Float32"""
    try:
        return struct.unpack_from('<f', data, 4)[0]
    except Exception:
        return None

def read_float64(data):
    """std_msgs/Float64"""
    try:
        return struct.unpack_from('<d', data, 4)[0]
    except Exception:
        return None

def read_string(data):
    """std_msgs/String"""
    try:
        offset = 4
        str_len = struct.unpack_from('<I', data, offset)[0]
        return data[offset+4: offset+4+str_len].decode('utf-8', errors='replace').rstrip('\x00')
    except Exception:
        return None

def read_bool(data):
    """std_msgs/Bool"""
    try:
        return bool(struct.unpack_from('<?', data, 4)[0])
    except Exception:
        return None

def read_int32(data):
    """std_msgs/Int32"""
    try:
        return struct.unpack_from('<i', data, 4)[0]
    except Exception:
        return None

# ─── Topic → deserializer map ────────────────────────────────────────────────

TOPICS = {
    '/docking/reliability':             ('float32',  'Docking Reliability (BN output)'),
    '/docking/mode_autonomous_prob':    ('float32',  'P(Autonomous | evidence)'),
    '/docking/mode_human_prob':         ('float32',  'P(Human | evidence)'),
    '/docking/mode_shared_prob':        ('float32',  'P(Shared | evidence)'),
    '/docking/mode_recommendation':     ('string',   'Mode Recommendation'),
    '/blueye/distance_to_dock':         ('float64',  'Distance to Dock (m)'),
    '/blueye/docking_station_detected': ('bool',     'Docking Station Detected'),
    '/blueye/aruco_visibility':         ('string',   'ArUco Visibility'),
    '/docking/visual_guidance_quality': ('string',   'Visual Guidance Quality'),
    '/blueye/human/fatigue':            ('float32',  'Operator Fatigue'),
    '/blueye/human/stress':             ('float32',  'Operator Stress'),
    '/blueye/battery_percentage':       ('float64',  'Battery (%)'),
    '/blueye/battery_level':            ('string',   'Battery Level'),
    '/blueye/usbl_strength':            ('string',   'USBL Strength'),
    '/mission_state':                   ('int32',    'Mission State'),
    '/blueye/odometry_flu/gt':          ('odom_pos', 'Odometry Position'),
}

# Maps battery_level string states to numeric % for plotting
BATTERY_LEVEL_MAP = {'High': 90, 'Medium': 60, 'Low': 30, 'Critical': 10}

def read_odom_position(data):
    """nav_msgs/Odometry — extract x, y from pose.pose.position (CDR offset 4+header)"""
    try:
        # Skip CDR header (4) + ROS header stamp (8) + frame_id string
        offset = 4
        # std_msgs/Header: stamp (8 bytes), frame_id (4 len + N bytes)
        frame_len = struct.unpack_from('<I', data, offset + 8)[0]
        offset += 8 + 4 + frame_len
        # child_frame_id string
        child_len = struct.unpack_from('<I', data, offset)[0]
        offset += 4 + child_len
        # geometry_msgs/PoseWithCovariance: pose.position.x, y, z
        x, y = struct.unpack_from('<dd', data, offset)
        return (x, y)
    except Exception:
        return None

DESERIALIZERS = {
    'float32':  read_float32,
    'float64':  read_float64,
    'bool':     read_bool,
    'string':   read_string,
    'int32':    read_int32,
    'odom_pos': read_odom_position,
}

# ─── Load bag ────────────────────────────────────────────────────────────────

def load_bag(db_path):
    conn = sqlite3.connect(db_path)
    cur  = conn.cursor()

    # Get topic id → name mapping
    cur.execute("SELECT id, name FROM topics")
    topic_map = {row[0]: row[1] for row in cur.fetchall()}

    wanted = {name: tid for tid, name in topic_map.items() if name in TOPICS}
    print(f"  Found {len(wanted)}/{len(TOPICS)} requested topics in bag:")
    for name in TOPICS:
        status = "✓" if name in wanted else "✗ MISSING"
        print(f"    {status}  {name}")

    data = {name: [] for name in TOPICS}

    if not wanted:
        conn.close()
        raise RuntimeError("No matching topics found in bag file.")

    id_list = ','.join(str(i) for i in wanted.values())
    cur.execute(
        f"SELECT topic_id, timestamp, data FROM messages WHERE topic_id IN ({id_list}) ORDER BY timestamp"
    )

    for topic_id, timestamp_ns, raw in cur.fetchall():
        topic_name = topic_map[topic_id]
        dtype, _ = TOPICS[topic_name]
        val = DESERIALIZERS[dtype](raw)
        if val is not None:
            data[topic_name].append((timestamp_ns, val))

    conn.close()

    # Convert to DataFrames with time in seconds from bag start
    dfs = {}
    t0 = None
    for name, rows in data.items():
        if rows:
            df = pd.DataFrame(rows, columns=['timestamp_ns', 'value'])
            if t0 is None:
                t0 = df['timestamp_ns'].min()
            df['t'] = (df['timestamp_ns'] - t0) / 1e9
            dfs[name] = df

    # Align all times to common t0
    if t0 is not None:
        for name, df in dfs.items():
            df['t'] = (df['timestamp_ns'] - t0) / 1e9

    print(f"\n  Bag duration: {max(df['t'].max() for df in dfs.values()):.1f} s")
    return dfs

# ─── Helpers ─────────────────────────────────────────────────────────────────

def smooth(series, window=15):
    arr = np.array(series, dtype=float)
    if len(arr) < window:
        return arr
    return uniform_filter1d(arr, size=window)

def to_numpy(series):
    """Always return a plain numpy array regardless of pandas version."""
    return np.array(series, dtype=float)

def encode_string_series(df, mapping):
    """Map string values to numeric using provided mapping dict."""
    return df['value'].map(mapping).fillna(np.nan)

MODE_MAP  = {'Autonomous': 3, 'Shared': 2, 'Human': 1}
VGQ_MAP   = {'High': 3, 'Medium': 2, 'Low': 1, 'None': 0}
USBL_MAP  = {'Strong': 3, 'Moderate': 2, 'Weak': 1}
ARUCO_MAP = {'All': 3, 'Partial': 2, 'None': 0}

# ─── Plot ────────────────────────────────────────────────────────────────────

def make_figure(dfs, output_path):
    # Decide on docking-phase window using mission_state if available
    t_start, t_end = 0, None
    if '/mission_state' in dfs:
        ms = dfs['/mission_state']
        unique_states = sorted(ms['value'].unique())
        print(f"\n  mission_state unique values in bag: {unique_states}")
        docking_state = None
        for candidate in [3, max(unique_states)]:
            rows = ms[ms['value'] == candidate]
            if not rows.empty:
                docking_state = candidate
                break
        if docking_state is not None:
            docking_rows = ms[ms['value'] == docking_state]
            t_start = float(docking_rows['t'].min())
            t_end   = float(docking_rows['t'].max())
            print(f"  Using mission_state={docking_state}: t={t_start:.1f}s → {t_end:.1f}s")
        else:
            print("  Could not detect docking phase — showing full bag")

    def clip(df):
        mask = df['t'] >= t_start
        if t_end:
            mask &= df['t'] <= t_end
        return df[mask].copy()

    # ── Colour palette ─────────────────────────────────────────────────────
    AXES_BG = 'white'
    GRID_C  = '#dddddd'
    TITLE_C = '#1a1a2e'
    LABEL_C = '#333333'
    TICK_C  = '#555555'

    # Per-mode colours consistent across all panels
    C_AUTO   = '#1565c0'   # blue
    C_SHARED = '#e65100'   # orange
    C_HUMAN  = '#558b2f'   # green
    MODE_COLOR = {'Autonomous': C_AUTO, 'Shared': C_SHARED, 'Human': C_HUMAN}

    # ── Figure: 3 panels ──────────────────────────────────────────────────
    fig = plt.figure(figsize=(13, 12))
    fig.patch.set_facecolor('white')

    gs = gridspec.GridSpec(
        3, 1, figure=fig,
        height_ratios=[2.2, 1.4, 1],
        hspace=0.50,
        top=0.91, bottom=0.08, left=0.10, right=0.88
    )

    axes = [fig.add_subplot(gs[i]) for i in range(3)]
    for ax in axes:
        ax.set_facecolor(AXES_BG)
        ax.tick_params(colors=TICK_C, labelsize=9)
        ax.xaxis.label.set_color(LABEL_C)
        ax.yaxis.label.set_color(LABEL_C)
        for spine in ax.spines.values():
            spine.set_edgecolor('#aaaaaa')
        ax.grid(True, color=GRID_C, linewidth=0.7, alpha=0.9, zorder=0)

    t_label = 'Mission time (s)'

    # ── Panel 1: All three P(mode | evidence) + mode recommendation ────────
    ax1 = axes[0]
    ax1.set_title('(a)  BN Output: P(mode | evidence) & Mode Recommendation',
                  color=TITLE_C, fontsize=10, pad=6, fontweight='bold', loc='left')

    # Three probability lines
    prob_topics = [
        ('/docking/mode_autonomous_prob', C_AUTO,   'P(Autonomous | evidence)'),
        ('/docking/mode_shared_prob',     C_SHARED, 'P(Shared | evidence)'),
        ('/docking/mode_human_prob',      C_HUMAN,  'P(Human | evidence)'),
    ]

    # Fallback: if individual prob topics absent, use /docking/reliability for autonomous
    has_prob_topics = any(t in dfs for t, _, _ in prob_topics)

    if has_prob_topics:
        for topic, color, label in prob_topics:
            if topic in dfs:
                df_p = clip(dfs[topic])
                t_p = to_numpy(df_p['t'])
                v_p = smooth(to_numpy(df_p['value']))
                ax1.plot(t_p, v_p, color=color, linewidth=2.0, label=label, zorder=2)
                ax1.fill_between(t_p, v_p, alpha=0.06, color=color, zorder=1)
    elif '/docking/reliability' in dfs:
        df_r = clip(dfs['/docking/reliability'])
        t_r = to_numpy(df_r['t'])
        v_r = smooth(to_numpy(df_r['value']))
        ax1.plot(t_r, v_r, color=C_AUTO, linewidth=2.0,
                 label='P(Autonomous | evidence)', zorder=2)
        ax1.fill_between(t_r, v_r, alpha=0.08, color=C_AUTO, zorder=1)

    ax1.axhline(0.5, color='#888888', linewidth=1.0, linestyle='--', zorder=1)
    ax1.text(0.01, 0.505, 'P = 0.50', color='#888888', fontsize=7.5,
             transform=ax1.get_yaxis_transform(), va='bottom')
    ax1.set_ylim(-0.02, 1.08)
    ax1.set_ylabel('Probability', color=LABEL_C, fontsize=9)
    ax1.yaxis.set_label_coords(-0.07, 0.5)

    # Mode recommendation scatter on right axis
    if '/docking/mode_recommendation' in dfs:
        ax1b = ax1.twinx()
        ax1b.set_facecolor(AXES_BG)
        df_m = clip(dfs['/docking/mode_recommendation'])
        for state, yval in MODE_MAP.items():
            mask = df_m['value'] == state
            subset = df_m[mask]
            if not subset.empty:
                ax1b.scatter(to_numpy(subset['t']), [yval] * len(subset),
                             c=MODE_COLOR[state], s=28, alpha=0.90,
                             zorder=4, label=f'Rec: {state}', marker='D')
        ax1b.set_ylim(0.4, 3.8)
        ax1b.set_yticks([1, 2, 3])
        ax1b.set_yticklabels(['Human', 'Shared', 'Autonomous'], fontsize=9, color=TICK_C)
        ax1b.tick_params(right=True, colors=TICK_C)
        ax1b.set_ylabel('Mode recommendation', color=LABEL_C, fontsize=9,
                        rotation=270, labelpad=15)
        for spine in ax1b.spines.values():
            spine.set_edgecolor('#aaaaaa')
        handles_l, labels_l = ax1.get_legend_handles_labels()
        handles_r, labels_r = ax1b.get_legend_handles_labels()
        ax1.legend(handles_l + handles_r, labels_l + labels_r,
                   loc='center right', fontsize=8, framealpha=0.92,
                   edgecolor='#cccccc', ncol=1)
    else:
        ax1.legend(loc='lower right', fontsize=8, framealpha=0.9, edgecolor='#cccccc')

    # ── Panel 2: Distance to dock + ArUco visibility ───────────────────────
    ax2 = axes[1]
    ax2.set_title('(b)  AUV Approach: Distance to Dock & Marker Visibility',
                  color=TITLE_C, fontsize=10, pad=6, fontweight='bold', loc='left')

    if '/blueye/distance_to_dock' in dfs:
        df_d = clip(dfs['/blueye/distance_to_dock'])
        t_d = to_numpy(df_d['t'])
        v_d = smooth(to_numpy(df_d['value']))
        ax2.plot(t_d, v_d, color='#c62828', linewidth=2.0,
                 label='Distance to dock (m)', zorder=2)
        ax2.set_ylabel('Distance (m)', color=LABEL_C, fontsize=9)
        ax2.yaxis.set_label_coords(-0.07, 0.5)
    elif '/blueye/odometry_flu/gt' in dfs:
        df_o = clip(dfs['/blueye/odometry_flu/gt'])
        try:
            xy = np.array([(v[0], v[1]) if isinstance(v, tuple) else (0, 0)
                           for v in df_o['value']], dtype=float)
            dist = np.sqrt(np.sum((xy - xy[0])**2, axis=1))
            ax2.plot(to_numpy(df_o['t']), smooth(dist), color='#c62828', linewidth=2.0,
                     label='Distance from start (m, proxy)', zorder=2)
            ax2.set_ylabel('Distance (m)', color=LABEL_C, fontsize=9)
            ax2.yaxis.set_label_coords(-0.07, 0.5)
        except Exception:
            pass

    if '/blueye/aruco_visibility' in dfs:
        df_ar = clip(dfs['/blueye/aruco_visibility'])
        ax2b = ax2.twinx()
        ax2b.set_facecolor(AXES_BG)
        aruco_colors = {'None': '#bdbdbd', 'Partial': '#fb8c00', 'All': '#2e7d32'}
        for state, yval in ARUCO_MAP.items():
            mask = df_ar['value'] == state
            subset = df_ar[mask]
            if not subset.empty:
                ax2b.scatter(to_numpy(subset['t']), [yval] * len(subset),
                             c=aruco_colors.get(state, '#999999'),
                             s=18, alpha=0.75, zorder=3, label=f'ArUco: {state}')
        ax2b.set_ylim(-0.3, 3.5)
        ax2b.set_yticks([0, 2, 3])
        ax2b.set_yticklabels(['None', 'Partial', 'All'], fontsize=9, color=TICK_C)
        ax2b.set_ylabel('ArUco visibility', color=LABEL_C, fontsize=9,
                        rotation=270, labelpad=15)
        for spine in ax2b.spines.values():
            spine.set_edgecolor('#aaaaaa')
        handles_l2, labels_l2 = ax2.get_legend_handles_labels()
        handles_r2, labels_r2 = ax2b.get_legend_handles_labels()
        ax2.legend(handles_l2 + handles_r2, labels_l2 + labels_r2,
                   loc='upper right', fontsize=8, framealpha=0.9, edgecolor='#cccccc')

    # ── Panel 3: Battery + USBL ────────────────────────────────────────────
    ax3 = axes[2]
    ax3.set_title('(c)  System State: Battery & USBL Strength',
                  color=TITLE_C, fontsize=10, pad=6, fontweight='bold', loc='left')

    if '/blueye/battery_percentage' in dfs:
        df_b = clip(dfs['/blueye/battery_percentage'])
        ax3.plot(to_numpy(df_b['t']), smooth(to_numpy(df_b['value'])),
                 color='#2e7d32', linewidth=2.0, label='Battery (%)')
        ax3.set_ylabel('Battery (%)', color=LABEL_C, fontsize=9)
        ax3.yaxis.set_label_coords(-0.07, 0.5)
    elif '/blueye/battery_level' in dfs:
        df_b = clip(dfs['/blueye/battery_level'])
        numeric = to_numpy(df_b['value'].map(BATTERY_LEVEL_MAP).fillna(50))
        ax3.step(to_numpy(df_b['t']), numeric, color='#2e7d32', linewidth=2.0,
                 where='post', label='Battery level')
        ax3.set_yticks([10, 30, 60, 90])
        ax3.set_yticklabels(['Critical', 'Low', 'Medium', 'High'], fontsize=8, color=TICK_C)
        ax3.set_ylabel('Battery level', color=LABEL_C, fontsize=9)
        ax3.yaxis.set_label_coords(-0.07, 0.5)

    if '/blueye/usbl_strength' in dfs:
        df_u = clip(dfs['/blueye/usbl_strength'])
        ax3b = ax3.twinx()
        ax3b.set_facecolor(AXES_BG)
        usbl_colors = {'Weak': '#ef9a9a', 'Moderate': '#ffcc80', 'Strong': '#90caf9'}
        for state, yval in USBL_MAP.items():
            mask = df_u['value'] == state
            subset = df_u[mask]
            if not subset.empty:
                ax3b.scatter(to_numpy(subset['t']), [yval] * len(subset),
                             c=usbl_colors.get(state, '#aaaaaa'),
                             s=18, alpha=0.75, zorder=3, label=f'USBL: {state}')
        ax3b.set_ylim(0.5, 3.5)
        ax3b.set_yticks([1, 2, 3])
        ax3b.set_yticklabels(['Weak', 'Moderate', 'Strong'], fontsize=9, color=TICK_C)
        ax3b.set_ylabel('USBL strength', color=LABEL_C, fontsize=9,
                        rotation=270, labelpad=15)
        for spine in ax3b.spines.values():
            spine.set_edgecolor('#aaaaaa')
        handles_l3, labels_l3 = ax3.get_legend_handles_labels()
        handles_r3, labels_r3 = ax3b.get_legend_handles_labels()
        ax3.legend(handles_l3 + handles_r3, labels_l3 + labels_r3,
                   loc='upper right', fontsize=8, framealpha=0.9, edgecolor='#cccccc')

    ax3.set_xlabel(t_label, color=LABEL_C, fontsize=9)

    # ── Hide intermediate x-tick labels ───────────────────────────────────
    for ax in axes[:-1]:
        ax.set_xticklabels([])

    # ── Set shared x limits ────────────────────────────────────────────────
    total_t = float(max(float(df['t'].max()) for df in dfs.values()))
    if t_end is None:
        t_end = total_t
    t_start_f = float(t_start)
    t_end_f   = float(t_end)
    for ax in axes:
        ax.set_xlim(t_start_f, t_end_f)

    # ── Suptitle ───────────────────────────────────────────────────────────
    fig.suptitle(
        'BLUE EYE AUV — Docking Phase BN Validation  |  H-SIA Framework',
        color=TITLE_C, fontsize=11, fontweight='bold', y=0.97
    )

    plt.savefig(output_path, dpi=200, bbox_inches='tight', facecolor='white')
    print(f"\n  ✓ Saved to: {output_path}")
    plt.close()

# ─── Main ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Docking BN validation visualizer')
    parser.add_argument('--bag', required=True,
                        help='Path to .db3 rosbag2 SQLite file')
    parser.add_argument('--output', default='docking_bn_validation.png',
                        help='Output PNG path')
    parser.add_argument('--full-mission', action='store_true',
                        help='Show full bag duration, not just docking phase')
    args = parser.parse_args()

    print(f"\nLoading bag: {args.bag}")
    dfs = load_bag(args.bag)

    if args.full_mission and '/mission_state' in dfs:
        dfs.pop('/mission_state')  # disables phase clipping

    print("\nGenerating figure...")
    make_figure(dfs, args.output)

if __name__ == '__main__':
    main()