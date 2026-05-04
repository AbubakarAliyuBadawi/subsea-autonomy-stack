#!/usr/bin/env python3
"""
Panel 1 only — P(mode | evidence) & Mode Recommendation
Starts at t=25s to skip the constant initialisation period.
Usage:
    python3 plot_panel1_only.py \
        --bag /path/to/docking_bn_mission_0.db3 \
        --output panel1_reliability.png
"""
import sqlite3
import struct
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.ndimage import uniform_filter1d
import warnings
warnings.filterwarnings('ignore')

# ── CDR deserializers ────────────────────────────────────────────────────────
def read_float32(data):
    try: return struct.unpack_from('<f', data, 4)[0]
    except: return None

def read_string(data):
    try:
        slen = struct.unpack_from('<I', data, 4)[0]
        return data[8:8+slen].decode('utf-8', errors='replace').rstrip('\x00')
    except: return None

def read_int32(data):
    try: return struct.unpack_from('<i', data, 4)[0]
    except: return None

TOPICS = {
    '/docking/mode_autonomous_prob': ('float32', read_float32),
    '/docking/mode_shared_prob':     ('float32', read_float32),
    '/docking/mode_human_prob':      ('float32', read_float32),
    '/docking/mode_recommendation':  ('string',  read_string),
    '/docking/reliability':          ('float32', read_float32),
    '/mission_state':                ('int32',   read_int32),
}

# ── Load bag ─────────────────────────────────────────────────────────────────
def load_bag(db_path):
    conn = sqlite3.connect(db_path)
    cur  = conn.cursor()
    cur.execute("SELECT id, name FROM topics")
    topic_map = {row[0]: row[1] for row in cur.fetchall()}
    wanted = {name: tid for tid, name in topic_map.items() if name in TOPICS}
    print("  Topics found:")
    for name in TOPICS:
        print(f"    {'✓' if name in wanted else '✗'}  {name}")
    raw_data = {name: [] for name in TOPICS}
    if not wanted:
        conn.close()
        raise RuntimeError("No matching topics found.")
    id_list = ','.join(str(i) for i in wanted.values())
    cur.execute(f"SELECT topic_id, timestamp, data FROM messages "
                f"WHERE topic_id IN ({id_list}) ORDER BY timestamp")
    for topic_id, ts, raw in cur.fetchall():
        name = topic_map[topic_id]
        _, deserializer = TOPICS[name]
        val = deserializer(raw)
        if val is not None:
            raw_data[name].append((ts, val))
    conn.close()
    dfs = {}
    t0 = None
    for name, rows in raw_data.items():
        if rows:
            df = pd.DataFrame(rows, columns=['ts', 'value'])
            if t0 is None:
                t0 = df['ts'].min()
            dfs[name] = df
    for name, df in dfs.items():
        df['t'] = (df['ts'] - t0) / 1e9
    total = max(float(df['t'].max()) for df in dfs.values())
    print(f"  Bag duration: {total:.1f} s")
    return dfs

# ── Helpers ──────────────────────────────────────────────────────────────────
def smooth(arr, window=12):
    arr = np.array(arr, dtype=float)
    return uniform_filter1d(arr, size=min(window, len(arr)))

def np_(series):
    return np.array(series, dtype=float)

# ── Plot ─────────────────────────────────────────────────────────────────────
def plot_panel1(dfs, output_path, t_clip_start=25.0):
    C_AUTO   = '#1565c0'
    C_SHARED = '#e65100'
    C_HUMAN  = '#558b2f'
    MODE_COLOR = {'Autonomous': C_AUTO, 'Shared': C_SHARED, 'Human': C_HUMAN}
    MODE_MAP   = {'Autonomous': 3, 'Shared': 2, 'Human': 1}
    GRID_C  = '#dddddd'
    TITLE_C = '#1a1a2e'
    LABEL_C = '#333333'
    TICK_C  = '#555555'

    def clip(df):
        return df[df['t'] >= t_clip_start].copy()

    t_end = float(max(float(df['t'].max()) for df in dfs.values()))

    fig, ax1 = plt.subplots(figsize=(12, 5))
    fig.patch.set_facecolor('white')
    ax1.set_facecolor('white')
    ax1.grid(True, color=GRID_C, linewidth=0.7, alpha=0.9, zorder=0)
    for spine in ax1.spines.values():
        spine.set_edgecolor('#aaaaaa')
    ax1.tick_params(colors=TICK_C, labelsize=9)

    # ── Three probability lines ───────────────────────────────────────────
    prob_topics = [
        ('/docking/mode_autonomous_prob', C_AUTO,   'P(Autonomous | evidence)'),
        ('/docking/mode_shared_prob',     C_SHARED, 'P(Shared | evidence)'),
        ('/docking/mode_human_prob',      C_HUMAN,  'P(Human | evidence)'),
    ]
    has_probs = any(t in dfs for t, _, _ in prob_topics)

    if has_probs:
        for topic, color, label in prob_topics:
            if topic in dfs:
                df_p = clip(dfs[topic])
                t_p = np_(df_p['t'])
                v_p = smooth(np_(df_p['value']))
                ax1.plot(t_p, v_p, color=color, linewidth=2.2, label=label, zorder=2)
                ax1.fill_between(t_p, v_p, alpha=0.07, color=color, zorder=1)
    elif '/docking/reliability' in dfs:
        df_r = clip(dfs['/docking/reliability'])
        t_r = np_(df_r['t'])
        v_r = smooth(np_(df_r['value']))
        ax1.plot(t_r, v_r, color=C_AUTO, linewidth=2.2,
                 label='P(Autonomous | evidence)', zorder=2)
        ax1.fill_between(t_r, v_r, alpha=0.07, color=C_AUTO, zorder=1)

    ax1.set_ylim(-0.02, 1.08)
    ax1.set_ylabel('Probability', color=LABEL_C, fontsize=10)
    ax1.set_xlabel('Mission time (s)', color=LABEL_C, fontsize=10)
    ax1.set_xlim(t_clip_start, t_end)

    # ── Mode recommendation — coloured background bands ───────────────────
    if '/docking/mode_recommendation' in dfs:
        df_m = clip(dfs['/docking/mode_recommendation'])
        t_rec = np_(df_m['t'])
        v_rec = list(df_m['value'])

        # Shade background per recommendation state
        band_alpha = 0.08
        band_colors = {'Autonomous': C_AUTO, 'Shared': C_SHARED, 'Human': C_HUMAN}
        for i in range(len(t_rec) - 1):
            state = v_rec[i]
            col = band_colors.get(state, '#aaaaaa')
            ax1.axvspan(t_rec[i], t_rec[i+1], alpha=band_alpha, color=col, zorder=0)

    # Legend — only the three probability lines
    ax1.legend(loc='center right', fontsize=9, framealpha=0.92,
               edgecolor='#cccccc', ncol=1)

    ax1.set_title('BN Output: P(mode | evidence) & Mode Recommendation',
                  color=TITLE_C, fontsize=11, fontweight='bold', loc='left', pad=8)
    fig.suptitle('BLUE EYE AUV — Docking Phase BN Validation  |  H-SIA Framework',
                 color=TITLE_C, fontsize=11, fontweight='bold', y=1.01)

    plt.tight_layout()
    plt.savefig(output_path, dpi=200, bbox_inches='tight', facecolor='white')
    print(f"\n  ✓ Saved: {output_path}")
    plt.close()

# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag',    required=True)
    parser.add_argument('--output', default='panel1_reliability.png')
    parser.add_argument('--t-start', type=float, default=25.0,
                        help='Start time in seconds (default: 25)')
    args = parser.parse_args()
    print(f"\nLoading: {args.bag}")
    dfs = load_bag(args.bag)
    print("\nGenerating Panel 1...")
    plot_panel1(dfs, args.output, t_clip_start=args.t_start)

if __name__ == '__main__':
    main()