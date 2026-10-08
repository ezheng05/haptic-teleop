"""
plot_force_bag.py - plot a haptic test recording, no ROS needed

Reads a rosbag2 (sqlite3) recorded with:
    ros2 bag record -o <name> /cbf/debug /phantom/force_feedback /cmd_vel_ref

and plots depth, commanded vs allowed velocity, and controller force vs the
force actually sent to the stylus.

usage:
    python3 plot_force_bag.py <bag_folder> [out.png] [--skip SECONDS]
"""
import glob, json, os, re, sqlite3, struct, sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PAT = re.compile(r"b=(-?[\d.]+) d=(-?[\d.]+) v=(-?[\d.]+)->(-?[\d.]+) "
                 r"F=\((-?[\d.]+),(-?[\d.]+)\) (\w+)")


def read(bag):
    db3 = glob.glob(os.path.join(bag, "*.db3"))[0]
    c = sqlite3.connect(db3).cursor()
    c.execute("SELECT id, name FROM topics")
    tid = {n: i for i, n in c.fetchall()}

    def rows(name):
        c.execute(f"SELECT timestamp, data FROM messages "
                  f"WHERE topic_id={tid[name]} ORDER BY timestamp")
        return c.fetchall()

    dbg_rows = rows("/cbf/debug")
    ff_rows = rows("/phantom/force_feedback")
    cv_rows = rows("/cmd_vel_ref")
    t0 = min(r[0][0] for r in (dbg_rows, ff_rows, cv_rows) if r)

    dbg = []
    for ts, blob in dbg_rows:                 # std_msgs/String
        n = struct.unpack_from("<I", blob, 4)[0]
        m = PAT.search(blob[8:8 + n - 1].decode())
        if m:
            dbg.append([(ts - t0) / 1e9] +
                       [float(m.group(k)) for k in range(1, 7)] +
                       [m.group(7) == "ACTIVE"])
    # omni_msgs/OmniFeedback: Vector3 force, Vector3 position (CDR, LE)
    ff = [[(ts - t0) / 1e9, *struct.unpack_from("<3d", blob, 4)]
          for ts, blob in ff_rows]
    # geometry_msgs/Twist: linear.x is the first double
    cv = [[(ts - t0) / 1e9, struct.unpack_from("<d", blob, 4)[0]]
          for ts, blob in cv_rows]
    return np.array(dbg, float), np.array(ff), np.array(cv)


def plot(dbg, ff, cv, out, skip):
    t, d, vs, Fc = dbg[:, 0], dbg[:, 2], dbg[:, 4], dbg[:, 5]
    act = dbg[:, 7].astype(bool)
    SC, INK, BL, GR, AM = "#CC0000", "#1A1A1A", "#1B6CA8", "#2E7D32", "#E07B39"

    fig, (a1, a2, a3) = plt.subplots(3, 1, figsize=(10, 7.2), sharex=True,
                                     gridspec_kw={"hspace": 0.12})

    def shade(ax):
        on = None
        for i, a in enumerate(act):
            if a and on is None:
                on = t[i]
            elif not a and on is not None:
                ax.axvspan(on, t[i], color="#FBE3E0", zorder=0)
                on = None
        if on is not None:
            ax.axvspan(on, t[-1], color="#FBE3E0", zorder=0)
        if skip:
            ax.axvspan(0, skip, color="#EEEEEE", zorder=0)

    for ax in (a1, a2, a3):
        shade(ax)

    a1.plot(t, d, color=INK, lw=2)
    a1.axhline(0.30, color=SC, ls=(0, (6, 4)), lw=1.6)
    a1.text(0.5, 0.31, "$r_{safe}$", color=SC, fontsize=11)
    a1.set_ylabel("depth from\ncamera (m)", fontsize=11)

    a2.plot(cv[:, 0], cv[:, 1], color=BL, lw=1.6, label="$u_{ref}$ from stylus")
    a2.plot(t, vs, color=GR, lw=2, label="$u^*$ allowed")
    a2.axhline(0, color="#bbb", lw=0.8)
    a2.set_ylabel("velocity\n(m/s)", fontsize=11)
    a2.legend(fontsize=10, frameon=False, loc="lower left", ncol=2)

    a3.plot(t, -Fc, color=INK, lw=1.6, ls="--", label="controller force (cbf_node)")
    a3.plot(ff[:, 0], -ff[:, 2], color=AM, lw=2.2, label="force sent to stylus")
    a3.set_ylabel("|F| (N)", fontsize=11)
    a3.set_xlabel("time (s)", fontsize=11)
    a3.legend(fontsize=10, frameon=False, loc="upper left")

    for ax in (a1, a2, a3):
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
        ax.set_xlim(0, t[-1])
    fig.savefig(out, dpi=170, bbox_inches="tight")


def summarise(dbg, ff, cv, skip):
    t, Fc = dbg[:, 0], -dbg[:, 5]
    act = dbg[:, 7].astype(bool) & (t > skip)
    fi = np.interp(t, ff[:, 0], -ff[:, 2])
    ui = np.interp(t, cv[:, 0], cv[:, 1])
    print(f"duration {t[-1]:.1f} s, active {act.sum()}/{len(t)} samples")
    if not act.any():
        return
    print(f"stylus/controller force ratio: "
          f"{np.median(fi[act] / np.maximum(Fc[act], 1e-6)):.2f}")
    print(f"peak force at stylus: {fi[act].max():.3f} N")
    print(f"u_ref while active: {ui[act].mean():.3f} +/- {ui[act].std():.3f} m/s")
    # each contiguous active stretch
    edges = np.flatnonzero(np.diff(np.r_[0, act.astype(int), 0]))
    for on, off in zip(edges[::2], edges[1::2]):
        seg = slice(on, off)
        k = on + int(np.argmax(fi[seg]))
        tail = fi[k:off]
        print(f"  {t[on]:5.1f}-{t[off - 1]:5.1f} s: ramp {t[k] - t[on]:.1f} s "
              f"to {fi[k]:.3f} N, then {tail.mean():.3f} +/- {tail.std():.3f} N")


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    skip = 0.0
    if "--skip" in sys.argv:
        skip = float(sys.argv[sys.argv.index("--skip") + 1])
        args = [a for a in args if a != str(sys.argv[sys.argv.index("--skip") + 1])]
    if not args:
        sys.exit(__doc__)
    bag = args[0]
    out = args[1] if len(args) > 1 else os.path.basename(bag.rstrip("/")) + ".png"
    dbg, ff, cv = read(bag)
    summarise(dbg, ff, cv, skip)
    plot(dbg, ff, cv, out, skip)
    print(f"wrote {out}")
