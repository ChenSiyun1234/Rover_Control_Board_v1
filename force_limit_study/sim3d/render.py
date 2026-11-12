"""Replay sim3d runs: MuJoCo views on the left, force and travel on the right; writes an mp4.

usage: python sim3d/render.py <repo> <run_dir> <out_dir> [segment ...]
Each run_dir/<name>.csv is a firmware trace (sim.trace columns) and run_dir/<name>_qpos.npz holds
time and qpos at 1 kHz. Nothing is simulated here; the views replay the stored joint positions.
"""
import os
import sys
from pathlib import Path

import imageio.v2 as imageio
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

REPO, RUNS, OUT = (Path(a) for a in sys.argv[1:4])
sys.path.insert(0, str(REPO))
from sim3d import scene  # noqa: E402

FPS = 20
MOVING, SEATED, FAULT = 2, 3, 5
SEGMENTS = {
    # name: (case, jam, title, seconds on screen)
    "arm_nominal_fsr_now": ("arm", False, "A. Board as built (1 MOhm pull-down): normal push", 6.0),
    "arm_jam_fsr_now": ("arm", True, "A. Board as built (1 MOhm pull-down): debris in the way", 6.0),
    "arm_nominal_fsr_fix": ("arm", False, "A. R26 changed to 2.2 kOhm: normal push", 6.0),
    "arm_jam_fsr_fix": ("arm", True, "A. R26 changed to 2.2 kOhm: debris in the way", 7.0),
    "rack_jam": ("rack", True, "B. Same channel, linear axis, 25 kg server: jam", 5.0),
    "arm_jam": ("arm", True, "A. Arm base pushes a drawer shut: debris in the way", 9.0),
    "arm_nominal": ("arm", False, "A. Arm base pushes a drawer shut: normal", 6.0),
    "rack_nominal": ("rack", False, "B. Same channel, linear axis, 25 kg server: normal", 5.0),
}
DEFAULT = ("arm_nominal_fsr_now", "arm_jam_fsr_now", "arm_nominal_fsr_fix", "arm_jam_fsr_fix",
           "rack_jam")
CAPTIONS = {
    "arm": "board channel 1: PWM1/DIR1 out on J7, encoder 1 in on J6,\n"
           "force sensor (FSR) in on J27 to PE10, 12-bit ADC",
    "rack": "same board channel; this bench keeps the earlier load-cell model",
}
BLUE, ORANGE, GREEN, RED, GRAY = "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "0.45"


def load_trace(path):
    with open(path, encoding="utf-8") as f:
        has_header = f.readline().lower().startswith("time")
    data = np.genfromtxt(path, delimiter=",", skip_header=int(has_header))
    return data


def window(trace):
    t, state = trace[:, 0], trace[:, 5]
    start = t[np.flatnonzero(state == MOVING)[0]]
    fault, seated = np.flatnonzero(state == FAULT), np.flatnonzero(state == SEATED)
    if len(fault):
        return start, t[fault[0]], min(t[-1], t[fault[0]] + 1.0)
    end = t[seated[0]] + 0.5 if len(seated) else t[-1]
    return start, None, min(t[-1], end)


def free_cam(lookat, distance, azimuth, elevation):
    cam = mujoco.MjvCamera()
    cam.lookat[:] = lookat
    cam.distance, cam.azimuth, cam.elevation = distance, azimuth, elevation
    return cam


def cameras(model, data, case):
    board = data.body("control_board").xpos.copy()
    if case == "arm":
        tool = data.body("arm_tool").xpos.copy()
        module = data.body("module").xpos.copy()
        return (free_cam(0.5 * (board + module) + [0.03, 0.02, 0.0], 2.4, 128, -20),
                free_cam(module + [-0.12, -0.05, 0.05], 0.85, 120, -15),
                free_cam(board + [0.008, 0, 0], 0.16, 180, -89.9))
    server = data.body("server").xpos.copy()
    carriage = data.body("axis_carriage").xpos.copy()
    return (free_cam(0.5 * (server + board) + [0, 0, 0.1], 3.0, 300, -20),
            free_cam(0.5 * (server + carriage), 0.9, 300, -18),
            free_cam(board + [0.008, 0, 0], 0.16, 180, -89.9))


def render_segment(name, writer):
    case, jam, title, seconds = SEGMENTS[name]
    trace = load_trace(RUNS / f"{name}.csv")
    stored = np.load(RUNS / f"{name}_qpos.npz")
    q_time, qpos = stored["time"], stored["qpos"]
    model, _ = scene.build(case, jam, look=os.environ.get("SIM3D_LOOK", scene.DEFAULT_LOOK))
    data = mujoco.MjData(model)
    t0, trip, t1 = window(trace)
    speed = (t1 - t0) / seconds
    show_arm = mujoco.MjvOption()  # arm and panel on, connector markers off
    show_arm.geomgroup[4], show_arm.geomgroup[5] = 1, 0
    hide_arm = mujoco.MjvOption()  # board view: arm off, connector markers on
    hide_arm.geomgroup[4], hide_arm.geomgroup[5] = 0, 1
    big = mujoco.Renderer(model, 440, 720)
    small = mujoco.Renderer(model, 280, 360)
    data.qpos[:] = qpos[np.searchsorted(q_time, t0)]
    mujoco.mj_forward(model, data)
    cam_all, cam_contact, cam_board = cameras(model, data, case)

    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    ax_all = fig.add_axes([0, 280 / 720, 720 / 1280, 440 / 720])
    ax_contact = fig.add_axes([0, 0, 360 / 1280, 280 / 720])
    ax_board = fig.add_axes([360 / 1280, 0, 360 / 1280, 280 / 720])
    for ax in (ax_all, ax_contact, ax_board):
        ax.axis("off")
    im_all = ax_all.imshow(np.zeros((440, 720, 3), np.uint8))
    im_contact = ax_contact.imshow(np.zeros((280, 360, 3), np.uint8))
    im_board = ax_board.imshow(np.zeros((280, 360, 3), np.uint8))
    ax_all.text(12, 28, title, fontsize=13, color="white", weight="bold",
                bbox=dict(facecolor="black", alpha=0.45, lw=0, pad=4))
    ax_all.text(12, 62, f"simulation, shown at {speed:.2g}x speed", fontsize=10, color="white",
                bbox=dict(facecolor="black", alpha=0.35, lw=0, pad=3))
    ax_contact.text(8, 22, "hand and drawer" if case == "arm" else "pusher and server", fontsize=10,
                    color="white", bbox=dict(facecolor="black", alpha=0.4, lw=0, pad=2))
    ax_board.text(8, 22, "control board (team KiCad model)", fontsize=10, color="white",
                  bbox=dict(facecolor="black", alpha=0.4, lw=0, pad=2))
    for i, (label, color) in enumerate((("motor 1 (J7)", "#ff7310"), ("encoder 1 (J6)", "#2680ff"),
                                        ("force sensor (J27)", "#1ad94d"))):
        ax_board.text(8 + 118 * i, 268, label, fontsize=8.5, color="black",
                      bbox=dict(facecolor=color, alpha=0.85, lw=0, pad=2))

    t = trace[:, 0]
    sel = (t >= t0) & (t <= t1)
    tt = t[sel] - t0
    ax_f = fig.add_axes([0.62, 0.55, 0.355, 0.40])
    ax_x = fig.add_axes([0.62, 0.13, 0.355, 0.33], sharex=ax_f)
    peak = max(trace[sel, 3].max(), trace[sel, 4].max(), trace[sel, 11].max())
    ax_f.set_ylim(-0.05 * peak, 1.15 * peak)
    ax_f.set_xlim(0, tt[-1])
    ax_f.set_ylabel("force (N)")
    ax_f.plot(tt, trace[sel, 11], color=GRAY, lw=1, ls="--", label="firmware limit")
    lf, = ax_f.plot([], [], color=BLUE, lw=3, alpha=0.55, label="force on the hand" if case == "arm" else "force on the pusher", zorder=3)
    lm, = ax_f.plot([], [], color=ORANGE, lw=1.0, drawstyle="steps-post", label="what the firmware reads",
                    zorder=4)
    ax_f.legend(fontsize=8, loc="upper left", frameon=False)
    ax_x.set_ylabel("travel (mm)")
    ax_x.set_xlabel("time (s)")
    lc, = ax_x.plot([], [], color=BLUE, lw=1.5, label="hand" if case == "arm" else "carriage")
    ls_, = ax_x.plot([], [], color=GREEN, lw=1.5, label="drawer" if case == "arm" else "server")
    lo = min(trace[sel, 1].min(), trace[sel, 2].min())
    hi = max(trace[sel, 1].max(), trace[sel, 2].max())
    ax_x.set_ylim(lo - 0.05 * (hi - lo) - 0.5, hi + 0.05 * (hi - lo) + 0.5)
    ax_x.legend(fontsize=8, loc="upper left", frameon=False)
    for ax in (ax_f, ax_x):
        ax.spines[["top", "right"]].set_visible(False)
        ax.tick_params(labelsize=8)
    trip_lines = [ax.axvline(-1, color=RED, lw=1) for ax in (ax_f, ax_x)]
    status = fig.text(0.62, 0.965, "", fontsize=11, color=RED, weight="bold")
    fig.text(0.62, 0.008, CAPTIONS[case], fontsize=10, color="0.2", linespacing=1.3)

    frames = int(seconds * FPS)
    for k in range(frames + 1):
        now = t0 + (t1 - t0) * k / frames
        data.qpos[:] = qpos[min(len(qpos) - 1, np.searchsorted(q_time, now))]
        mujoco.mj_forward(model, data)
        for renderer, cam, im in ((big, cam_all, im_all), (small, cam_contact, im_contact)):
            renderer.update_scene(data, camera=cam, scene_option=show_arm)
            im.set_data(renderer.render())
        if k == 0:
            small.update_scene(data, camera=cam_board, scene_option=hide_arm)
            im_board.set_data(small.render())
        n = int(np.searchsorted(tt, now - t0, side="right"))
        lf.set_data(tt[:n], trace[sel, 3][:n])
        lm.set_data(tt[:n], trace[sel, 4][:n])
        lc.set_data(tt[:n], trace[sel, 1][:n])
        ls_.set_data(tt[:n], trace[sel, 2][:n])
        state = trace[sel, 5][max(0, n - 1)]
        if trip is not None and now >= trip:
            for line in trip_lines:
                line.set_xdata([trip - t0] * 2)
            status.set_text("firmware tripped: brake, back off 2 mm")
        elif state == SEATED:
            status.set_color(GREEN)
            status.set_text("drawer closed" if case == "arm" else "seated")
        fig.canvas.draw()
        writer.append_data(np.asarray(fig.canvas.buffer_rgba())[..., :3].copy())
    plt.close(fig)
    big.close()
    small.close()
    print(name, f"window {t0:.3f}-{t1:.3f} s, speed {speed:.3f}x, trip {trip}")


OUT.mkdir(parents=True, exist_ok=True)
names = sys.argv[4:] or list(DEFAULT)
with imageio.get_writer(OUT / "interface_test.mp4", fps=FPS, macro_block_size=8) as writer:
    for name in names:
        render_segment(name, writer)
print("wrote", OUT / "interface_test.mp4")
