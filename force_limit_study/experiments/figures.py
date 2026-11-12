"""Render the requested slide-size figures from reviewed experiment rows."""

import matplotlib

from sim.trace import TraceColumn
from sim.params import Params
from matplotlib.lines import Line2D

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

COLORS = ("#285b9b", "#b67819", "#c75e29", "#6b7834", "#ac5279", "#333333")
STYLES = ("-", "--", "-.", "-", ":", "--")
MARKERS = ("o", "s", "^", "D", "v", "x")


def save_figure(figure, path):
    """Save a reproducible PNG and release its matplotlib resources."""
    figure.savefig(path, dpi=150, metadata={"Software": "insertion-axis-sil"})
    plt.close(figure)


def make_figures(output, rows, nominal_trace, jam_traces, adc_order):
    """Write six figures with units and stable redundant sensor encodings."""
    plt.rcParams.update(
        {
            "font.size": 13,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "lines.linewidth": 2,
        }
    )
    figure, axes = plt.subplots(
        3,
        1,
        figsize=(12, 9),
        sharex=True,
        height_ratios=(2, 2, 0.65),
        layout="constrained",
    )
    time = nominal_trace[:, TraceColumn.TIME_S] - nominal_trace[0, TraceColumn.TIME_S]
    axes[0].plot(
        time,
        nominal_trace[:, TraceColumn.CARRIAGE_MM],
        label="Carriage",
        color=COLORS[0],
    )
    axes[0].plot(
        time,
        nominal_trace[:, TraceColumn.SERVER_MM],
        label="Server",
        color=COLORS[1],
        linestyle="--",
    )
    axes[0].set_ylabel("Position (mm)")
    axes[0].legend(loc="upper left", ncols=2)
    axes[1].plot(
        time, nominal_trace[:, TraceColumn.TRUE_FORCE_N], label="True", color=COLORS[0]
    )
    axes[1].plot(
        time,
        nominal_trace[:, TraceColumn.MEASURED_FORCE_N],
        label="Measured",
        color=COLORS[1],
        linestyle="--",
        linewidth=1,
        alpha=0.65,
    )
    axes[1].set_ylabel("Force (N)")
    axes[1].legend(loc="upper left", ncols=2)
    for state, label, color in (
        (0, "Disabled", "#dddddd"),
        (1, "Tare / idle", "#e8c77d"),
        (2, "Moving", "#a6bddb"),
        (3, "Seated", "#c2c9a4"),
    ):
        selected = np.flatnonzero(nominal_trace[:, TraceColumn.STATE] == state)
        if len(selected):
            left, right = time[selected[0]], time[selected[-1]]
            axes[2].axvspan(left, max(left + 0.015, right), color=color)
            if right - left > 0.08:
                axes[2].text((left + right) / 2, 0.5, label, ha="center", va="center")
    axes[2].annotate(
        "Seated",
        xy=(time[-1], 0.5),
        xytext=(-80, 0),
        textcoords="offset points",
        va="center",
        arrowprops={"arrowstyle": "->", "color": "#333333"},
    )
    axes[2].set_yticks([])
    axes[2].set_ylabel("State")
    axes[2].set_xlabel("Elapsed simulation time (s)")
    for axis in axes[:2]:
        axis.grid(alpha=0.2)
    save_figure(figure, output / "nominal.png")

    figure, axis = plt.subplots(figsize=(12, 7), layout="constrained")
    for index, adc in enumerate(adc_order):
        metrics, trace = jam_traces[adc]
        aligned_ms = (trace[:, TraceColumn.TIME_S] - metrics["contact_s"]) * 1000
        selected = (aligned_ms >= -20) & (aligned_ms <= 650)
        axis.plot(
            aligned_ms[selected],
            trace[selected, TraceColumn.TRUE_FORCE_N],
            label=adc,
            color=COLORS[index],
            linestyle="-",
        )
        if adc in ("ADS1220_1000", "HX711_10"):
            axis.plot(aligned_ms[selected], trace[selected, TraceColumn.RACK_FORCE_N],
                      color=COLORS[index], linestyle="--", linewidth=1)
    axis.axhline(
        60, color="#333333", linewidth=1, linestyle=":", label="Free limit: 60 N"
    )
    axis.set(xlabel="Time since first jam contact (ms)", ylabel="Contact force (N)")
    handles, labels = axis.get_legend_handles_labels()
    handles.extend([Line2D([0], [0], color="gray", linestyle="-"), Line2D([0], [0], color="gray", linestyle="--", linewidth=1)])
    axis.legend(handles, labels + ["Solid: pusher", "Dashed: rack"], ncols=2)
    axis.grid(alpha=0.2)
    save_figure(figure, output / "jam_adc.png")

    figure, axes = plt.subplots(1, 2, figsize=(15, 7), sharex=True, layout="constrained")
    for index, adc in enumerate(adc_order):
        selected = [row for row in rows["e1"] if row["adc"] == adc]
        for axis, field in zip(axes, ("peak_pusher_force_N", "peak_rack_force_N")):
            axis.plot([row["speed_mm_s"] for row in selected],
                      [row[field] for row in selected], label=adc, color=COLORS[index],
                      linestyle=STYLES[index], marker=MARKERS[index])
    speeds = np.array([25, 50, 100, 200])
    params = Params()
    axes[1].plot(speeds, speeds / 1000 * np.sqrt(params.k_jam * 1000 * params.server_mass),
                 color="gray", linewidth=1, label="Server-only momentum estimate")
    for axis, label in zip(axes, ("Peak pusher force (N)", "Peak rack force (N)")):
        axis.set(xlabel="Approach speed (mm/s)", ylabel=label)
        axis.set_xticks(speeds)
        axis.grid(alpha=0.2)
        axis.legend(fontsize=9)
    save_figure(figure, output / "speed_peak.png")

    figure, axis = plt.subplots(figsize=(12, 7), layout="constrained")
    for index, (adc, speed) in enumerate(
        ((adc, speed) for adc in ("HX711_80", "ADS1220_1000") for speed in (50, 100))
    ):
        selected = sorted(
            (
                row
                for row in rows["e2"]
                if row["adc"] == adc and row["speed_mm_s"] == speed
            ),
            key=lambda row: row["stiffness_N_mm"],
        )
        axis.plot(
            [row["stiffness_N_mm"] for row in selected],
            [row["peak_pusher_force_N"] for row in selected],
            label=f"{adc}, {speed} mm/s",
            color=COLORS[index],
            linestyle=STYLES[index],
            marker=MARKERS[index],
        )
    axis.set(
        xlabel="Pusher stiffness (N/mm)", ylabel="Peak pusher force (N)", xscale="log"
    )
    axis.set_xticks([5, 20, 50, 200], labels=["5", "20", "50", "200"])
    axis.legend()
    axis.grid(alpha=0.2)
    save_figure(figure, output / "compliance.png")

    figure, axes = plt.subplots(1, 3, figsize=(15, 6), layout="constrained")
    for axis, field, label in zip(
        axes,
        ("measured_trip_N", "peak_pusher_force_N", "backoff_force_N"),
        ("Measured trip force (N)", "Peak pusher force (N)", "Force after backoff (N)"),
    ):
        for enabled, shift, color, hatch, name in (
            (True, -0.18, COLORS[0], "", "Envelope"),
            (False, 0.18, COLORS[1], "//", "Global"),
        ):
            values = [row[field] for row in rows["e3"] if row["envelope"] == enabled]
            axis.bar(
                np.arange(3) + shift, values, 0.36, color=color, hatch=hatch, label=name
            )
        axis.set_xticks(range(3), ["50", "150", "250"])
        axis.set(xlabel="Jam position (mm)", ylabel=label, ylim=(0, None))
        axis.grid(axis="y", alpha=0.2)
    handles, labels = axes[0].get_legend_handles_labels()
    figure.legend(handles, labels, loc="outside upper center", ncols=2, frameon=False)
    save_figure(figure, output / "envelope.png")

    figure, axes = plt.subplots(1, 2, figsize=(12, 6), layout="constrained")
    labels = [
        "Off" if row["cutoff_Hz"] == 0 else str(row["cutoff_Hz"]) for row in rows["e4"]
    ]
    for axis, field, label in zip(
        axes,
        ("max_free_filtered_N", "jam_peak_N"),
        ("Observed free-travel maximum (N)", "Jam peak pusher force (N)"),
    ):
        axis.plot(
            range(len(labels)),
            [row[field] for row in rows["e4"]],
            color=COLORS[0],
            marker="o",
        )
        axis.set_xticks(range(len(labels)), labels)
        axis.set(xlabel="Force filter cutoff (Hz)", ylabel=label)
        axis.grid(alpha=0.2)
    save_figure(figure, output / "filter_tradeoff.png")
