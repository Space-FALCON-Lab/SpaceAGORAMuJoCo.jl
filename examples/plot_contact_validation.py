"""Plot an existing report directory; requires matplotlib (no simulation is run)."""
import csv
import pathlib
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

root = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "contact-validation")
def rows(name):
    with (root / name).open() as stream:
        return list(csv.DictReader(stream))
def series(data, key):
    return [float(row[key]) for row in data]

fig, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
sliding = rows("sliding.csv")
t = series(sliding, "time_s")
power = series(sliding, "friction_power_W")
axes[0, 0].plot(t, power, color="#a23535", lw=1.5)
axes[0, 0].axhline(0, color="black", lw=.6)
axes[0, 0].fill_between(t, power, 0, where=[p > 0 for p in power], color="#a23535", alpha=.25)
axes[0, 0].set(xlim=(.04, .08), xlabel="Time (s)", ylabel="Tangential power (W)",
               title="Sliding: passive friction")
energy = series(sliding, "mechanical_energy_J")
work = [float(r["contact_work_J"]) + float(r["joint_damping_work_J"]) for r in sliding]
axes[0, 1].plot(t, [e-energy[0] for e in energy], label="Mechanical energy change", lw=2)
axes[0, 1].plot(t, work, "--", label="Contact + joint damping work", lw=1.5)
axes[0, 1].set(xlabel="Time (s)", ylabel="Energy / work (J)", title="Independent energy-work closure")
axes[0, 1].legend(fontsize=8)
for name in ("head_on", "sliding", "damped", "low_slip", "grazing"):
    data = rows(name + ".csv")
    e0 = float(data[0]["mechanical_energy_J"])
    residual = [(float(r["mechanical_energy_J"])-e0-float(r["contact_work_J"])-float(r["joint_damping_work_J"]))/1.3125 for r in data]
    axes[1, 0].plot(series(data, "time_s"), residual, label=name.replace("_", " "), lw=1)
axes[1, 0].set(xlabel="Time (s)", ylabel="Energy-work residual / 1.3125 J", title="Finest-step bookkeeping residuals")
axes[1, 0].legend(fontsize=8, ncol=2)
convergence = rows("convergence.csv")
for name in ("head_on", "sliding", "damped", "low_slip", "grazing"):
    data = [r for r in convergence if r["case"] == name and float(r["scaled_trajectory_error"]) != 0]
    axes[1, 1].loglog(series(data, "maximum_step_s"), series(data, "scaled_trajectory_error"), "o-", label=name.replace("_", " "), lw=1)
axes[1, 1].axhline(1e-3, color="black", ls=":", label="Acceptance target")
axes[1, 1].invert_xaxis()
axes[1, 1].set_xticks([2**-p for p in (10, 11, 12)], labels=[r"$2^{-10}$", r"$2^{-11}$", r"$2^{-12}$"])
axes[1, 1].xaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
axes[1, 1].set(xlabel="Core maximum step (s; finer →)", ylabel="Scaled trajectory error vs finest",
               title="Tight-tolerance floor; no assumed event order")
axes[1, 1].legend(fontsize=8, ncol=2)
if any(r["status"] == "failed" for r in convergence):
    axes[1, 1].text(.03, .03, "Gaps mark refused native solves", transform=axes[1, 1].transAxes, fontsize=8)
for ax in axes.flat:
    ax.grid(alpha=.2)
fig.suptitle("Issue #4: passive regularized friction validation", fontsize=13)
fig.savefig(root / "validation.png", dpi=160)
fig.savefig(root / "validation.svg")
