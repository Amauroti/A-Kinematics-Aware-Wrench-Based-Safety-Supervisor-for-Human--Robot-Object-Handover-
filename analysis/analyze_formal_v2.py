#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import csv
import math
import statistics

import matplotlib.pyplot as plt


ROOT = Path.home() / "handover_evaluation"
INPUT = ROOT / "analysis" / "formal_trials_v2.csv"
OUT = ROOT / "analysis"
PLOTS = ROOT / "plots"

OUT.mkdir(parents=True, exist_ok=True)
PLOTS.mkdir(parents=True, exist_ok=True)


def fnum(row, key):
    value = str(row.get(key, "")).strip()

    if value == "":
        return None

    try:
        return float(value)
    except ValueError:
        return None


def inum(row, key):
    value = str(row.get(key, "")).strip()

    if value == "":
        return None

    try:
        return int(float(value))
    except ValueError:
        return None


def safe_div(a, b):
    return a / b if b else float("nan")


with INPUT.open(newline="", encoding="utf-8") as f:
    rows = list(csv.DictReader(f))

if len(rows) != 14:
    raise RuntimeError(
        f"Expected 14 formal trials, found {len(rows)}"
    )

methods = ["baseline", "proposed"]

scenario_order = [
    "no_interaction",
    "weak_pull",
    "correct_pull",
    "strong_pull",
    "side_push",
    "reverse_pull",
    "out_of_zone_pull",
]


# ============================================================
# Method-level confusion metrics
# ============================================================

method_metrics = []

for method in methods:
    group = [
        r for r in rows
        if r["method"] == method
    ]

    counts = {
        "TP": 0,
        "TN": 0,
        "FP": 0,
        "FN": 0,
    }

    for r in group:
        cls = r["classification"]

        if cls in counts:
            counts[cls] += 1

    tp = counts["TP"]
    tn = counts["TN"]
    fp = counts["FP"]
    fn = counts["FN"]

    correct = tp + tn
    total = len(group)

    accuracy = safe_div(correct, total)
    precision = safe_div(tp, tp + fp)
    recall = safe_div(tp, tp + fn)
    specificity = safe_div(tn, tn + fp)
    f1 = safe_div(
        2 * precision * recall,
        precision + recall
    )

    balanced_accuracy = (
        recall + specificity
    ) / 2.0

    decision_means = [
        fnum(r, "decision_processing_mean_ms")
        for r in group
    ]
    decision_means = [
        x for x in decision_means
        if x is not None
    ]

    release_latencies = [
        fnum(r, "interaction_to_safe_release_ms")
        for r in group
        if inum(r, "actual_release") == 1
    ]
    release_latencies = [
        x for x in release_latencies
        if x is not None
    ]

    method_metrics.append({
        "method": method,
        "trials": total,
        "TP": tp,
        "TN": tn,
        "FP": fp,
        "FN": fn,
        "correct_decisions": correct,
        "accuracy": accuracy,
        "precision": precision,
        "recall_sensitivity": recall,
        "specificity": specificity,
        "f1_score": f1,
        "balanced_accuracy": balanced_accuracy,
        "mean_trial_decision_processing_ms":
            statistics.mean(decision_means),
        "mean_observed_release_latency_ms":
            statistics.mean(release_latencies)
            if release_latencies
            else "",
    })


metric_file = OUT / "formal_method_metrics_v2.csv"

with metric_file.open(
    "w",
    newline="",
    encoding="utf-8"
) as f:

    fields = list(method_metrics[0].keys())

    writer = csv.DictWriter(
        f,
        fieldnames=fields
    )

    writer.writeheader()
    writer.writerows(method_metrics)


# ============================================================
# Scenario comparison table
# ============================================================

scenario_rows = []

for scenario in scenario_order:

    baseline = next(
        r for r in rows
        if r["scenario"] == scenario
        and r["method"] == "baseline"
    )

    proposed = next(
        r for r in rows
        if r["scenario"] == scenario
        and r["method"] == "proposed"
    )

    scenario_rows.append({
        "scenario": scenario,
        "expected_release":
            baseline.get("expected_release", ""),
        "baseline_classification":
            baseline.get("classification", ""),
        "proposed_classification":
            proposed.get("classification", ""),
        "baseline_correct":
            baseline.get("correct_decision", ""),
        "proposed_correct":
            proposed.get("correct_decision", ""),
        "baseline_actual_release":
            baseline.get("actual_release", ""),
        "proposed_actual_release":
            proposed.get("actual_release", ""),
        "baseline_max_force_n":
            baseline.get("max_force_magnitude_n", ""),
        "proposed_max_force_n":
            proposed.get("max_force_magnitude_n", ""),
        "baseline_max_pull_n":
            baseline.get("max_pull_force_n", ""),
        "proposed_max_pull_n":
            proposed.get("max_pull_force_n", ""),
        "baseline_max_lateral_n":
            baseline.get("max_lateral_force_n", ""),
        "proposed_max_lateral_n":
            proposed.get("max_lateral_force_n", ""),
        "baseline_release_latency_ms":
            baseline.get(
                "interaction_to_safe_release_ms",
                ""
            ),
        "proposed_release_latency_ms":
            proposed.get(
                "interaction_to_safe_release_ms",
                ""
            ),
        "baseline_decision_processing_ms":
            baseline.get(
                "decision_processing_mean_ms",
                ""
            ),
        "proposed_decision_processing_ms":
            proposed.get(
                "decision_processing_mean_ms",
                ""
            ),
    })


scenario_file = OUT / "formal_scenario_comparison_v2.csv"

with scenario_file.open(
    "w",
    newline="",
    encoding="utf-8"
) as f:

    writer = csv.DictWriter(
        f,
        fieldnames=list(
            scenario_rows[0].keys()
        )
    )

    writer.writeheader()
    writer.writerows(scenario_rows)


# ============================================================
# Console summary
# ============================================================

print()
print("==============================================")
print("FORMAL EVALUATION V2")
print("==============================================")

for m in method_metrics:

    print()
    print(m["method"].upper())
    print("-" * 40)

    print(
        f"TP={m['TP']} "
        f"TN={m['TN']} "
        f"FP={m['FP']} "
        f"FN={m['FN']}"
    )

    print(
        "Correct decisions: "
        f"{m['correct_decisions']}/"
        f"{m['trials']}"
    )

    print(
        f"Accuracy: "
        f"{100*m['accuracy']:.1f}%"
    )

    print(
        f"Precision: "
        f"{m['precision']:.3f}"
    )

    print(
        f"Recall/Sensitivity: "
        f"{m['recall_sensitivity']:.3f}"
    )

    print(
        f"Specificity: "
        f"{m['specificity']:.3f}"
    )

    print(
        f"F1: "
        f"{m['f1_score']:.3f}"
    )

    print(
        f"Balanced accuracy: "
        f"{m['balanced_accuracy']:.3f}"
    )

    print(
        "Mean trial decision processing: "
        f"{m['mean_trial_decision_processing_ms']:.3f} ms"
    )

    latency = m[
        "mean_observed_release_latency_ms"
    ]

    if latency != "":
        print(
            "Mean observed release latency: "
            f"{latency:.1f} ms"
        )


# ============================================================
# Plot 1: Correctness by scenario
# ============================================================

baseline_correct = []
proposed_correct = []

for scenario in scenario_order:

    baseline = next(
        r for r in rows
        if r["scenario"] == scenario
        and r["method"] == "baseline"
    )

    proposed = next(
        r for r in rows
        if r["scenario"] == scenario
        and r["method"] == "proposed"
    )

    baseline_correct.append(
        int(baseline["correct_decision"])
    )

    proposed_correct.append(
        int(proposed["correct_decision"])
    )


x = list(range(len(scenario_order)))
width = 0.36

plt.figure(figsize=(11, 5))

plt.bar(
    [v - width / 2 for v in x],
    baseline_correct,
    width,
    label="Baseline"
)

plt.bar(
    [v + width / 2 for v in x],
    proposed_correct,
    width,
    label="Proposed"
)

plt.xticks(
    x,
    scenario_order,
    rotation=25,
    ha="right"
)

plt.yticks([0, 1], ["Incorrect", "Correct"])
plt.ylim(0, 1.15)

plt.ylabel("Release decision")
plt.title(
    "Decision correctness across controlled handover scenarios"
)

plt.legend()
plt.tight_layout()

plt.savefig(
    PLOTS / "formal_scenario_correctness_v2.png",
    dpi=300
)

plt.close()


# ============================================================
# Plot 2/3: confusion matrices
# ============================================================

def plot_confusion(method, tp, tn, fp, fn):

    matrix = [
        [tn, fp],
        [fn, tp],
    ]

    plt.figure(figsize=(5, 4.5))

    plt.imshow(matrix)

    plt.xticks(
        [0, 1],
        ["Predicted no release", "Predicted release"]
    )

    plt.yticks(
        [0, 1],
        ["Expected no release", "Expected release"]
    )

    for i in range(2):
        for j in range(2):
            plt.text(
                j,
                i,
                str(matrix[i][j]),
                ha="center",
                va="center",
                fontsize=16
            )

    plt.title(
        f"{method.capitalize()} confusion matrix"
    )

    plt.tight_layout()

    plt.savefig(
        PLOTS /
        f"formal_confusion_{method}_v2.png",
        dpi=300
    )

    plt.close()


for m in method_metrics:
    plot_confusion(
        m["method"],
        m["TP"],
        m["TN"],
        m["FP"],
        m["FN"],
    )


# ============================================================
# Plot 4: release latency for positive scenarios
# ============================================================

positive_scenarios = [
    "correct_pull",
    "strong_pull",
]

baseline_latency = []
proposed_latency = []

for scenario in positive_scenarios:

    b = next(
        r for r in rows
        if r["method"] == "baseline"
        and r["scenario"] == scenario
    )

    p = next(
        r for r in rows
        if r["method"] == "proposed"
        and r["scenario"] == scenario
    )

    baseline_latency.append(
        fnum(
            b,
            "interaction_to_safe_release_ms"
        )
    )

    proposed_latency.append(
        fnum(
            p,
            "interaction_to_safe_release_ms"
        )
    )


x = list(range(len(positive_scenarios)))

plt.figure(figsize=(7, 5))

plt.bar(
    [v - width / 2 for v in x],
    baseline_latency,
    width,
    label="Baseline"
)

plt.bar(
    [v + width / 2 for v in x],
    proposed_latency,
    width,
    label="Proposed"
)

plt.xticks(
    x,
    positive_scenarios
)

plt.ylabel(
    "Interaction to SAFE_RELEASE (ms)"
)

plt.title(
    "Observed release latency in valid release scenarios"
)

plt.legend()
plt.tight_layout()

plt.savefig(
    PLOTS / "formal_release_latency_v2.png",
    dpi=300
)

plt.close()


# ============================================================
# Plot 5: processing time by scenario
# ============================================================

baseline_processing = []
proposed_processing = []

for scenario in scenario_order:

    b = next(
        r for r in rows
        if r["method"] == "baseline"
        and r["scenario"] == scenario
    )

    p = next(
        r for r in rows
        if r["method"] == "proposed"
        and r["scenario"] == scenario
    )

    baseline_processing.append(
        fnum(
            b,
            "decision_processing_mean_ms"
        )
    )

    proposed_processing.append(
        fnum(
            p,
            "decision_processing_mean_ms"
        )
    )


x = list(range(len(scenario_order)))

plt.figure(figsize=(11, 5))

plt.bar(
    [v - width / 2 for v in x],
    baseline_processing,
    width,
    label="Baseline"
)

plt.bar(
    [v + width / 2 for v in x],
    proposed_processing,
    width,
    label="Proposed"
)

plt.xticks(
    x,
    scenario_order,
    rotation=25,
    ha="right"
)

plt.ylabel("Mean decision processing time (ms)")

plt.title(
    "Supervisor computation time by scenario"
)

plt.legend()
plt.tight_layout()

plt.savefig(
    PLOTS / "formal_decision_processing_v2.png",
    dpi=300
)

plt.close()


# ============================================================
# Text summary for later report use
# ============================================================

summary_txt = OUT / "formal_results_summary_v2.txt"

baseline = next(
    m for m in method_metrics
    if m["method"] == "baseline"
)

proposed = next(
    m for m in method_metrics
    if m["method"] == "proposed"
)

with summary_txt.open(
    "w",
    encoding="utf-8"
) as f:

    f.write(
        "FORMAL EVALUATION V2 - DESCRIPTIVE SUMMARY\n"
    )
    f.write(
        "==========================================\n\n"
    )

    f.write(
        "Evaluation design: "
        "7 scenarios x 2 methods x 1 retained trial "
        "= 14 controlled trials.\n\n"
    )

    f.write(
        "Baseline:\n"
        f"TP={baseline['TP']}, "
        f"TN={baseline['TN']}, "
        f"FP={baseline['FP']}, "
        f"FN={baseline['FN']}\n"
        f"Scenario-level decision accuracy="
        f"{100*baseline['accuracy']:.1f}%\n"
        f"Balanced accuracy="
        f"{baseline['balanced_accuracy']:.3f}\n"
        f"Mean trial decision processing="
        f"{baseline['mean_trial_decision_processing_ms']:.3f} ms\n\n"
    )

    f.write(
        "Proposed:\n"
        f"TP={proposed['TP']}, "
        f"TN={proposed['TN']}, "
        f"FP={proposed['FP']}, "
        f"FN={proposed['FN']}\n"
        f"Scenario-level decision accuracy="
        f"{100*proposed['accuracy']:.1f}%\n"
        f"Balanced accuracy="
        f"{proposed['balanced_accuracy']:.3f}\n"
        f"Mean trial decision processing="
        f"{proposed['mean_trial_decision_processing_ms']:.3f} ms\n\n"
    )

    f.write(
        "Key observed distinction:\n"
        "Baseline produced false-positive releases during "
        "side_push, reverse_pull and out_of_zone_pull.\n"
        "The proposed supervisor rejected all three "
        "of these unsafe interaction scenarios.\n\n"
    )

    f.write(
        "Important limitation:\n"
        "One retained trial was used per method-scenario condition. "
        "The results are therefore descriptive controlled-scenario "
        "observations and should not be presented as statistically "
        "significant population-level estimates.\n"
    )


print()
print("[OK] Wrote:")
print(metric_file)
print(scenario_file)
print(summary_txt)

print()
print("[OK] Plots:")
for p in sorted(PLOTS.glob("formal_*_v2.png")):
    print(p)

