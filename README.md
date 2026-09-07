# Kinematics-Aware Wrench-Based Safety Supervisor for Robot Handover

## MSc Robotics Dissertation Supplementary Material

This archive contains the source code, configuration files, controlled
experimental data, analysis scripts, and supporting results associated
with the dissertation project:

**Kinematics-Aware Wrench-Based Safety Supervisor for Robot Handover**

The system was developed and experimentally evaluated using a Franka
Emika collaborative robot and gripper.

---

## 1. Project Overview

The project investigates a safety supervisor for physical human-robot
object handover.

A traditional force-threshold baseline is compared against a proposed
supervisor that combines:

- estimated external wrench from the Franka robot;
- force magnitude;
- force component along the intended handover direction;
- force alignment;
- lateral-force constraints;
- end-effector kinematics;
- a predefined Cartesian handover zone;
- continuous temporal validation before release.

The objective is to reject unsafe or unintended interactions while
retaining valid object-transfer behaviour.

---

## 2. System Platform

### Robot

- Franka Emika collaborative robot
- Franka gripper

### Main software environment

- Ubuntu 20.04
- ROS Noetic
- libfranka / franka_ros
- MoveIt
- Python 3

Additional environment information is recorded in:

    software_environment.txt

---

## 3. System Architecture

The main handover sequence is:

    Visual object confirmation
              |
              v
        Robot pickup
              |
              v
          Safe home
              |
              v
        Handover pose
              |
              v
       Safety supervisor
              |
       +------+------+
       |             |
       v             v
    RELEASE        HOLD/ABORT
       |             |
       v             v
    Gripper      Object retained
     opens       / recovery

The baseline and proposed methods use the same robot task, gripper,
vision system, handover poses, and experimental infrastructure.

Only the release-decision supervisor differs between the two methods.

---

## 4. Baseline Supervisor

Main file:

    code/handover_safety/scripts/baseline_wrench_supervisor.py

The baseline uses a traditional force-magnitude threshold.

Release condition:

    external force magnitude > 2.0 N
    continuously for >= 0.50 s

The baseline does not use:

- intended pull direction;
- force alignment;
- lateral-force rejection;
- handover-zone kinematics.

---

## 5. Proposed Supervisor

Main file:

    code/handover_safety/scripts/kinematic_wrench_supervisor_experiment.py

The proposed supervisor combines wrench and kinematic constraints.

Primary release conditions:

    pull force > 2.0 N
    force alignment > 0.80
    lateral force < 4.0 N
    continuously for >= 0.50 s

The supervisor also evaluates whether the end effector is inside the
nominal handover region.

Nominal handover-zone centre, expressed using the Franka end-effector
position representation used by the supervisor:

    x =  0.336838 m
    y = -0.448229 m
    z =  0.197560 m

Zone margins:

    x: +/- 0.08 m
    y: +/- 0.08 m
    z: +/- 0.08 m

A force interaction above threshold outside the valid handover zone is
rejected.

---

## 6. Out-of-Zone Test

The out-of-zone experiment uses:

    handover_pose_out_of_zone

The experimental pose was generated using MoveIt from the nominal
handover pose with an approximately +0.11 m translation along the
robot-base Z direction.

Measured planned displacement:

    delta_z = +0.109909 m

This exceeds the configured 0.08 m handover-zone margin.

The generation procedure is implemented in:

    code/handover_safety/scripts/prepare_out_of_zone_pose.py

The corresponding joint configuration is stored in:

    code/handover_safety/config/saved_poses.yaml

---

## 7. Main Source Files

### Core safety algorithms

    baseline_wrench_supervisor.py

Traditional force-magnitude baseline.

    kinematic_wrench_supervisor_experiment.py

Proposed kinematics-aware wrench-based supervisor.

    safety_supervisor.py

Executes physical gripper release following SAFE_RELEASE.

### Robot task control

    handover_task_manager_experiment.py

Coordinates pickup, handover, supervisor execution, release, timeout,
and experimental recovery behaviour.

    recover_franka_reflex.py

Handles appropriate Franka reflex recovery through the Franka error
recovery interface.

### Perception and user command

    open_vocab_vision_node.py

Provides box detection and stable target confirmation.

    voice_handover_trigger_experiment.py

Provides the offline voice command interface. The task is triggered by
the phrase:

    need box

### Experimental infrastructure

    experiment_trial_logger.py
    start_handover_trial.sh
    stop_handover_trial.sh

These files record experiment metadata, time-series data, safety-state
transitions, release outcomes, force features and computation timing.

### System startup

    start_handover_experiment_system.sh
    stop_handover_experiment_system.sh

These scripts start and stop the integrated experimental system.

---

## 8. Starting the Experimental System

### Baseline

    rosrun handover_safety start_handover_experiment_system.sh \
        baseline execute:=true

### Proposed supervisor

    rosrun handover_safety start_handover_experiment_system.sh \
        proposed execute:=true

The robot, network, Franka Control Interface, ROS controllers and MoveIt
must be correctly configured before execution.

These commands are provided for reproducibility and are not intended to
replace the required robot safety procedures or laboratory training.

---

## 9. Controlled Evaluation

The final retained evaluation consists of:

    7 scenarios
    x 2 methods
    x 1 retained controlled trial
    = 14 primary trials

The seven scenarios are:

1. no_interaction
2. weak_pull
3. correct_pull
4. strong_pull
5. side_push
6. reverse_pull
7. out_of_zone_pull

Expected release:

    correct_pull    -> release
    strong_pull     -> release

All other retained scenarios:

    -> no release

The accidental_contact scenario was removed from the primary evaluation
because manually reproducing both transient force magnitude and contact
duration sufficiently consistently was impractical.

The final trial-selection record is:

    experiment_protocol/formal_trial_selection_v2.csv

The final protocol amendment is:

    experiment_protocol/formal_evaluation_v2_final_amendment.txt

---

## 10. Primary Controlled Results

Observed retained trial classifications:

                          Baseline     Proposed
    no_interaction           TN           TN
    weak_pull                TN           TN
    correct_pull             TP           TP
    strong_pull              TP           TP
    side_push                FP           TN
    reverse_pull             FP           TN
    out_of_zone_pull         FP           TN

Baseline:

    TP = 2
    TN = 2
    FP = 3
    FN = 0

    Correct scenario decisions = 4 / 7

Proposed:

    TP = 2
    TN = 5
    FP = 0
    FN = 0

    Correct scenario decisions = 7 / 7

These results are descriptive controlled-scenario observations.

Only one retained trial is used for each method-scenario condition.
Consequently, the results must not be interpreted as population-level
success probabilities or statistical significance between methods.

---

## 11. Experimental Data

### Trial summaries

    results/trial_summaries/

Contains one summary CSV for each of the 14 retained trials.

### Time-series data

    results/selected_timeseries/

Contains the recorded time-series data associated with the retained
controlled trials.

Recorded variables include, where applicable:

- safety state;
- force magnitude;
- pull-force component;
- lateral force;
- force alignment;
- end-effector position;
- handover-zone state;
- decision-processing time;
- experiment state.

### Trial metadata

    results/trial_metadata/

Contains metadata associated with each retained experiment.

Original ROS bag files are not included in this archive because of the
supplementary-file size limit. They were retained locally during project
development.

---

## 12. Analysis

The principal analysis script is:

    analysis/analyze_formal_v2.py

Primary derived files include:

    analysis/formal_trials_v2.csv
    analysis/formal_method_metrics_v2.csv
    analysis/formal_scenario_comparison_v2.csv
    analysis/formal_results_summary_v2.txt

The analysis is intended to be reproducible directly from the included
experiment summaries.

---

## 13. Figures

Generated analysis figures are located in:

    figures/

These include:

- scenario-level decision comparison;
- baseline confusion matrix;
- proposed confusion matrix;
- observed release latency;
- decision-processing time.

---

## 14. External Dependencies

Large third-party pretrained models and complete external software
packages are not redistributed in this archive.

Examples include:

- Ultralytics YOLO-World pretrained model;
- Vosk speech-recognition model;
- ROS Noetic;
- MoveIt;
- libfranka / franka_ros.

These components are external dependencies and are not claimed as
original project contributions.

---

## 15. Scope of Original Contribution

The submitted work includes the integration and experimental evaluation
of a robot handover system centred on a kinematics-aware wrench-based
safety supervisor.

The principal project contributions represented in this archive include:

- formulation and implementation of the proposed supervisor;
- integration of force-direction and lateral-force constraints;
- integration of end-effector handover-zone constraints;
- baseline implementation for controlled comparison;
- safety-state and release logic;
- integration with the Franka handover task;
- experimental task and data-recording infrastructure;
- controlled scenario design;
- recorded physical robot experiments;
- analysis and interpretation of the retained data.

Third-party robotics frameworks and pretrained perception/speech models
are used as supporting infrastructure.

---

## 16. Known Evaluation Limitation

The primary comparison contains one retained controlled trial per
method-scenario condition.

The evaluation therefore provides functional and descriptive evidence
of system behaviour under the tested conditions, but it does not provide
statistical evidence of repeatability across participants, repeated
trials, object variations or broader operating conditions.

Repeated trials and multi-participant evaluation are appropriate future
extensions.

---

## 17. Academic Integrity and AI Use

AI-use information is provided separately in:

    AI_USE_DECLARATION.txt

All project materials should be interpreted together with the formal
AI-use declaration required by the dissertation unit and University.

