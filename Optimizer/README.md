# Hip-Exo-Optimization — Human-in-the-Loop (HILO) Metabolic Optimizer

PC-side software that runs **human-in-the-loop optimization** of a hip-exoskeleton
torque profile using **CMA-ES**, driven by **real-time metabolic cost** measured
from a COSMED indirect-calorimetry system. The PC ranks candidate torque profiles,
updates the CMA-ES search distribution, and streams new parameters to the
exoskeleton controller over **Bluetooth Low Energy (BLE)**.

Everything in this directory runs on the **Windows host PC**. The embedded controller
that drives the motors runs on the exoskeleton's **STM32**, from the firmware in
[`../Controller/`](../Controller/) of this same repository. The two communicate only
over BLE: the PC sends a torque-profile parameter packet, and the exoskeleton applies it.

**Affiliation:** Harvard Slade Lab
**Authors:** Jae Choi, Yinkai Dong

---

## Installation

**Platform: Windows only.** 

**1. Get the code**
```bash
git clone https://github.com/Harvard-Slade-Lab/OpenHi-P-Exo.git
cd OpenHi-P-Exo/Optimizer
```

**2. Create the conda environment** (pinned in `metcma.yml`)
```bash
conda env create -f metcma.yml
conda activate metcma
```

**3. Install Tesseract OCR** (reads numbers off the Omnia screen). Download the
preconfigured package
([Google Drive](https://drive.google.com/file/d/17DEtPNnrKnunzDX3ql_moAN2LRDF_OS9/view?usp=sharing))
and extract it into this directory so that `Tesseract-OCR/tesseract.exe` exists —
`metabolics_extract.py` resolves this path relative to its own location.

For a full hardware run you also need a BLE adapter, the exoskeleton advertising as
`HIP_EXO_MAIN`, and a COSMED metabolic cart with the Omnia window visible on screen.

---

## Usage

```bash
python main.py
```

It will:

1. Prompt for **subject number** and **body weight (kg)**.
2. Connect to the exoskeleton over BLE (device name `HIP_EXO_MAIN`) and wait until
   connected.
3. Ask you to draw three OCR regions on the Omnia window — **Time, VO2, VCO2**.
4. Open a **PyQt5 + pyqtgraph** GUI with live plots and **Start / Stop / Skip**
   buttons, then run the CMA-ES optimization of the torque profile.

- **Start** begins the CMA-ES loop; **Stop** aborts the current condition and
  ramps torque to zero (BLE stays up); **Skip** discards the current condition.
- **Resuming:** if a subject's `cmaLogFile.csv` already exists, the run reloads the
  saved CMA-ES state and continues.
- **Simulation mode (no hardware):** set `metUse = False` (and `bleUse = False`) in
  `main.py` to run the full loop on synthetic breaths.

---

## Repository structure

```
OpenHi-P-Exo/Optimizer/
│
├── main.py                    # Entry point: wires BLE + OCR + controller + GUI
├── hiloMetCmaController.py    # Central coordinator (HILOMetCmaController)
├── cma_optimizer.py           # CMA-ES optimizer
├── metabolics_collect.py      # Metabolics collection loop
├── metabolics_extract.py      # Screen capture + OCR of the Omnia display
├── bleFunctions.py            # BLE client and torque-parameter protocol
├── serialPlotter.py           # Real-time visualization (PyQtGraph)
├── param_utils.py             # Torque-profile parameter handling
├── timer_manager.py           # Qt timer helper
├── metcma.yml                 # Pinned conda environment
├── (Tesseract-OCR/)           # Installed separately, see step 3
└── README.md
```

Runtime output is written under `outcmaes/subject_<N>/` (per-condition
CSV log, pickled CMA-ES state + backups, per-breath metabolics CSVs, and plot PNGs).
