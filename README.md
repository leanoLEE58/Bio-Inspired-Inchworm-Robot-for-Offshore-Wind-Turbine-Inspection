# 🌊 Bio-Inspired Inchworm Robot for Offshore Wind Turbine Inspection

<div align="center">

[![Project Status](https://img.shields.io/badge/Status-Prototype%20Testing-yellow)](https://github.com/leanoLEE58/Bio-Inspired-Inchworm-Robot)
[![University](https://img.shields.io/badge/University-Ocean%20University%20of%20China-blue)](http://www.ouc.edu.cn/)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)
[![Funding Project Proposal](https://img.shields.io/badge/Funding-National%20Innovation%20Program-orange)](https://drive.google.com/file/d/1B0JYfbPkvaEYSYDNv414ZGevBeQjS8nl/view?usp=sharing)

**A Rigid-Flexible Coupled Robotic System with Vision-Tactile Fusion for Confined Space Inspection**

[🎬 Demos](#-demonstration-videos) · [📖 Documentation](#-table-of-contents) · [⚙️ Installation](#-installation-guide) · [💬 Discussions](https://github.com/leanoLEE58/Bio-Inspired-Inchworm-Robot/discussions)

---

### 🎥 System Demonstration

[![System Demo](https://img.youtube.com/vi/1U_8g-SPhKs/maxresdefault.jpg)](https://www.youtube.com/watch?v=1U_8g-SPhKs)

*Click to watch: Complete system demonstration including locomotion, detection, and manipulation*

---

</div>

## 📑 Table of Contents

<details open>
<summary><b>📖 Click to expand/collapse full navigation</b></summary>

### Main Sections

- [**Overview**](#-overview)
  - [Key Features](#key-features)
  - [Technical Specifications](#technical-specifications)
  - [System Highlights](#system-highlights)

- [**1. Introduction**](#1-introduction)
  - [1.1 Research Background](#11-research-background)
  - [1.2 Problem Statement](#12-problem-statement)
  - [1.3 Our Solution](#13-our-solution)

- [**2. System Design**](#2-system-design)
  - [2.1 Design Philosophy](#21-design-philosophy)
  - [2.2 Mechanical Architecture](#22-mechanical-architecture)
  - [2.3 Bio-Inspired Locomotion](#23-bio-inspired-locomotion)

- [**3. Implementation**](#3-implementation)
  - [3.1 Pneumatic System](#31-pneumatic-actuation-system)
  - [3.2 Electronic Control](#32-electronic-control-system)
  - [3.3 Simulation & Analysis](#33-computational-fluid-dynamics-analysis)

- [**4. Experimental Results**](#4-experimental-validation)
  - [4.1 PAM Development](#41-pneumatic-artificial-muscle-development)
  - [4.2 Rigid-Flexible Coupling](#42-rigid-flexible-coupled-manipulator)
  - [4.3 Kinematic Validation](#43-kinematic-model-validation)
  - [4.4 Integrated Performance](#44-integrated-system-performance)

- [**5. Performance Metrics**](#5-performance-metrics)
  - [5.1 Benchmark Comparison](#benchmark-comparison)
  - [5.2 Economic Impact Analysis](#economic--environmental-impact)

- [**Getting Started**](#-installation-guide)
  - [Quick Start](#quick-start)
  - [Hardware Setup](#hardware-requirements)
  - [Software Configuration](#software-installation)

- [**Additional Resources**](#additional-resources)
  - [Demonstration Videos](#-demonstration-videos)
  - [Repository Structure](#-repository-structure)
  - [Contributing](#-contributing)
  - [Team & Acknowledgments](#-team)
  - [Citation](#-citation)
  - [License](#-license)

</details>

---

## 🎯 Overview

This repository presents an **open-source bio-inspired inchworm robot** designed for autonomous inspection of offshore wind turbine internal structures. The system addresses three critical challenges in confined space maintenance through innovative integration of bionic locomotion, multi-modal sensing, and adaptive manipulation.

### Key Features

```
🐛 Bio-Inspired Locomotion    →  80% improvement in confined space accessibility
👁️  Vision-Tactile Fusion     →  92% defect detection in low-light (≤100 lux)
🦾 Rigid-Flexible Coupling    →  80% reduction in component damage risk
```

### Technical Specifications

| **Capability** | **Specification** | **Performance** |
|----------------|-------------------|-----------------|
| **Minimum Passage Diameter** | Ø 25 cm (design target) | Ø 30 cm (current prototype) |
| **Positioning Accuracy** | ±0.5 mm (target) | ±1.2 mm (achieved) |
| **Contact Force** | ≤5 N (safe threshold) | ≤8 N (measured) |
| **Vacuum Adhesion** | ≥20 N per cup | 22 N (verified) |
| **Vision Detection** | 0.1 mm crack resolution | @ >100 lux conditions |
| **Tactile Resolution** | 0.01 N @ 1 kHz | 0.015 N (current) |
| **System Response** | <50 ms (target) | 68 ms (measured) |

### System Highlights

- ✅ **Three-module design**: Front anchor + Pneumatic drive + Rear anchor
- ✅ **Six PAM actuators**: 40% contraction ratio, 0.01-0.5 MPa pressure range
- ✅ **Dual-modal sensing**: OAK-D-Lite camera + Custom 9DTact tactile array
- ✅ **ESP32 control**: Dual-core 240 MHz with real-time sensor fusion
- ✅ **Open-source**: Complete CAD files, firmware, and simulation tools

---

## 1. Introduction

### 1.1 Research Background

China's offshore wind power has reached **41.27 GW** installed capacity (49.6% global share), but maintenance operations face critical bottlenecks:

**Industry Challenges**:
- **Economic Loss**: ¥1+ billion annually from blade damage incidents
- **High Costs**: ¥200k-500k per manual inspection cycle
- **Safety Risks**: 70% of industry accidents occur during maintenance
- **Environmental Impact**: 1,500 tons CO₂ emissions per turbine downtime event

### 1.2 Problem Statement

Current inspection technologies encounter **three fundamental barriers**:

<div align="center">

| Challenge | Technical Limitation | Impact |
|-----------|---------------------|---------|
| **🚫 Inaccessibility** | Rigid robots cannot enter Ø<40 cm spaces | 30%+ undetected zones |
| **🚫 Detection Failure** | Vision-only systems fail at ≤100 lux | 40% miss rate |
| **🚫 Component Damage** | >50 N contact force harms fragile parts | High repair costs |

</div>

### 1.3 Our Solution

**Integrated approach combining three core innovations**:

```
Problem 1: Confined Space   →  Inchworm Gait Mechanism
Problem 2: Low-Light Detection  →  Vision-Tactile Fusion
Problem 3: Fragile Components   →  Rigid-Flexible Coupling
```

---

## 2. System Design

### 2.1 Design Philosophy

The research methodology follows a systematic problem-to-solution mapping:

<div align="center">

![Design Philosophy](https://github.com/user-attachments/assets/a6367d0b-82eb-4d06-82d8-e6f064404d16)

**Figure 1**: Research workflow from problem identification to practical implementation. The diagram illustrates: renewable energy significance → confined space challenges → manual inspection inefficiencies → bio-inspired solution → experimental validation → deployment strategy.

</div>

**Design Logic**:
```
Offshore Wind Energy (Critical renewable resource)
           ↓
Blade Internal Spaces (Ø<40 cm narrow cavities)
           ↓
Human Inspection Bottleneck (8-12 hours per turbine)
           ↓
Bionic Inchworm Solution (Flexible navigation + Safe interaction)
           ↓
Multi-Physics Validation (CFD + FEM + Experimental)
           ↓
Energy-Efficient Maintenance Platform
```

---

### 2.2 Mechanical Architecture

<div align="center">

![Mechanical Structure](https://github.com/user-attachments/assets/968b4d2b-defb-4ed7-a959-f69f5296b9e6)

**Figure 2**: Complete mechanical assembly (SolidWorks CAD model). The modular architecture comprises five functional units: (i) **Front Anchor Module** - vacuum suction array for temporary fixation, (ii) **Pneumatic Drive Section** - 6× McKibben PAM actuators for bidirectional motion, (iii) **Rear Anchor Module** - alternating support mechanism, (iv) **Vision-Tactile Module** - multi-modal sensing suite, (v) **Flexible End-Effector** - compliant manipulation with 2-DOF wrist.

</div>

**Module Specifications**:

| Module | Dimensions | Mass | Key Components | Primary Function |
|--------|-----------|------|----------------|------------------|
| **Front Anchor** | Ø80×L80 mm | 0.3 kg | 3× Silicone vacuum cups (Ø30 mm) | Temporary fixation (−50 kPa) |
| **Drive Section** | Ø60×L180-280 mm | 0.8 kg | 6× PAM actuators (Kevlar braided) | Locomotion (40% contraction) |
| **Rear Anchor** | Ø80×L80 mm | 0.3 kg | 3× Vacuum cups + pressure sensor | Alternating support |
| **Detection Module** | Ø50×L120 mm | 0.6 kg | OAK-D camera + 9DTact sensor | Multi-modal inspection |
| **End-Effector** | Ø40×L100 mm | 0.5 kg | TPU gripper (Shore 85A hardness) | Safe manipulation (≤5 N) |

**System Totals**: Length 460-580 mm (variable) | Mass 2.5 kg | Power 35.4 W (peak)

---

### 2.3 Bio-Inspired Locomotion

<div align="center">

<table>
<tr>
<td width="50%">

![Biological Inspiration](https://github.com/user-attachments/assets/cb64765f-ac2c-44ee-a680-f471d3398d18)

**(a) Natural inspiration**: *Geometridae* caterpillar demonstrating characteristic arched-body locomotion on narrow plant stems

</td>
<td width="50%">

![Engineered System](https://github.com/user-attachments/assets/4aef2ea8-a1ad-4712-bd28-3df4892e8c09)

**(b) Engineering implementation**: Three-segment robotic system replicating attach-extend-release gait cycle

</td>
</tr>
</table>

**Figure 3**: Biomimetic locomotion design. The inchworm's unique gait (optimized for narrow spaces over millions of years) is replicated through: **anterior/posterior vacuum arrays** (≥20 N adhesion per cup) and **central pneumatic drive** (6× symmetrical PAM achieving 40% contraction).

</div>

**Gait Cycle State Machine**:

| Phase | Description | Anchor Status | PAM Pressure | Duration |
|-------|-------------|---------------|--------------|----------|
| **1** | Front attachment | Front: ON, Rear: OFF | 0.01 MPa | 0.5 s |
| **2** | Body extension | Front: ON, Rear: OFF | 0.30 MPa | 2.0 s |
| **3** | Rear attachment | Front: ON, Rear: ON | 0.30 MPa | 0.5 s |
| **4** | Front release | Front: OFF, Rear: ON | 0.30 MPa | 0.3 s |
| **5** | Body contraction | Front: OFF, Rear: ON | 0.50 MPa | 2.0 s |
| **6** | Cycle reset | Front: ON, Rear: ON | 0.50 MPa | 0.5 s |

**Measured Performance**:
- Step displacement: 5-10 cm ± 0.5 cm
- Cycle time: ~10 seconds
- Energy efficiency: 0.8 J/cm
- Vertical wall climbing: Tested up to 90° inclination

---

## 3. Implementation

### 3.1 Pneumatic Actuation System

<div align="center">

![Pneumatic Platform](https://github.com/user-attachments/assets/0661a5ef-5ff1-40ef-a469-09a8243d766c)

**Figure 4**: Pneumatic actuation architecture. Compressed air (0.6 MPa source) flows through **MS6-LFR pressure regulator** to **manifold island**, distributing to **7× SMC ITV0030-3MS proportional valves**. Each valve independently controls one PAM actuator with ±0.001 MPa precision, enabling differential actuation for 3D spatial bending.

</div>

**Component Specifications**:

| Component | Model | Key Parameters | Function |
|-----------|-------|----------------|----------|
| **Proportional Valve** | SMC ITV0030-3MS | 0.001-0.5 MPa range, ±1% linearity | Precision pressure control |
| **Pressure Regulator** | MS6-LFR-1/2-D7 | 0.05-0.85 MPa, 1200 L/min flow | Source stabilization |
| **Vacuum Generator** | ZH10DSA-06-06-08 | −80 kPa max, 0.5 L/min | Suction generation |
| **PAM Actuator** | Custom McKibben | Ø12 mm, L=180 mm, 40% contraction | Linear actuation |

**Power Efficiency**:
- Control voltage: 10 V DC (low power signaling)
- Operating voltage: 24 V DC
- Pressure range: 0.001-0.5 MPa (500:1 dynamic range)
- Average power: 18.7 W during locomotion

---

### 3.2 Electronic Control System

<div align="center">

![Electronic Control](https://github.com/user-attachments/assets/12cd56fd-66e9-4a20-80b5-4898b8dd6e99)

**Figure 5**: Electronic control platform overview. **ESP32 dual-core MCU** (240 MHz Xtensa processors) serves as central controller, interfacing with proportional valves via **PWM-to-voltage conversion modules**. Real-time feedback from pressure sensors, angle encoders, and tactile arrays enables **closed-loop PID control** with 68 ms system response time.

</div>

---

<div align="center">

![Control Flowchart](https://github.com/user-attachments/assets/c531847c-ffe4-4847-bb7b-df483e64b9fc)

**Figure 6**: Control system workflow diagram. **Power management**: 3S LiPo battery (11.1V/5Ah) + Pololu S18V20F12 regulator provides stable 12V/2A supply. **Data processing**: ESP32 fuses multi-modal sensor data (vision, tactile, proprioceptive) with gamepad inputs. **Actuation pipeline**: Generates PWM signals (0-255) → voltage mapping (0-5V) → valve control → PAM pressure modulation. **Safety systems**: Overcurrent/overpressure protection + 2000 ms watchdog timer.

</div>

**Control Signal Chain**:
```
Xbox Controller Input
        ↓
ESP32 MCU Processing
        ↓
PWM Signal (0-255 range)
        ↓
Voltage Conversion (0-5 V linear mapping)
        ↓
SMC Proportional Valve Actuation
        ↓
Pneumatic Pressure Output (0-0.5 MPa)
        ↓
PAM Muscle Contraction (0-40% strain)
        ↓
Real-time Feedback (Pressure sensors + Encoders)
        ↓
PID Control Loop (Kp=1.2, Ki=0.05, Kd=0.3)
```

---

<div align="center">

![Circuit Schematic](https://github.com/user-attachments/assets/7726cb52-5d29-4282-bb10-d1fdd6b95341)

**Figure 7**: Complete circuit schematic design. **Key subsystems**: (i) Power distribution network with multi-rail voltage regulation (3.3V/5V/12V), (ii) ESP32 microcontroller with peripheral interfaces (UART/I2C/SPI), (iii) PWM-to-analog converter for valve control, (iv) Multi-channel sensor signal conditioning (pressure/angle/tactile), (v) Protection circuits (overcurrent detection, relay isolation, EMI filtering). PCB layout optimized for noise reduction with dedicated ground planes and decoupling capacitors.

</div>

**PCB Design Details**:
- Board size: 100 mm × 80 mm (2-layer FR-4)
- Signal isolation: Opto-couplers for high-current switching
- Protection: 5A fast-blow fuses + 10A relay modules
- Communication buses: UART (115200 baud), I2C (400 kHz)
- EMI mitigation: Ground plane + 0.1 µF decoupling caps

---

### 3.3 Computational Fluid Dynamics Analysis

**Problem Identification**: First-generation PAM prototype exhibited unpredictable bending behavior during actuation.

<div align="center">

<table>
<tr>
<td width="50%">

![Volume Fraction](https://github.com/user-attachments/assets/bef9400d-8a7b-4496-8b3b-6d762e7cd486)

**(a) Gas-phase volume fraction field**  
Two-phase flow interface shows **0.1-0.9 volume fraction gradient**, indicating severe inter-phase interaction. Stress fluctuations reach **±50% of baseline** due to velocity shear and local turbulence effects.

</td>
<td width="50%">

![Velocity Field](https://github.com/user-attachments/assets/609c54f7-0467-4fd7-a69c-3f7b7267f153)

**(b) Surface velocity distribution**  
Peak velocity **69.55×10⁶ m/s** in deceleration zones. Bottom region exhibits **streamline turbulence** and **vortex formation** (circled areas) causing asymmetric pressure field → non-uniform radial expansion → uncontrolled bending.

</td>
</tr>
</table>

**Figure 8**: ANSYS Fluent CFD simulation of PAM internal gas dynamics. Analysis reveals root cause of actuation instability: **non-uniform gas distribution** creates pressure differentials leading to unwanted bending moments. Maximum velocity gradients concentrate at tube-sleeve interface, triggering **vortex shedding** at Reynolds number Re ≈ 2,300 (transitional flow regime).

</div>

**Simulation Setup**:
- **Software**: ANSYS Fluent 2021 R1
- **Turbulence model**: k-ε realizable with enhanced wall treatment
- **Mesh**: 2.4 million tetrahedral elements (refined near walls)
- **Boundary conditions**: Inlet 0.3 MPa, outlet atmospheric pressure
- **Convergence**: Residuals <10⁻⁴ for continuity and momentum

**Critical Findings**:
1. **Pressure non-uniformity**: ±25% variation across tube cross-section
2. **Flow instability**: Vortex formation begins at Re ≈ 2,300
3. **Bending mechanism**: Asymmetric pressure → unbalanced radial forces → lateral deflection

---

## 4. Experimental Validation

### 4.1 Pneumatic Artificial Muscle Development

**Design Iteration 1** (Baseline): Standard braided sleeve + tube assembly
- ❌ Issues: Non-uniform contraction, gas leakage (>1%/hr), unpredictable bending

**CFD-Guided Redesign** (Based on Figure 8 analysis):

<div align="center">

<table>
<tr>
<td width="50%">

![Improved Fabrication](https://github.com/user-attachments/assets/fff81b19-700f-40bb-910d-d6a0b66224b4)

**(a) Second-generation fabrication process**  
**Key improvements**: (i) High-density elastic braided sleeve (eliminates flow turbulence), (ii) Multi-ring sealed end fittings (prevents leakage), (iii) Heat-shrink protective layers (structural reinforcement), (iv) Diameter-matched elastic tubing (uniform pressure distribution).

</td>
<td width="50%">

![Prototype Testing](https://github.com/user-attachments/assets/241e7c87-f5f1-4f1a-b362-d3f19d2260eb)

**(b) Validated prototype**  
**Test results**: Underwater leak test = 0% water ingress (1-hour submersion), Contraction uniformity = ±2% variance across 6× actuators, Fatigue life = >10,000 cycles @ 0.3 MPa.

</td>
</tr>
</table>

**Figure 9**: Optimized PAM design addressing CFD-identified deficiencies. Structural modifications eliminate internal flow irregularities, achieving **2× elongation improvement** (13 cm → 26 cm @ 58 kPa) compared to baseline.

</div>

---

**Material Selection Study**:

<div align="center">

![Material Comparison](https://github.com/user-attachments/assets/0fd85527-e0b5-4810-b741-dfbba6ffba19)

**Figure 10**: Systematic investigation of PAM performance factors. **Experimental matrix**: 3× tube materials (PE, silicone, latex) × 2× sleeve types (standard, stretchable). Test conditions: 58 kPa pressure, 20 cm initial length, 8 s inflation time. **Optimal configuration identified**: Latex tube + stretchable braided sleeve achieves **26 cm elongation** (130% of initial length), demonstrating superior elastic compliance and energy efficiency.

</div>

**Performance Comparison**:

| Material Combination | Elongation (cm) | Linearity (R²) | Cost | Durability | Selection |
|---------------------|-----------------|----------------|------|------------|-----------|
| PE + Standard Sleeve | 12 | 0.78 | $ | Good | ❌ |
| Silicone + Standard | 15 | 0.85 | $$ | Excellent | ⚠️ |
| **Latex + Stretchable** | **26** | **0.94** | **$$** | **Good** | **✅** |

**Key Metrics (Optimized Design)**:
- Elongation: **116% improvement** vs. baseline
- Response time: <2 s to 90% steady-state
- Fatigue resistance: >10,000 cycles @ 0.3 MPa
- Leakage rate: <0.1% per hour (pressure decay test)

---

### 4.2 Rigid-Flexible Coupled Manipulator

<div align="center">

<table>
<tr>
<td width="50%">

![MATLAB Simulation](https://github.com/user-attachments/assets/3885b189-44de-4784-a051-28d2879545f0)

**(a) MATLAB kinematic simulation**  
Three-PAM configuration (120° symmetry) enables omnidirectional bending. Color gradient represents strain distribution: **maximum at outer fiber** (ε_max = 15%), **minimum at neutral axis** (ε ≈ 0%).

</td>
<td width="50%">

![Physical Prototype](https://github.com/user-attachments/assets/c8fd3293-7220-48ad-997a-815ce664c77e)

**(b) Physical prototype demonstration**  
Achieves **180° bending range**. Hybrid design: **2-axis rigid wrist** (servo-driven, ±0.5 mm precision) + **6× PAM flexible section** (compliant interaction, ≤5 N contact force). Measured angular accuracy: **±3.2° vs. simulation**.

</td>
</tr>
</table>

**Figure 11**: Rigid-flexible coupled manipulator validation. The integrated architecture provides: (i) **Structural stability** from rigid wrist (supports ≥20 kg payload), (ii) **Adaptive compliance** from pneumatic muscles (safe interaction with fragile components). Experimental trajectory tracking confirms **<5% deviation** between simulated and actual bending paths.

</div>

**Stiffness Modulation Capability**:

| Operating Mode | PAM Pressure (MPa) | Tip Stiffness (N·m/rad) | Typical Application |
|---------------|-------------------|------------------------|---------------------|
| **Soft Mode** | 0.05 | 0.1 | Delicate surface contact, tactile scanning |
| **Medium Mode** | 0.25 | 5.2 | General manipulation, tool operation |
| **Rigid Mode** | 0.50 | 50.0 | High-precision positioning, load bearing |

**Adjustable stiffness range**: **0.1-50 N·m/rad (500:1 dynamic range)**

---

### 4.3 Kinematic Model Validation

<div align="center">

![Kinematic Validation](https://github.com/user-attachments/assets/bb3d3d49-629e-4041-900f-affbff5a35fb)

**Figure 12**: Forward kinematics experimental validation (MATLAB R2021b). Three test configurations: (i) **Config 1** - 45° bending (blue trajectory), (ii) **Config 2** - 90° bending (red trajectory), (iii) **Config 3** - 135° bending (green trajectory). Comparison between **theoretical model** (solid lines) and **experimental measurements** (discrete markers) yields average positional error of **8.2 mm** (1.4% of total arm length), confirming model fidelity for control implementation.

</div>

**Mathematical Framework**:

Based on geometric constraints of 120° symmetrical PAM arrangement:

**Orientation angle** (defines bending plane):
```math
\phi = \arctan\left(\frac{m_2 - m_3}{2m_1 + m_2 + m_3}\right) + \text{quadrant correction}
```

**Curvature radius** (quantifies bending sharpness):
```math
r = \frac{h}{\sqrt{m_1^2 + m_2^2 + m_3^2 - m_1m_2 - m_1m_3 - m_2m_3 + 3h^2}}
```

**Bending angle** (total angular displacement):
```math
\theta = \frac{1}{2r}\sqrt{m_1^2 + m_2^2 + m_3^2 - m_1m_2 - m_1m_3 - m_2m_3}
```

where $m_1, m_2, m_3$ are individual PAM lengths, and $h$ is structural height parameter.

**Validation Statistics**:

| Test Configuration | Simulated θ (°) | Measured θ (°) | Angular Error (%) | Position Error (mm) |
|-------------------|----------------|----------------|-------------------|---------------------|
| Config 1 (45°) | 45.2 | 43.8 | 3.1% | 6.4 |
| Config 2 (90°) | 90.5 | 87.3 | 3.5% | 8.7 |
| Config 3 (135°) | 135.3 | 130.6 | 3.5% | 9.5 |
| **Mean ± SD** | - | - | **3.4 ± 0.2%** | **8.2 ± 1.6 mm** |

**Conclusion**: Model accuracy sufficient for real-time trajectory planning and control.

---

### 4.4 Integrated System Performance

<div align="center">

![Research Workflow](https://github.com/user-attachments/assets/f967aca2-5eb8-4072-b650-fe501f810049)

**Figure 13**: Comprehensive research methodology integrating multi-physics simulation, iterative prototyping, and experimental validation. The workflow encompasses: (i) **CFD analysis** → identified flow instabilities causing actuation errors, (ii) **Material optimization** → achieved 2× performance gain through systematic testing, (iii) **FEM validation** → confirmed kinematic model accuracy (<5% error), (iv) **Multi-modal fusion** → demonstrated 92% detection rate in challenging environments, (v) **System integration** → verified complete functionality in simulated wind turbine cavity. Results confirm technical feasibility of bio-inspired approach for industrial confined space inspection.

</div>

---

## 5. Performance Metrics

### Benchmark Comparison

**Performance vs. State-of-the-Art Systems**:

| Metric | Traditional Rigid Robot | Pure Soft Robot | **Our Hybrid System** | **Advantage** |
|--------|------------------------|-----------------|----------------------|---------------|
| **Confined Space Access** | Ø ≥ 40 cm | Ø ≥ 30 cm | **Ø ≥ 25 cm** | **+60% vs. rigid** |
| **Positioning Accuracy** | ±0.2 mm ✅ | ±5 mm ❌ | **±1.2 mm** ✅ | **6× better than soft** |
| **Contact Force** | >50 N ❌ | <2 N ✅ | **≤8 N** ✅ | **84% reduction vs. rigid** |
| **Payload Capacity** | >50 kg | <5 kg | **≥20 kg** | **Balanced capability** |
| **Detection (Low-Light)** | 45% | N/A | **85%** | **+89% accuracy** |
| **Stiffness Range** | Fixed | 0.1 N·m/rad | **0.1-50 N·m/rad** | **500:1 tunability** |
| **Component Damage Risk** | High | Low | **Low** | **80% risk reduction** |

---

### Economic & Environmental Impact

**Case Study: Vineyard Wind 1 Offshore Project** (62 turbines × 13 MW)

<div align="center">

| Parameter | Traditional Method | **Our System** | **Improvement** |
|-----------|-------------------|---------------|-----------------|
| **Downtime per Inspection** | 10 days | **6 days** | **−40%** ⏱️ |
| **Economic Loss** | ¥800,000 | **¥480,000** | **¥320k saved** 💰 |
| **CO₂ Emissions** | 800 tons | **480 tons** | **320 tons reduced** 🌱 |
| **Annual Savings** (6 turbines/year) | - | - | **¥1.92M + 1,920 tons CO₂** |

</div>

**Scalability Analysis**:
- **China's offshore wind**: 45.3 GW installed capacity
- **Adoption scenario**: 60% market penetration
- **Annual CO₂ reduction**: ~**64,800 tons** (equivalent to removing **12,960 gasoline vehicles**)
- **Economic benefit**: ~**¥324 million/year** in reduced downtime losses

---

## 🎬 Demonstration Videos

<div align="center">

### Complete System Demonstration

[![Full System Demo](https://img.youtube.com/vi/1U_8g-SPhKs/maxresdefault.jpg)](https://www.youtube.com/watch?v=1U_8g-SPhKs)

**Video 1**: Simple demo of integrated system performance - Locomotion, multi-modal detection, and safe manipulation in simulated wind turbine blade cavity

---

### Inchworm Locomotion Mechanics

[![Locomotion Demo](https://img.youtube.com/vi/cCN1oI8rT0M/maxresdefault.jpg)](https://youtu.be/cCN1oI8rT0M)

**Video 2**: Introduce the project by prensentation.

</div>

---

## 🚀 Installation Guide

### Quick Start

```bash
# Clone repository
git clone https://github.com/leanoLEE58/Bio-Inspired-Inchworm-Robot.git
cd Bio-Inspired-Inchworm-Robot

# Install Python dependencies
pip install -r requirements.txt

# Flash ESP32 firmware
cd firmware/esp32_controller
pio run --target upload

# Launch control interface
python gui/main_interface.py
```

### Hardware Requirements

**Minimum Configuration**:
- ESP32 development board (or compatible)
- USB camera (for basic vision testing)
- Servo motors (2× for wrist simulation)
- Power supply (7.4V Li-ion or bench supply)

**Full System Configuration**:
- ESP32-WROOM-32D microcontroller
- OAK-D-Lite camera (with MyriadX VPU)
- 7× SMC ITV0030-3MS proportional valves
- 6× Custom PAM actuators (fabrication guide in `/docs`)
- Vacuum generator + 6× suction cups
- 3S LiPo battery (11.1V / 5000mAh)

### Software Installation

**Prerequisites**:
- Python 3.8+ (tested on 3.8, 3.9, 3.10)
- Arduino IDE 1.8.19+ or PlatformIO
- MATLAB R2020b+ (optional, for simulation only)

**Python Dependencies**:
```
torch>=1.10.0
opencv-python>=4.5.5
numpy>=1.21.0
scipy>=1.7.0
pyserial>=3.5
pyyaml>=6.0
depthai>=2.14.0  # For OAK-D camera
```

**Detailed setup instructions**: See [Installation Guide](docs/INSTALLATION.md)

---

## 📂 Repository Structure

```
Bio-Inspired-Inchworm-Robot/
├── README.md                          # This file
├── LICENSE                            # MIT License
├── requirements.txt                   # Python dependencies
│
├── docs/                              # Documentation
│   ├── images/                        # All figures from paper
│   │   ├── fig-2-1-design-philosophy.png
│   │   ├── fig-2-2-mechanical-structure.png
│   │   └── ...
│   ├── INSTALLATION.md                # Detailed setup guide
│   ├── HARDWARE.md                    # Hardware assembly instructions
│   └── API.md                         # Software API documentation
│
├── firmware/                          # Embedded software
│   └── esp32_controller/
│       ├── main.ino                   # Arduino main entry point
│       ├── locomotion_control.cpp     # Gait FSM implementation
│       ├── pid_controller.cpp         # Closed-loop pressure control
│       └── sensor_interface.cpp       # Multi-modal sensor drivers
│
├── vision_module/                     # Computer vision
│   ├── crack_detector.py              # YOLOv7-based detection
│   ├── low_light_enhance.py           # CLAHE enhancement
│   └── tactile_processor.py           # Pressure map analysis
│
├── gui/                               # User interface
│   └── main_interface.py              # PyQt5 control dashboard
│
├── simulation/                        # Analysis tools
│   ├── matlab/                        # Kinematic models
│   │   └── forward_kinematics.m
│   └── ansys/                         # CFD/FEM projects
│
├── mechanical/                        # CAD files
│   ├── solidworks/                    # Assembly & parts (.SLDPRT)
│   └── stl/                           # 3D printable files
│
└── scripts/                           # Utility tools
    ├── calibrate_pressure.py
    └── test_pam_linearity.py
```

---

## 🤝 Contributing

We welcome contributions from the robotics community! Areas for improvement:

- 🔧 Hardware optimization (lighter materials, smaller form factor)
- 💻 Software enhancements (ROS2 integration, SLAM navigation)
- 📊 Additional testing (different environments, failure analysis)
- 📖 Documentation (tutorials, troubleshooting guides)

**How to contribute**:
1. Fork the repository
2. Create feature branch (`git checkout -b feature/YourFeature`)
3. Commit changes (`git commit -m 'Add YourFeature'`)
4. Push to branch (`git push origin feature/YourFeature`)
5. Open Pull Request

**Code style**: Follow [Google C++ Style](https://google.github.io/styleguide/cppguide.html) for firmware, [PEP 8](https://pep8.org/) for Python.

---

## 👥 Team

### Core Contributors

<table>
<tr>
<td align="center" width="20%">
<b>Tian Xinrong 、Li Jiayi</b><br>
<i>Project Lead</i><br>
Control Systems & Vision<br>
📧 <a href="mailto:332323223@stu.ouc.edu.cn">Email</a>
</td>
<td align="center" width="20%">
<b>Lu Jingjing</b><br>
<i>Co-Developer</i><br>
Algorithms & Circuits
</td>
<td align="center" width="20%">
<b>Zhang Siyuan</b><br>
<i>Mechanical Engineer</i><br>
CAD & Fabrication
</td>
<td align="center" width="20%">
<b>Lu Guohang</b><br>
<i>Sensor Specialist</i><br>
Tactile & PCB Design
</td>
<td align="center" width="20%">
<b>Zheng Zhiyang</b><br>
<i>Manufacturing Lead</i><br>
3D Printing & Assembly
</td>
</tr>
</table>

### Faculty Advisors

**Prof. Chen Qi**  
College of Engineering, Ocean University of China  
📧 qichen@ouc.edu.cn  
*Research: Flexible structure dynamics, vibration control*

**Dr. Zhu Jinchi**  
Experimental Engineer, Ocean University of China  
📧 zjc@ouc.edu.cn  
*Guidance: Mechanical testing, experimental design*

---

## 🙏 Acknowledgments

**Funding Support**:
- Ocean University of China - National College Student Innovation Training Program (¥30,500)
- National Natural Science Foundation of China (Grant No. 42204005)
- Shandong Provincial Natural Science Foundation (Grant No. ZR2022QF130)

**Facilities & Resources**:
- OUC Robotics Laboratory (equipment access)
- OUC 3D Printing Center (rapid prototyping services)
- Prof. Chen Qi's research group (technical guidance)

**Open-Source Dependencies**:
- [ESP32 Arduino Core](https://github.com/espressif/arduino-esp32) - Embedded framework
- [OpenCV](https://opencv.org/) - Computer vision library
- [MATLAB](https://www.mathworks.com/) - Simulation platform
- [SolidWorks](https://www.solidworks.com/) - CAD design

---

## 📄 Citation

If you use this work in your research, please cite:

```bibtex
@misc{tian2025inchworm,
  title={Bio-Inspired Inchworm Robot for Offshore Wind Turbine Inspection: 
         A Rigid-Flexible Coupled Approach with Vision-Tactile Fusion},
  author={Tian, Xinrong and Li Jiayi and Lu, Jingjing and Zhang, Siyuan and 
          Lu, Guohang and Zheng, Zhiyang and Chen, Qi and Zhu, Jinchi},
  year={2025},
  institution={Ocean University of China},
  howpublished={\url{https://github.com/leanoLEE58/Bio-Inspired-Inchworm-Robot}},
  note={Open-source robotics project - National Innovation Program}
}
```

---

## 📜 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

**Patent Status**: Chinese patent application pending (CN202410XXXXX.X). For commercial inquiries, please contact the team.

---

## 📞 Contact

**Project Lead**: Tian Xinrong、Li Jiayi、Lu Jingjing、Lu Guohang、Zheng Zhiyang、Zhang Siyuan
**Affiliation**: College of Electronic Engineering, Ocean University of China  
**Email**: leanolee58@gmai.com、332323223@stu.ouc.edu.cn  
**GitHub**: [@leanoLEE58](https://github.com/leanoLEE58)

**For questions**:
- 🐛 Bug reports: [GitHub Issues](https://github.com/leanoLEE58/Bio-Inspired-Inchworm-Robot/issues)
- 💬 Technical discussions: [GitHub Discussions](https://github.com/leanoLEE58/Bio-Inspired-Inchworm-Robot/discussions)
- 📧 Collaboration: Email directly

---

<div align="center">

### ⭐ Star this repository if you find it useful! ⭐

**We're actively developing this project and welcome community feedback**

![GitHub stars](https://img.shields.io/github/stars/leanoLEE58/Bio-Inspired-Inchworm-Robot?style=social)
![GitHub forks](https://img.shields.io/github/forks/leanoLEE58/Bio-Inspired-Inchworm-Robot?style=social)

---

Made with ❤️ by the Inchworm Robot Team @ Ocean University of China

**Last Updated**: January 2025 | **Version**: 1.0.0-beta

[🔝 Back to Top](#-bio-inspired-inchworm-robot-for-offshore-wind-turbine-inspection)

</div>
