# 🌊 Bio-Inspired Inchworm Robot for Offshore Wind Turbine Inspection

<div align="center">

[![Project Status](https://img.shields.io/badge/Status-Prototype%20Testing-yellow)](https://github.com/leanoLEE58/Bio-Inspired-Inchworm-Robot-for-Offshore-Wind-Turbine-Inspection)
[![University](https://img.shields.io/badge/University-Ocean%20University%20of%20China-blue)](http://www.ouc.edu.cn/)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)
[![Funding](https://img.shields.io/badge/Funding-National%20Innovation%20Program-orange)](https://github.com/leanoLEE58/Bio-Inspired-Inchworm-Robot-for-Offshore-Wind-Turbine-Inspection)

**A rigid-flexible coupled robotic system with vision-tactile fusion for autonomous inspection of confined spaces in offshore wind turbines**

[📺 Demo Video](#-demonstration-videos) | [🔧 Hardware](#-hardware-design) | [💻 Software](#-software-implementation) | [📊 Results](#-experimental-results)

---

### 🎬 System Overview

<!-- 替换为你上传的视频截图，点击可跳转视频 -->
![System Overview](https://github.com/user-attachments/assets/your-overview-image-id.png)

*Click image above to watch full demonstration video*

</div>

---

## 📖 Project Introduction

This project presents a **bio-inspired inchworm robot** designed for autonomous inspection of offshore wind turbine internal structures. The system addresses three critical challenges in wind turbine maintenance:
┌─────────────────────────────────────────────────────────────────┐
│ Challenge 1: "Cannot Enter" │
│ • 30%+ unreachable areas due to confined spaces (Ø < 40cm) │
├─────────────────────────────────────────────────────────────────┤
│ Challenge 2: "Cannot Detect Accurately" │
│ • 40% defect miss rate in low-light environments (≤100 lux) │
├─────────────────────────────────────────────────────────────────┤
│ Challenge 3: "Cannot Touch Safely" │
│ • Rigid robots damage fragile components (>50N contact force) │
└─────────────────────────────────────────────────────────────────┘

text


**Our Solution:**
- 🐛 **Inchworm Locomotion**: Bio-inspired "attach-extend-release" gait for Ø ≥ 25cm confined spaces
- 👁️ **Vision-Tactile Fusion**: YOLOv7 vision + 9DTact tactile sensors for robust defect detection
- 🦾 **Rigid-Flexible Coupling**: Adjustable stiffness (0.1-50 N·m/rad) pneumatic manipulator

---

## ✨ Key Features

### Quick Performance Overview

| Metric | Specification |
|--------|--------------|
| **Minimum Passage** | Ø 25cm (design) / Ø 30cm (current) |
| **Positioning Accuracy** | ±0.5mm (target) / ±1.2mm (achieved) |
| **Contact Force** | ≤5N (safe for fragile materials) |
| **Suction Force** | ≥20N per vacuum cup |
| **Vision Detection** | 0.1mm crack resolution @ >100 lux |
| **Tactile Resolution** | 0.01N @ 1kHz sampling rate |

---

## 🎥 Demonstration Videos

### Full System Demonstration

<!-- 上传后替换为实际的视频预览图和链接 -->
![Full Demo](https://github.com/user-attachments/assets/your-full-demo-thumbnail.png)

*3-minute comprehensive system overview (Upload your video and link here)*

---

### Inchworm Locomotion in Action

<!-- GIF 示例 - 上传后替换 -->
![Inchworm Motion](https://github.com/user-attachments/assets/your-inchworm-motion.gif)

*Bio-inspired gait cycle: Attach → Extend → Release*

---

### Vision-Tactile Detection Demo

<table>
<tr>
<td width="50%">

![Vision Detection](https://github.com/user-attachments/assets/your-vision-detection.gif)

*YOLOv7 real-time crack detection*

</td>
<td width="50%">

![Tactile Sensing](https://github.com/user-attachments/assets/your-tactile-heatmap.gif)

*Tactile pressure mapping & 3D reconstruction*

</td>
</tr>
</table>

---

## 🏗️ System Architecture

### Overall System Diagram
┌──────────────────────────────────────────────────────────────────┐
│ Control Terminal │
│ Xbox Controller / Upper Computer GUI │
└────────────────┬─────────────────────────────────────────────────┘
│ Bluetooth / WiFi
┌────────────────▼─────────────────────────────────────────────────┐
│ Main Control Unit (ESP32) │
│ ┌──────────────────────────────────────────────────────────┐ │
│ │ • Locomotion Control (Inchworm Gait FSM) │ │
│ │ • PWM → Voltage → Pressure Mapping │ │
│ │ • PID Closed-Loop Control │ │
│ │ • Multi-Modal Data Fusion (Vision + Tactile) │ │
│ └──────────────────────────────────────────────────────────┘ │
└────┬─────────┬──────────────┬──────────────┬────────────────────┘
│ │ │ │
┌────▼───┐ ┌──▼─────┐ ┌─────▼──────┐ ┌────▼──────┐
│ OAK-D │ │ 9DTact │ │ Pneumatic │ │ Vacuum │
│ Camera │ │ Sensor │ │ Valves │ │ Pump │
└────────┘ └────────┘ └──────┬─────┘ └───────────┘
│
┌─────▼──────┐
│ 6× PAM │
│ Actuators │
└────────────┘

text


### Modular Robot Structure

![Modular Structure](https://github.com/user-attachments/assets/your-modular-structure.png)

*Three-segment design: Front Anchor → Drive Section → Rear Anchor → End-Effector*

---

## 🔧 Hardware Design

### Core Components

| Category | Component | Model/Spec | Quantity |
|----------|-----------|-----------|----------|
| **Main Control** | Microcontroller | ESP32-DevKitC | 1 |
| **Vision** | Camera | OAK-D-Lite (4MP) | 1 |
| **Tactile** | Sensor | Custom 9DTact | 1 |
| **Actuation** | Proportional Valve | SMC ITV0010 | 3 |
| | Vacuum Pump | ZH10DSA-06-06-08 | 1 |
| | Servo Motor | MG996R (20kg·cm) | 2 |
| **Power** | Battery | Li-ion 7.4V 5000mAh | 1 |

### Prototype Photos

<table>
<tr>
<td><img src="https://github.com/user-attachments/assets/your-pam-muscles.jpg" width="250"><br><i>6× PAM Actuators</i></td>
<td><img src="https://github.com/user-attachments/assets/your-vacuum-system.jpg" width="250"><br><i>Vacuum Suction System</i></td>
<td><img src="https://github.com/user-attachments/assets/your-esp32-board.jpg" width="250"><br><i>ESP32 Control Board</i></td>
</tr>
<tr>
<td><img src="https://github.com/user-attachments/assets/your-flexible-gripper.jpg" width="250"><br><i>TPU Flexible Gripper</i></td>
<td><img src="https://github.com/user-attachments/assets/your-9dtact-sensor.jpg" width="250"><br><i>9DTact Tactile Sensor</i></td>
<td><img src="https://github.com/user-attachments/assets/your-full-assembly.jpg" width="250"><br><i>Full Assembly</i></td>
</tr>
</table>

---

## 💻 Software Implementation

### Control System Highlights

#### PWM-Pressure Calibration

![PWM Mapping](https://github.com/user-attachments/assets/your-pwm-calibration.png)

*Experimental calibration: PWM → Voltage → Pressure → PAM Length*

#### Inchworm Gait Control (Core Algorithm)

```cpp
// Simplified locomotion control
void inchwormGait(float stepSize_cm) {
    // Phase 1: Front anchor attachment
    frontVacuum.attach(-50);  // -50 kPa
    delay(500);
    
    // Phase 2: PAM extension
    for(int i = 0; i < 6; i++) {
        pam[i].setPressure(0.3);  // 0.3 MPa
    }
    
    // Phase 3: Rear release & contraction
    rearVacuum.release();
    for(int i = 0; i < 6; i++) {
        pam[i].setPressure(0.5);  // Max contraction
    }
    
    // Phase 4: Rear reattach & front release
    rearVacuum.attach(-50);
    frontVacuum.release();
}
Vision Processing Pipeline
YOLOv7 Crack Detection
Python

class CrackDetector:
    def detect(self, image):
        # Preprocessing
        img_tensor = self.preprocess(image)
        
        # Inference (on-board MyriadX chip)
        with torch.no_grad():
            pred = self.model(img_tensor)[0]
        
        # Post-processing
        detections = non_max_suppression(pred, conf_thres=0.5)
        
        # Extract crack features
        results = []
        for det in detections:
            crack_length_mm = self.calculate_size(det['bbox'])
            results.append({
                'bbox': det['bbox'],
                'confidence': det['conf'],
                'length_mm': crack_length_mm
            })
        
        return results
Detection Results
<table> <tr> <td width="50%">
Crack Detection 1

Surface crack detection (0.5mm width)

</td> <td width="50%">
Crack Detection 2

Multiple defect detection

</td> </tr> </table>
📊 Experimental Results
Performance Metrics Summary
Metric	Target	Achieved	Status
Min. Passage Diameter	Ø 25cm	Ø 30cm	🔧 In Progress
Positioning Accuracy	±0.5mm	±1.2mm	🔧 PID Tuning
Contact Force	≤5N	≤8N	🔧 Refinement
Suction Force	≥20N	22N ✅	✅ Met
Vision Detection (>100 lux)	90%	85%	⚠️ Dataset Expansion
Tactile Resolution	0.01N	0.015N	⚠️ Acceptable
Detection Accuracy by Environment
Lighting (lux)	Vision Only	Tactile Only	Fused (Ours)
>500	92%	78%	95% ✅
100-500	85%	82%	92% ✅
50-100	65%	85%	88% ✅
<50	35%	87%	85% ✅
Kinematic Model Validation
Simulation vs Real

MATLAB simulation vs. experimental bending angles (avg. error: 8.2mm)

🚀 Installation & Usage
Quick Start
1. Clone Repository
Bash

git clone https://github.com/leanoLEE58/Bio-Inspired-Inchworm-Robot-for-Offshore-Wind-Turbine-Inspection.git
cd Bio-Inspired-Inchworm-Robot-for-Offshore-Wind-Turbine-Inspection
2. Install Dependencies
Bash

# Python dependencies
pip install -r requirements.txt

# Arduino libraries (for ESP32)
arduino-cli lib install "ESP32Servo" "AsyncTCP" "ESPAsyncWebServer"
3. Flash ESP32 Firmware
Bash

cd firmware/esp32_controller
pio run --target upload
4. Run GUI
Bash

python gui/main_interface.py
Basic Operation
Python

from robot_controller import InchwormRobot

# Initialize robot
robot = InchwormRobot(port='/dev/ttyUSB0')
robot.connect()
robot.calibrate()

# Manual control
robot.set_mode('manual')
robot.move_forward(distance=10)  # Move 10cm

# Start inspection
robot.start_inspection(duration=60)  # Scan for 60 seconds
results = robot.get_latest_detections()

# Export report
robot.export_report('inspection_2025_01_20.json')
📂 Repository Structure
text

Bio-Inspired-Inchworm-Robot/
├── README.md                          # This file
├── LICENSE                            # MIT License
├── requirements.txt                   # Python dependencies
│
├── docs/                              # Documentation & media
│   ├── images/                        # Photos & diagrams
│   ├── videos/                        # Demo videos
│   └── assembly_guide.pdf             # Hardware assembly
│
├── firmware/                          # ESP32 embedded code
│   └── esp32_controller/
│       ├── main.ino
│       ├── locomotion_control.cpp
│       └── pid_controller.cpp
│
├── vision_module/                     # Computer vision
│   ├── crack_detector.py
│   ├── image_enhancement.py
│   ├── tactile_processor.py
│   └── weights/
│       └── yolov7-crack.pt
│
├── gui/                               # User interface
│   ├── main_interface.py
│   └── widgets/
│
├── scripts/                           # Utility scripts
│   ├── calibrate_pressure.py
│   └── test_pam_linearity.py
│
├── config/                            # Configuration
│   └── robot_params.yaml
│
└── mechanical/                        # CAD files
    ├── cad/                           # SolidWorks
    └── stl/                           # 3D printable
🔮 Future Work
Next Steps (6 Months)
 Reduce positioning error to ±0.5mm (optical encoder feedback)
 Expand YOLOv7 training dataset to 3,000+ images
 Implement ROS2 integration for modular control
 Field testing in simulated wind turbine environment
Long-Term Vision (1-2 Years)
 SLAM-based autonomous navigation
 Multi-robot collaborative inspection
 Underwater adaptation for offshore subsea structures
 Commercial prototype development
👥 Team
Core Contributors
<table> <tr> <td align="center" width="20%"> <b>Tian Xinrong<br>(Li Jiayi)</b><br> <i>Project Leader</i><br> Control & Vision<br> 📧 <a href="mailto:332323223@stu.ouc.edu.cn">Email</a> </td> <td align="center" width="20%"> <b>Lu Jingjing</b><br> <i>Co-Developer</i><br> Algorithms & Circuits </td> <td align="center" width="20%"> <b>Zhang Siyuan</b><br> <i>Mechanical Engineer</i><br> CAD & Fabrication </td> <td align="center" width="20%"> <b>Lu Guohang</b><br> <i>Sensor Specialist</i><br> Tactile & PCB </td> <td align="center" width="20%"> <b>Zheng Zhiyang</b><br> <i>Fabrication Lead</i><br> 3D Printing </td> </tr> </table>
Faculty Advisors
Prof. Chen Qi (陈琪) - College of Engineering, Ocean University of China
Dr. Zhu Jinchi (朱近赤) - Experimental Engineer, OUC
🙏 Acknowledgments
Funding
This project is supported by:

Ocean University of China - National Innovation Training Program (¥30,500)
National Natural Science Foundation of China - Grant No. 42204005
Shandong Provincial Natural Science Foundation - Grant No. ZR2022QF130
Open Source References
YOLOv7 - Object detection framework
OpenCV - Computer vision library
ESP32 Arduino Core - Embedded development
📄 License
This project is licensed under the MIT License - see the LICENSE file for details.

Note: Patent pending (CN202410XXXXX.X). For commercial use, please contact the team.

📞 Contact
Project Lead: Tian Xinrong (Li Jiayi)
Email: 332323223@stu.ouc.edu.cn
Institution: Ocean University of China
GitHub: @leanoLEE58

For technical questions or collaboration, please open an issue or contact us directly.

📚 Citation
If you use this work in your research, please cite:

bibtex

@misc{tian2025inchworm,
  title={Bio-Inspired Inchworm Robot for Offshore Wind Turbine Inspection},
  author={Tian, Xinrong and Lu, Jingjing and Zhang, Siyuan and Lu, Guohang and Zheng, Zhiyang},
  year={2025},
  institution={Ocean University of China},
  url={https://github.com/leanoLEE58/Bio-Inspired-Inchworm-Robot-for-Offshore-Wind-Turbine-Inspection}
}
<div align="center">
⭐ Star this repository if you find it interesting! ⭐
Contributions, issues, and feature requests are welcome!

Made with ❤️ by the Inchworm Robot Team @ Ocean University of China

🔝 Back to Top

</div> ```
