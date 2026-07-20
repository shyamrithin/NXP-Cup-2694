# FAQs

---
**FAQ1: What is the NXP Cup India 2026 challenge about?**

**Answer:** The NXP Cup 2026 – Autonomous Medical Response Challenge places participants in a realistic smart-city simulation where the NXP buggy acts as an autonomous emergency response vehicle. Participants must develop a complete autonomous solution capable of navigating city roads, interpreting traffic signs, identifying patients, communicating with a Municipality Server, and delivering patients to their assigned hospitals — all within a designated time limit.

---

**FAQ2: What hardware platform is used for the challenge?**

**Answer:** The challenge uses the **NXP MR-B3RB** buggy as the target hardware. It requires a forward-facing camera for QR code detection, building recognition, and sign recognition, and relies on sensors such as LIDAR, encoders, and IMU for localization, mapping, and navigation using Nav2.

---

**FAQ3: What simulation tool is used for development and testing?**

**Answer:** **Gazebo Simulator** is used for development and testing. It provides a B3RB model with all necessary sensors and packages such as Nav2, allowing participants to test their autonomous solution in a simulated city environment before the regional finale.

---

**FAQ4: What software framework does this project use?**

**Answer:** The project is based on **CogniPilot** (AIRY Release for B3RB), which is an open autopilot project. Participants can refer to the [CogniPilot AIRY Dev Guide](https://airy.cognipilot.org/) for details. The competition-specific logic is built inside the ROS2 Python package `b3rb_ros_line_follower`, which integrates into Cranium.

---

**FAQ5: How many buildings are in the simulation city, and what types are there?**

**Answer:** The city contains **8 mission-critical buildings** in total:
- **3 Patient Buildings**: PATIENT_1, PATIENT_2, PATIENT_3
- **3 Hospital Buildings**: HOSPITAL_1, HOSPITAL_2, HOSPITAL_3
- **2 Fake Hospital Buildings**: FAKE_HOSPITAL_1, FAKE_HOSPITAL_2

Each building is uniquely identified by a QR code.

---

**FAQ6: What is the role of QR codes in the challenge?**

**Answer:** QR codes are attached to every building (both patient and hospital buildings). Participants must use the onboard camera to detect and decode these QR codes. When near a patient building, scanning the QR code gives the patient identifier (e.g., `{LOC: PATIENT_1}`). When at a hospital, scanning verifies that the correct patient is being delivered to the correct hospital.

---

**FAQ7: How do road sign boards guide navigation?**

**Answer:** Sign boards are placed throughout the city and provide directional arrows (Left ←, Straight ↑, Right →) pointing toward specific buildings. The sign-to-building mapping is:
| Sign | Building |
|------|----------|
| A | PATIENT_1 |
| B | PATIENT_2 |
| C | PATIENT_3 |
| X | HOSPITAL_1 |
| Y | HOSPITAL_2 |
| Z | HOSPITAL_3 |

Participants must interpret these signs and take the correct turns at intersections.

---

**FAQ8: What is the Municipality Server and how does it work?**

**Answer:** The Municipality Server is an external service that assigns hospital destinations to patients. After scanning a patient's QR code and while still inside the **Patient Zone**, participants must publish the decoded patient ID to the Municipality Server. The server responds with the assigned hospital (e.g., PATIENT_1 → HOSPITAL_2). Leaving the Patient Zone before receiving the hospital assignment results in a **penalty**.

---

**FAQ9: What are Patient Zones and Hospital Zones?**

**Answer:** These are invisible boundary regions mapped to each building:
- **Patient Zone**: The buggy must be inside this zone when publishing the patient ID to the Municipality Server. Leaving before receiving the hospital assignment is penalized.
- **Hospital Zone**: The buggy must be inside this zone when sending the hospital QR scan to confirm delivery. Attempting to confirm delivery outside this zone results in a **penalty** and an INVALID response from the server.

---

**FAQ10: What is the complete mission flow?**

**Answer:** The mission flow is:
1. Start the buggy and enter the city
2. Follow lane discipline and navigate using sign boards
3. Find Patient 1 → Scan QR code
4. Contact Municipality Server from inside Patient Zone → Receive hospital assignment
5. Navigate to the assigned hospital
6. Scan hospital QR inside Hospital Zone → Confirm delivery
7. Repeat for Patient 2 and Patient 3
8. After delivering the 3rd patient → Mission complete
9. Bonus: Exit city, navigate to parking area, send "Parked" message to server within 1 minute

---

**FAQ11: How is the buggy supposed to follow lane discipline?**

**Answer:** The city is surrounded by a lane network represented as **white roads bordered with black lines**. Participants must use black line detection algorithms to keep the NXP Buggy within the white lane at all times. Crossing or touching the black lines counts as a **lane jump** and results in a penalty for each occurrence.

---

**FAQ12: What are the penalties in the challenge?**

**Answer:** The following actions result in negative points (penalties):
- **Collision**: Every collision with an obstacle or city infrastructure
- **Lane Jump**: Every instance of crossing/touching the black lane lines
- **Incorrect Hospital Delivery**: Delivering a patient to the wrong hospital
- **Fake Hospital Delivery**: Attempting delivery at FAKE_HOSPITAL_1 or FAKE_HOSPITAL_2
- **Leaving Patient Zone early**: Crossing the Patient Zone boundary before receiving the hospital assignment
- **Patient Death**: If the buggy is too slow and a patient's lifetime expires before reaching the hospital

---

**FAQ13: How is scoring calculated?**

**Answer:** Scoring includes both positive and negative components:
- **+ve Points**: Correct patient QR identification, successful Municipality Server communication, correct hospital delivery, mission completion bonus, and time-based percentile ranking
- **-ve Points**: Collisions, lane jumps, incorrect hospital deliveries, and fake hospital attempts
- **Bonus Points**: Successfully parking inside the designated parking zone after the third patient delivery
- The winner is the team with the **highest final score**. In case of a tie, the team with the **lower mission completion time** ranks higher.

---

**FAQ14: What is the bonus task and how are bonus points earned?**

**Answer:** After delivering the third patient, participants may attempt the bonus task:
1. Exit the city through the designated exit, which is located **in front of the final hospital delivery zone**
2. Navigate to the parking area
3. Send a **"PARKED"** message to the Municipality Server while inside the parking area **within 1 minute** of entering parking

Bonus points are awarded even if the buggy is still moving, as long as it is inside the parking area when the "PARKED" message is sent.

---

**FAQ15: What are the OS and environment requirements to set up the software stack?**

**Answer:** The required setup is:
- **OS**: Ubuntu 22.04.5 (fresh installation is recommended to avoid compatibility issues)
- **Internet**: Unrestricted personal internet (college/official Wi-Fi is not recommended)
- The setup involves installing **CogniPilot** using its universal installer, building the workspace using `build_workspace`, selecting the `airy` release and `b3rb` platform, and building Foxglove Studio.

---

**FAQ16: How do you install the CogniPilot environment?**

**Answer:** Run the following commands in a terminal:
```bash
sudo apt-get update
sudo apt-get install git wget -y
mkdir -p ~/cognipilot/installer
wget -O ~/cognipilot/installer/install_cognipilot.sh https://raw.githubusercontent.com/CogniPilot/helmet/main/install/install_cognipilot.sh
chmod a+x ~/cognipilot/installer/install_cognipilot.sh
/bin/bash ~/cognipilot/installer/install_cognipilot.sh
```
During setup, select `1` (airy) for release, `1` (native) for installer type, and `n` for SSH keys. Then run `build_workspace`, choose `n` for SSH keys and `1` (b3rb) for platform.

---

**FAQ17: Which Python modules are pre-approved for use in the solution?**

**Answer:** The following Python modules are pre-approved and will be available on the NXP evaluation laptop:
- torch==2.3.0, torchvision==0.18.0
- numpy==1.26.4
- opencv-python==4.11.0.86
- scipy==1.15.1, scikit-learn==1.5.2
- tk==0.1.0, pyzbar==0.1.9
- matplotlib==3.5.1, pyyaml==6.0.2
- tflite-runtime==2.14.0

Any additional Python module requires **written consent from the NXP CUP Technical Team** before it can be used.

---

**FAQ18: Which folder do participants need to modify and submit?**

**Answer:** Participants must modify and submit only the **`b3rb_ros_line_follower`** folder located at `~/cognipilot/cranium/src/b3rb_ros_line_follower`. This is the only folder participants are expected to edit. All challenge logic must be implemented within this folder.

---

**FAQ19: What are the submission rules and format?**

**Answer:** Submission requirements:
1. Create a folder named `NXP_CUP26_<your_team_id>` (e.g., `NXP_CUP26_3124`)
2. Place the `b3rb_ros_line_follower` folder inside it
3. Zip the folder — the zip file name must be `NXP_CUP26_<your_team_id>.zip`
4. The upload location will be communicated via the **Teams Channel**
5. No additional package installation will be allowed on the NXP evaluation laptop
6. NXP Team may also request **video submissions** if participant count exceeds a threshold — details will be shared via Teams Channel

---

**FAQ20: How do you run the simulation and individual ROS2 nodes?**

**Answer:** After building the workspace, run the following commands:
1. **Build and launch simulation**:
   ```bash
   cd ~/cognipilot/cranium/ && colcon build
   source ~/cognipilot/cranium/install/setup.bash
   ros2 launch b3rb_gz_bringup sil.launch.py world:=Raceway_1
   ```
2. **Run individual nodes** (each in a fresh terminal with `source setup.bash`):
   - Lane Vector Extractor: `ros2 run b3rb_ros_line_follower vectors`
   - Sign Board Classifier: `ros2 run b3rb_ros_line_follower detect`
   - QR Scanner Node: `ros2 run b3rb_ros_line_follower qr_detect`
   - Runner Node: `ros2 run b3rb_ros_line_follower runner`

Running all 4 nodes together makes the buggy move autonomously in the simulation.

---
