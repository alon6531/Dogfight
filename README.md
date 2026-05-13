Here is the README translated into English. You can copy and paste this content into your README.md file:

✈️ Autonomous Air-to-Air Combat Simulation
Developed by: Alon Shalitner

Final Project: Software Engineering Technician (May 2026)

📖 Project Abstract
This project presents the development of a complex air-to-air combat simulation in a 3D space. The system integrates advanced artificial intelligence technologies, aerodynamic physics, and flight control, allowing autonomous units (aircraft) to make tactical decisions in real-time, navigate dynamic environments, and perform interception, evasion, and patrol missions entirely independently.

🛠 Core Technologies & Algorithms
The project was developed using an Object-Oriented Programming (OOP) approach in C++ and incorporates the following algorithms and libraries:

Graphics and UI: Raylib (for real-time 3D rendering), Dear ImGui (for a live user interface), PL_MPEG (for video decoding).

Navigation and Pathfinding: * A + ATL (A with Landmarks): Used for calculating an efficient initial path based on an accurate heuristic function pre-calculated using Dijkstra's algorithm.

D Lite:* For dynamic path updating in real-time (adapting as the enemy aircraft changes position).

Flight Control and Physics: MPC (Model Predictive Control) for path smoothing, handling physical constraints (lift, drag, thrust, and weight forces), and optimal maneuvering.

Collision Detection: GJK (Gilbert-Johnson-Keerthi) for highly accurate proximity calculation and collision avoidance in a 3D space.

Behavior Management (AI): FSM (Finite State Machine) managing the strategic states of the aircraft (Takeoff, Patrol, Pursuit, Evasion, Return for Refuel).

💻 System Requirements & Environment
OS: Windows (Tested on Windows 11).

IDE: CLion / Project management via CMake.

Recommended Hardware: Modern CPU (e.g., AMD Ryzen 5), 16GB RAM.

🚀 Running Instructions
Ensure the executable file (main.exe) is located in the exact same directory as the Assets folder.

Run main.exe.

Main Menu: Choose your desired camera mode:

FIRST PERSON: Camera follows the aircraft from a chase perspective.

SPECTATE: Free camera mode fully controlled by the user.

After the introductory video finishes, the simulation will load and start running automatically.

🎮 User Guide & UI Control
During the simulation, a Heads-Up Display (HUD) will show real-time data about the aircraft: current FSM state, acting forces, ammo count, fuel level, and overall speed.

Green Line: Represents the planned trajectory/path of the aircraft.

TAB Key: Toggles the Debug display on/off:

Displays the navigation graph grid (red dots) and edges (darker color = higher weight/cost).

Opens a time control window (bottom right) allowing you to slow down or speed up the simulation speed (default is 1.0x).

In Evasion mode, a control window will open allowing you to tweak evasion parameters in real-time (each parameter has a tooltip explaining what it does).

ESC Key: Closes the simulation and exits the application.

🎯 The Lock System
The combat is decided by the Lock System:

A targeting box appears around each aircraft:

Green: No potential for a lock.

Orange: An advantage is required (such as an altitude advantage) to initiate a lock.

Red (LOCK): Full lock acquired.

When an aircraft has a chance to lock onto its opponent, a green gauge will start filling up under the targeted aircraft. Once full, the box turns red, displaying "LOCK", and the simulation ends.

End State: Depending on who locked onto whom, you will be transitioned to a Victory or Defeat screen. From there, you can choose to return to the home screen or rerun the simulation.
