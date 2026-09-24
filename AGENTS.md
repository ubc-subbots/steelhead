# Agent Knowledge Base: Steelhead AUV

This file contains critical context, rules, and learnings for AI agents working in the `steelhead` repository.

**ATTENTION ALL AGENTS:** You MUST read and follow these rules before making any codebase modifications.

## 1. Core Rules & AI Policy
* **ROS 2 Version:** We are using **ROS 2 Lyrical** on Ubuntu 26.04.
* **Clean Artifacts:** Never leave behind temporary scratch scripts (like `fix_plugin.py`), dummy comments, or conversational AI placeholders in the codebase (e.g., `// Add your code here`). If you create scratch scripts, delete them before finishing your turn.
* **Documentation:** If you are changing functionality of a package, make sure to check its README.md to see if the documentation needs to be updated. Similarly, creating a new package requires a README.md to be created describing its functionality. Use steelhead_controls README.md as a basis.
* **Accountability:** AI is infamous for hallucinating outdated ROS 2 or Gazebo Classic code. Always double-check API versions and documentation before applying fixes. Do not introduce breaking changes to APIs without verifying they exist in ROS 2 Lyrical or Gazebo Jetty.

## 2. Gazebo Jetty & ROS 2 Migration Gotchas
The physics engine has undergone two major rebrandings (Gazebo Classic -> Ignition -> Gazebo Sim/Jetty). 

**Rule of Thumb:** Always verify that code snippets, XML tags, or API calls you find online are for **Gazebo Sim/Jetty** (headers like `<gz/sim/...>`), and NOT Gazebo Classic (headers like `<gazebo/gazebo.hh>`).

## 3. Workflow & Conventions
Before making any codebase modifications, you MUST read and understand the following documents:
- `WORKFLOW.md`: Contains the standard git/branching workflow and PR requirements.
- `CONVENTIONS.md`: Contains strict Python (Ruff), C++, and ROS 2 styling conventions.

DO NOT skip this step. You must read these files to ensure your code aligns with the repository standards.
