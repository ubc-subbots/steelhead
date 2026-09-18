# Agent Knowledge Base: Steelhead AUV

This file contains critical context, rules, and learnings for AI agents working in the `steelhead` repository.

**ATTENTION ALL AGENTS:** You MUST read and follow these rules before making any codebase modifications.

## 1. Core Rules & AI Policy
* **ROS 2 Version:** We are using **ROS 2 Lyrical** on Ubuntu 26.04.
* **Clean Artifacts:** Never leave behind temporary scratch scripts (like `fix_plugin.py`), dummy comments, or conversational AI placeholders in the codebase (e.g., `// Add your code here`). If you create scratch scripts, delete them before finishing your turn.
* **Accountability:** AI is infamous for hallucinating outdated ROS 2 or Gazebo Classic code. Always double-check API versions and documentation before applying fixes. Do not introduce breaking changes to APIs without verifying they exist in ROS 2 Lyrical.

## 2. Gazebo Harmonic & ROS 2 Migration Gotchas
The physics engine has undergone two major rebrandings (Gazebo Classic -> Ignition -> Gazebo Sim/Harmonic). 

**Rule of Thumb:** Always verify that code snippets, XML tags, or API calls you find online are for **Gazebo Sim/Harmonic** (headers like `<gz/sim/...>`), and NOT Gazebo Classic (headers like `<gazebo/gazebo.hh>`).
