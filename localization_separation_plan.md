# Localization Algorithm Development Project Plan

## Objective B: Standalone Localization Repository with API
- **Description:** Create IMU & odometry-based localization in a new repository with an API to the central algorithm (AI-driver)
- **Success Metrics:**
  - Completed extraction of localization module from vehicle control
  - Implemented clean API for the AI-driver to access localization data
  - Successfully transitioned vehicle control to use the new localization API
  - Comprehensive documentation and testing of the new repository

### Tasks for Objective B: Standalone Localization Repository with API

#### Current State and Challenges
- Localization module currently nested in vehicle control module
- Not isolated as a standalone component with clean API
- Need to establish connections with AI-driver and visual odometry module
- Sensor API already implemented in common control & localization repository

#### Task Breakdown

##### Repository Setup and Code Migration

| Task # | Description | Effort (days) |
|--------|-------------|---------------|
| B1 | Merge localization metrics pull request to main branch | 1 |
| B2 | Create new repository structure for standalone localization module | 1 |
| B3 | Extract and migrate localization code from Vehicle Control repository | 3 |
| B4 | Set up build system and dependency management | 2 |

##### Python Bindings and API Enhancement

| Task # | Description | Status | Effort (days) |
|--------|-------------|--------|---------------|
| B5 | Create or verify Python bindings (pybind) for localization algorithm | ✅ Completed | 3 |
| B6 | Ensure compatibility with existing sensor API | ✅ Completed | 2 |

##### Testing Framework

| Task # | Description | Status | Effort (days) |
|--------|-------------|--------|---------------|
| B7 | Create Python test script for loading AI-driver trip data | 🔄 In Progress | 2 |
| B8 | Implement synchronous sensor data processing using timestamps | 🔄 In Progress (synchronous loading implemented, to do: process data in algorithm and verify correctness) | 3 |

##### Documentation

| Task # | Description | Effort (days) |
|--------|-------------|---------------|
| B9 | Document API usage and examples | 2 |

##### Code Cleanup

| Task # | Description | Effort (days) |
|--------|-------------|---------------|
| B10 | Remove control-related unused functionality | 2 |

**Total Effort for Objective B: 21 days**
