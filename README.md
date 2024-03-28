# thesis-mppi-model-ident

something useful probably or the complete opposite

## Main guidelines

- focus on keeping the developer environment same as enjoy dev container
  - keep the important part (backbone)
  - minimize for not needed complexity
  - keep added extras separate
- use same versions as in enjoy workspace (docker, syntax, pip, apt)
  - only differ in versions if specific packages needs them

## Useful stuff

### General

- NAV2: https://navigation.ros.org/concepts/index.html#ros-2
- https://github.com/ros-planning/navigation2/tree/main/nav2_controller

- BT book: https://arxiv.org/pdf/1709.00084.pdf

- ros-docker: https://github.com/osrf/rocker

### Github

- Discourse about nav2 benchmark: https://discourse.ros.org/t/nav2-discussion-metrics-framework-for-quantitative-evaluation-of-navigation-performance/28082

- NAV2 benchmark tools (planner): https://github.com/ros-planning/navigation2/tree/main/tools/planner_benchmarking

- NAV2 issue about controller benchmark: https://github.com/ros-planning/navigation2/issues/3239
    - branch: https://github.com/enricosutera/navigation2/tree/benchmark_controllers

## Sources

### Docker

Dockerfile setup and docker-compose.yml with ros2
- http://wiki.ros.org/docker/Tutorials/Docker
- https://roboticseabass.com/2021/04/21/docker-and-ros/
- https://roboticseabass.com/2023/07/09/updated-guide-docker-and-ros2/

### ROS devcontainer

Devcontainer inside vscode for ros
- https://docs.ros.org/en/humble/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html
- https://github.com/lzptr/VS_Code_ROS

### VsCode

Intelisense for roc2 cpp (c_cpp_properties.json):
- https://code.visualstudio.com/docs/cpp/c-cpp-properties-schema-reference
Tasks:
- https://github.com/athackst/vscode_ros2_workspace/blob/humble/.vscode/tasks.json
- https://www.allisonthackston.com/articles/vscode-docker-ros2.html

### Benchmarking
REP 2014
- it is mainly for latency cpu usage...
- https://ros.org/reps/rep-2014.html (smart things for benchmarking - pkges...)
- https://discourse.ros.org/t/rep-2014-rfc-benchmarking-performance-in-ros-2/27770
- https://github.com/ros-infrastructure/rep/pull/364
Performance test (communication - dds)
- https://gitlab.com/ApexAI/performance_test/-/tree/master/performance_test?ref_type=heads

### NAV2 benchmarking
- peak Maci comments: https://discourse.ros.org/t/nav2-discussion-metrics-framework-for-quantitative-evaluation-of-navigation-performance/28082/
