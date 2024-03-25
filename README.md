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







## Sources:
### Docker
Dockerfile setup and docker-compose.yml with ros2
- http://wiki.ros.org/docker/Tutorials/Docker
- https://roboticseabass.com/2021/04/21/docker-and-ros/
- https://roboticseabass.com/2023/07/09/updated-guide-docker-and-ros2/
### ROS devcontainer
Devcontainer inside vscode for ros
- https://docs.ros.org/en/humble/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html