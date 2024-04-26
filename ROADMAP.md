# Roadmap

## before 2024. 03. 28.
- created dev container (docker, vscode)

## 2024. 03. 24.
- forked nav2
- merged wip controller benchmark to my fork

## 2024. 03. 29.
- created own pkg for controller benchmark
- created suitable launch file for controller benchmark
- run test with own launch file

## 2024. 04. 05-09.
- study metrics.py
- rewrite metrics.py -> run_test.py
- talked with Emil about metrics -> stamp msgs when arrived

## 2024. 04. 14.
- run_test.py and plot_results.py working

## 2024. 04. 16.
- started working on turtlebot
  - what type of raspberry does it use?
    - test in local and only copy image
  - need of an sd card (min 16gb) and a reader
  - next time bring own cable (ethernet)

## TODOs - 2024. 04. 10.
- make gazebo gui headless?
- IMPORT WHYY NOT WORKING???
- finish run_test.py
  - parametrize
  - think about number of retries
- separate important configs to yaml file
  - those that importatnly needded for run_test.py
  - for example: publish twist stamped true
  - to override given params.yaml
- currently collected data sufficent enough?
  - if not, what to collect?
- make run_test.py to be able to run on turtlebot
  - separate launch?
  - how to collect data?

## Plans
- study metrics.py make it better, simpler
- rewise used metrics
- run metrics with own params
- change something in mppi controller to see the difference
