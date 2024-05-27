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

## 2024. 05. 12.
- fixed container bashrc
- fixed container entrypoint
- fixed container ros2 install update -> nav2 added as rosdep
- added frechet module to calculate plan and route distance

## 2024. 05. 15.
- optimized:
  - plot_results_v2.py - dist_fullplan -> np array
  - maketh benchmark decorator
    - found that fdfdm eats a lot of memory -> one object (global)
  - calculate_metrics with ProcessPoolExecutor paralellized

## 2024. 05. 20.
- nav2 doesnot work with rosddep -> change to apt install
  - mppi critics cannot be forwarded to print out
- removed noise from turtlebot model sdf for 100% odometry
- added obstacles and mapcreator
  - added map change to runtest

## 2024. 05. 21.
- added custom start goal generator (optimize it to prefer 0, 0)
- better polar coordinate random goal generator
- mppi stil not working eh
- added world with plugin so entity set state service can be called

## 2024. 05. 27.
- mppi critic publisher with plotting (big stuff)
- finalized randomg polar generation
- made derivative benchmarking (összehasonlítás és van saját newton3/8-as)
  - jobb mert kisebb az outlier érték, de lecheckolni a végén miért nagy
  - ugyanitt vektorosítani hogy szép legyen
  - TODO: de lehet nem lehet vektorosítani mert dt ugye nem uniform


végső kukoc ahol vizualizálja (annak stampjével) rádurni a critikeket újból és azokat publisholni
https://github.com/turtlewizard73/navigation2/blob/a5fd16c0aea84a4dbeccec15bec37f1d35a3b332/nav2_mppi_controller/src/controller.cpp#L109


TODO: outlier filtering for points, and interpolate the filtered values

## TODOs - 2024. 04. 10.
- make gazebo gui headless? - done
- IMPORT WHYY NOT WORKING??? - working - somehow solved
- finish run_test.py
  - parametrize - done
  - think about number of retries
- separate important configs to yaml file - done
  - those that importatnly needded for run_test.py
  - for example: publish twist stamped true
  - to override given params.yaml
- currently collected data sufficent enough?
  - if not, what to collect?
- make run_test.py to be able to run on turtlebot
  - separate launch?
  - how to collect data?
