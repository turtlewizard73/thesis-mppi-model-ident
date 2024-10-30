# Roadmap 2

1. build container
2. run gazebo on empty world
3. run bringup in controller_benchmark pkg
4. run tests


## 2024. 09. 10.
 - try to get it working again
 <!-- - choose to optimize it so make it so it could be -->
 - ControllerResult class storing memory optimalization
 - logging perfected to file and stream added argparse -d debug flag
   - https://docs.python.org/3/howto/logging.html#handler-basic

## 2024. 09. 11.
- ha start_time: rclpy.clock.Time = rclpy.clock.Clock(clock_type=ClockType.ROS_TIME).now()
- ot használok és subsrciber nodeokban csak elmentem akkor a msg header stampben random szimulációs idő llesz tehát halál fasza
- szóval a következmény hogy msg beérkezésekor új időt kell adni ami azért jó mert gyakorlatilag controllert is az befolyásolja (minden feliratkozó egyszerre kapja meg) tehát az az idő számít amikor megkapja nem amikor létrejött a msg
- meg amúgy is a szimuláció konstans fut szóval ott is kérdőjelek


## 2024. 10. 27.
- fut metricek is
- paraméter újratöltésen elkezdtem dolgozni

## 2024. 10. 28.
| Algorithm | Pros | Cons | When to Use |
|-|-|-|-|
| **Grid Search**          | Exhaustive, simple to implement                | Computationally expensive, slow                | Small parameter space                      |
| **Random Search**        | More efficient than Grid Search, simple        | Random nature can miss optimal solutions       | Larger parameter space                     |
| **Bayesian Optimization**| Efficient, balances exploration and exploitation | Requires setup and additional libraries        | Large parameter space, limited trials      |
| **Genetic Algorithms**   | Good for complex, non-linear problems          | Computationally expensive, needs tuning        | Complex spaces, when other methods fail    |
| **Simulated Annealing**  | Simple, good for escaping local minima         | Can be slow to converge                        | Moderate complexity, non-linear problems   |
| **Hyperband**            | Efficient, quickly prunes poor solutions       | Needs tuning for resource allocation           | Multi-trial optimization                   |
| **Particle Swarm**       | Intuitive, works for multi-dimensional spaces  | Can get stuck in local minima                  | Non-linear, multi-dimensional spaces       |

https://chatgpt.com/c/66e00e8e-aaa8-8011-bc03-99adcddc8d95

https://scikit-learn.org/dev/getting_started.html

gtid search
https://www.youtube.com/watch?v=xRhPwQdNMss

!!!!!!!!!!!
metrikák ból egy függvény ugye hogy közel milyen jó lenne pl
(sebesség optimális - mért) * súly .....
aztán kézzel irányítani -> majd abcd súlyokkal görbét illeszteni


## 2024. 10. 30.
- setting params programatically from benchmark
- https://roboticsbackend.com/rclpy-params-tutorial-get-set-ros2-params-with-python/
