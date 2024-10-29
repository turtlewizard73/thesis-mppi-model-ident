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

!!!!!!!!!!!
metrikák ból egy függvény ugye hogy közel milyen jó lenne pl
(sebesség optimális - mért) * súly .....
aztán kézzel irányítani -> majd abcd súlyokkal görbét illeszteni
