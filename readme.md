# Modern Robotics

- This directory is the dev direcotory for the planning course

- Link to modern robotics course - (https://www.coursera.org/specializations/modernrobotics)
- My user is Mike Wise - mikewise1618@gmail.com
- Actual task: https://www.coursera.org/learn/modernrobotics-course4/peer/yCetp/sampling-based-planning

 # Astarmain.py tests

```
D:\python\prmrrt>python astarmain.py -fp

mincost:0.110 maxcost:0.599 avgcost:0.159
bestpath: ['1', '2', '5', '7', '10', '12']
bestpath cost:1.61330
```
Alternate test
```
D:\python\prmrrt>python astarmain.py -f 6.000000 -t 2.000000  -s oldPRM -d oldPRM -fp
mincost:0.057 maxcost:0.317 avgcost:0.087
bestpath: ['6.000000', '4.000000', '3.000000', '2.000000']
bestpath cost:0.59820
```
```
D:\python\prmrrt>python astarmain.py -f 1.000000 -t 36.000000  -s oldPRM -d oldPRM -fp
edge costs min:0.057 max:0.317 avg:0.087
bestpath: ['1.000000', '9.000000', '14.000000', '16.000000', '21.000000', '23.000000', '27.000000', '33.000000', '34.000000', '36.000000']
bestpath cost:1.54925
```