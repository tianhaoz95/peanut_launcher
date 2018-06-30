# Supervisors APIs

## Overview

Simply put, supervisor will first take in many launchers, and then run them in different threads and get report back when they finish. Compose a final report fom the report and let the user know how the overall launching process went. Here is a diagram showing this process:

![What is a supervisor](https://raw.githubusercontent.com/tianhaoz95/pics/master/Blank%20Diagram%20-%20Page%201.png)

## Supervisor Classes

### BaseSupervisor

#### Example

```python
from peanut_launcher.supervisor import BaseSupervisor

class MySupervisor(BaseSupervisor):
  def __init__(self):
    self.launchers = []

  def add_launcher(self, launcher):
    self.launchers.append(launcher)
  
  def launch_all(self):
    try:
      for launcher in self.launchers:
        launcher.launch()
      return 'success', None
    except:
      return 'fail', None
```

### MultiThreadSupervisor

#### Example

```python
from peanut_launcher.supervisor import MultiThreadSupervisor
from peanut_launcher.launchers import CommandLauncher

class Checker():
  def __init__(self, package_name):
    self.package_name = package_name
  
  def check_output(self, output):
    return self.package_name in output

cl1 = CommandLauncher('roslaunch my_package_1', 'rostopic list', Checker('my_package_1').check_output)
cl2 = CommandLauncher('roslaunch my_package_2', 'rostopic list', Checker('my_package_2').check_output)
cl3 = CommandLauncher('roslaunch my_package_3', 'rostopic list', Checker('my_package_3').check_output)

s = MultiThreadSupervisor()
s.add_launcher('cl1', cl1)
s.add_launcher('cl2', cl2)
s.add_launcher('cl3', cl3)

status, info = s.launch_all()
```
