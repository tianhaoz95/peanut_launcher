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
