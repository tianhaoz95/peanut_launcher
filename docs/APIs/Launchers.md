# Launchers APIs

## Overview

Although you can always write your own launcher as long as it has a launch method which returns a report dictionary, and it actually launches something, we will still propose a generic approach for writting a launcher, and this is how many of our out-of-box launchers are written. Commonly, a launcher can be broken into several pieces including an initial check, an action, a continuous check, and a report gathering. A simplified version of it is shown here:

![Common launcher approach](https://raw.githubusercontent.com/tianhaoz95/pics/master/launcher_schema.png)

## Launcher classes

### BaseLauncher

BaseLauncher is the abstract base class for all launchers which only enforce a launch method. If you want to fully customize your launcher, you will derive from this class and ensure that your process is fully up when the launch method returns.

#### Example

```python
from peanut_launcher.launchers import BaseLauncher

class MyLauncher(BaseLauncher):
  def launch(self):
    print('pretending I am launching something ...')
```

### BlockingLauncher

#### Example

```python
from peanut_launcher.launchers import BlockingLauncher

class MyLauncher(BlockingLauncher):
  def launch_finished(self):
    if 'my_topic' in check_output('rostopic list'):
      return True
    else:
      return False

  def launch_process(self):
    launch_command('roslaunch my_package')
```

### CommandLauncher

#### Example

```python
from peanut_launcher.launchers import CommandLauncher

launch_command = 'roslaunch my_package'
check_command = 'rostopic list'

def output_checker(output):
  if 'my_package' in output:
    return True
  else:
    return False

my_launcher = CommandLauncher(launch_command, check_command, output_checker)
```
