# Launchers

## Overview

Although you can always write your own launcher as long as it has a launch method which returns a report dictionary, and it actually launches something, we will still propose a generic approach for writting a launcher, and this is how many of our out-of-box launchers are written. Commonly, a launcher can be broken into several pieces including an initial check, an action, a continuous check, and a report gathering. A simplified version of it is shown here:

![Common launcher approach](https://raw.githubusercontent.com/tianhaoz95/pics/master/launcher_schema.png)
