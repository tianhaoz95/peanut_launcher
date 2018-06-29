<img width="30%" src="https://raw.githubusercontent.com/tianhaoz95/pics/master/peanut-logo.PNG"/>

# Peanut Launcher

## Introduction

Launching for ROS project can be painful when the project gets big. Even when you have a perfect launch file, it is still hard to migrate and reuse part of the project in another project or configure the launch file dynamically for testing purpose. Here the idea is to let each atomic part of the project take care of its own launching process, and the high level project will act like a supervisor to monitor the launching process without being aware of what is being launched.

## A project consists of 1 supervisor and many launchers

The basic idea is that supervisor will manage how to launch launchers, and launchers will take care of the details of actual launching process. The only contract supervisor ask from launcher is that it has a launch method which returns a report (dict) consist of status and details of this launching task. This means that you can take any combination of parts from one project and make them into a new project because the supervior doesn't know details about launcher by default, so nothing will break. A simplified version of what supervisor and launchers do is shown here:

![Supervisor vs. Launcher](https://raw.githubusercontent.com/tianhaoz95/pics/master/supervisor%20vs%20launcher%20-%20Page%201.png)

## Supervisor controls the flow of launching and Launcher controls the details

Simply put, supervisor will first take in many launchers, and then run them in different threads and get report back when they finish. Compose a final report fom the report and let the user know how the overall launching process went.

![What is a supervisor](https://raw.githubusercontent.com/tianhaoz95/pics/master/Blank%20Diagram%20-%20Page%201.png)
