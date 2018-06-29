# Peanut Launcher

## Introduction

Launching for ROS project can be painful when the project gets big. Even when you have a perfect launch file, it is still hard to migrate and reuse part of the project in another project or configure the launch file dynamically for testing purpose. Here the idea is to let each atomic part of the project take care of its own launching process, and the high level project will act like a supervisor to monitor the launching process without being aware of what is being launched.

## Supervisor controls the flow of launching 

![What is a supervisor](https://raw.githubusercontent.com/tianhaoz95/pics/master/Blank%20Diagram%20-%20Page%201.png)
