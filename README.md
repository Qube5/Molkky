For more information, please see [molkky.weebly.com](https://molkky.weebly.com)

# Mölkky
Baxter Robot plays [Molkky](https://en.wikipedia.org/wiki/Mölkky), a Finnish lawn game. Berkeley EECS 106A Project Fall 2019
We are modifying the game to scaled down foam pieces and play indoors on a table top. 

## Molkky Example
[![Molkky Overview](https://img.youtube.com/vi/52gHJ3twa5g/0.jpg)](https://www.youtube.com/watch?v=52gHJ3twa5g)

## Molkky Rules Overview
[![Molkky Rules](https://img.youtube.com/vi/S65up-hEmaI/0.jpg)](https://www.youtube.com/watch?v=S65up-hEmaI)

## Sawyer plays Molkky
[![Sawyer plays Molkky](https://img.youtube.com/vi/5wgZ72Oa4w0/0.jpg)](https://www.youtube.com/watch?v=5wgZ72Oa4w0)

## Project Overview
#### Vision 
- Find pins, read pin score values 

#### Strategy
- Calculate optimal expected move based on move choices and likelihood of achieving moves

#### Projectile Trajectory Planning
- Calculate pin trajectory using projectile motion. Use this to extract toss velocity and angle

#### Forward Kinematics & Motion Planning
- Baxter moves to location and launches pin

#### Physical Modifications
- Custom Gripper for Baxter to hold and launch toss pin

#### Other Manufacturing
- Manufactured game board and pieces to optimize vision algorithms

# Getting Started
- see [src/README.md](https://github.com/Qube5/Molkky/edit/master/src)
