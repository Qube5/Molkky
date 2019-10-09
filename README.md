# Mölkky
Baxter Robot plays [Molkky](https://en.wikipedia.org/wiki/Mölkky), a Finnish lawn game. Berkeley EECS 106A Project Fall 2019
We are modifying the game to scaled down foam pieces and play indoors on a table top. 

## Molkky Example
[![Molkky Overview](https://img.youtube.com/vi/52gHJ3twa5g/0.jpg)](https://www.youtube.com/watch?v=52gHJ3twa5g)

## Molkky Rules Overview
[![Molkky Rules](https://img.youtube.com/vi/S65up-hEmaI/0.jpg)](https://www.youtube.com/watch?v=S65up-hEmaI)

## Project Overview
#### Vision 
- Find pins, read pin score values 

#### Strategy
- Calculate optimal expected move based on move choices and likelihood of achieving moves

#### Projectile Trajectory Planning
- Calculate 'pin' trajectory using projectile motion. Use this to extract toss velocity and angle

#### Forward Kinematics & Motion Planning
- Baxter performs tossing motion

#### Physical Modifications
- Gripper for Baxter to hold and release toss pin

#### Other Manufacturing
- Make game board and game pieces

#### Additional Reach Goals:
- Have Baxter able to reset pins
- Baxter can reload his toss pin

# Getting Started
- Clone Repo
- Run Launch File
