# Modeling and Simulation of Social Systems Fall 2018 – Research Plan

> * Group Name: EPFM
> * Group participants names: Edoardo Berardo, Felix Schur, Mathias Gassner, Pedro Rosso
> * Project Title: evacuation bottleneck
> * Programming language: Python

## General Introduction

In our project "evacuation bottleneck" we are modeling the escape dynamics in a panic situation in a single room.
As many disasters in the past years have shown, we still do not fully understand crowd behavior especially in panic situations. An example would be the Love Parade disaster in 2010 in Duisburg or the Madrid incident in 2012, where five girls were killed.
With our model we want to give an insight in such evacuation situations and try to establish a connection between the number of barriers in the room and casualties.

## The Model

We create different rooms with a variation of additional barriers inside.
By running the simulation repeatedly we hope to find a relationship between the amount of barriers we added, their position and the number of casualties.

Since our model is a highly simplified version of reality, the application to real life scenarios needs to be further tested. Alone the reduction to two-dimensions is significant.


## Fundamental Questions
To what extent are additional barriers responsible for casualties in an evacuation scenario?
Do the position of the barriers in the room matter?

## Expected Results
Additional barriers increase the number of casualties, because they narrow the space and people will be closer to each other. Also the position should be of great significands. Two barriers getting more and more narrow, should increase the forces experience by the people.

## References 
[1]  D. Helbing, I. Farkas, and T. Vicsek, “Simulating dynamical features of escapepanic,”Nature, vol. 407, no. 6803, p. 487, 2000.

There are many extensions to our model. It is always possible to extend the scale of the rooms maybe even simulate an evacuation of a multifloor building.
Also adding random behavior would increase the connection to real life events.

## Research Methods
We are using a social force model.

## Results
The simulations are uploaded in the youtube playlist:
https://www.youtube.com/playlist?list=PLzdxxpuxb1n3Ur7Cy9LNWn5Lg70n7Ue9-

## Other
There are no external datasets used.

# Reproducibility

## Light test
We assume you have python 3.6 (or higher) installed.


### Debian/Ubuntu:

If not allready installed, install pip by running the command 

"sudo apt-get install python3-pip"

in your terminal.

Install the python packages numpy, pygame, scipy,  progress.bar, sys, matplotlib by
running the following commands in your terminal:

"sudo pip install numpy"

"sudo pip install scipy"

"sudo -m pip install -U pygame --user"

"sudo pip install progressbar2"

"sudo -m pip install -U matplotlib"

Open running_code.py in your editor and read the comments.
Now run the code by running the following command:

"python3 PATH_TO_FOLDER/evacuation-bottleneck-master/code/code_python/running_code.py"

It should not run longer than two minutes. Now there should pop up a Window which shows you three graphs in different colors.
If you close this Window, a new one will pop up and show the simulation in "real time".
It is possible, that there are bugs in the simulation. This is because you used leap_frog 
as the integration methode. Leap_frog is inaccurate, but is fast enough for a normal laptop.

If you want you can now change some variables and look at the result.


## Full test

We assume you have python 3.6 (or higher) installed.


### Debian/Ubuntu:

If not allready installed, install pip by running the command 

"sudo apt-get install python3-pip"

in your terminal.

Install the python packages numpy, pygame, scipy,  progress.bar, sys, matplotlib by
running the following commands in your terminal:

"sudo pip install numpy"

"sudo pip install scipy"

"sudo -m pip install -U pygame --user"

"sudo pip install progressbar2"

"sudo -m pip install -U matplotlib"

Open running_code.py in your editor und read the comments.
Now run the code by running the following command:

"python3 PATH_TO_FOLDER/evacuation-bottleneck-master/code/code_python/running_code.py"

It should not run longer than two minutes. Now there should pop up a Window which shows you three graphs in different colors.
If you close this Window, a new one will pop up and show the simulation in "real time".
It is possible, that there are bugs in the simulation. This is because you used leap_frog 
as the integration methode. Leap_frog is inaccurate, but is fast enough for a normal laptop.

If you want you can now change some variables and look at the result.

# Files in this folder

Simulation_class.py: A class which coordinates the simulation and saves the results. It contains an instance of Integrators, Room and diff_equation.

Integrators.py: A class used for the integration of the simulation.

Room.py: A class with all the rooms, for which we implemented simulation-tests.

diff_equation.py: A class for the differential equation of the forces on the agents. It is used in the Integration class.