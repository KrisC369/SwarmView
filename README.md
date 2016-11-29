## Instalation
Configure IDE to use autovalue
- http://www.codeaffine.com/2014/03/04/using-the-autovalue-code-generator-in-eclipse/

## Project Structure
The project is divided in two sub-modules
- Swarmview-trajectory
- Swarmview-visualization

Importing the main project using gradle will compile both sub-modules.

## Using the View
The presentation starts in a paused state.

The view has key-bindings defined as:
- **z** activates/deactivates the mouse
- **t** displays/hides the simulation time
- **d** displays/hides the drone names
- **,** backwards the simulation time by 5 seconds
- **.** forwards the simulation time by 5 seconds
- **SPACE key** pauses the simulation view. Note that the simulation time continues advancing in the background. Imagine this option as a snapshot in time.
- **r** restarts the simulation

## Programming interface
New **Drone** objects can be created by specifying 
- Drone(PApplet canvas, FiniteTrajectory4d trajectory)
- Drone(PApplet canvas, FiniteTrajectory4d trajectory, int color, int trailSize)
