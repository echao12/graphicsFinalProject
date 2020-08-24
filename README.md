# CSE 170 Computer Graphics Final Project
![Project Picture](https://github.com/echao12/graphicsFinalProject/blob/master/graphicsFinalPicture.PNG)
## Description
This program will load a city model with various characters that can be animated.
The player can control the movement of a minecraft inspired model that spawns in the center of the map.
When the "animate" button is pressed, there will be a camera fly-by animation for the viewer and the models in the city will begin to move.

## Controls
W - forward movement &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; Q - Turn head left</br>
A - Turn body left &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;E - Turn head right</br>
S - Take a step back </br>
D - Turn right</br>

## Development
This project was developed by a team of 4 total members using the provided sig library from CSE 170. 
Unfortunately, I was unable to upload the library to this repo.

My role in this project was essentially the programming lead.
I took the code developed by my partners and compiled them into one program for the final submission.
Thus, the code structure, the block NPCs, the Bezier curves and their animations, and the model positions were developed by me.

The city model, the robot, the red car, and the camera fly-by functionality were developed and obtained by my partners.

## Design Decisions
I structured the code in a way that it was easily modifiable and abstracted model generation to different functions.
This way, it was much easier to spawn in many NPCs to fill the city as well as other models.

The minecraft inspired block models were developed using provided primitives.
The character was a heirarchical model where each body part was contained in individual groups.
Then, I took those groups and added them to a root group. 
This made global movement easy since I didn't have to apply transforms to each body part to move the model around.
The individual body parts are animated when the character moves, but in a stiff static fashion.

I decided to just randomize the colors for each model's body part to add some color since the city model didn't spawn in with color.

The movement along the city were developed with Bezier curves, however I was unable to complete the full path, thus they are more like Bezier lines xD.
