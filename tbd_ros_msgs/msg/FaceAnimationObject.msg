string name # Name of the object
float32[] target # Target to Reach, will be single object array if only have one variable
float32 duration # Length of animation

uint8 type # type of animation
uint8 LINEAR=0 # go from (0,1) at constant speed 
uint8 ACCELERATE_DECCELERATE=1 # start slowly and accelerate until reach 0.5 and deccelerate. Basically a triangle profile.
