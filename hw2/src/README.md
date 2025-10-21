## Running the code
To run, `cd` into the `src` folder and run `make`. Then, to run and view a test image, run `.\main ..\data\your_scene.txt xres yres | magick - out.png`

## Bresenham's
To implement a generalized Bresenham algorithm, we first look at the case when the slope is between 0 and 1 (low) and when the slope is larger than 1 (high) (in the case of negative slopes, we can simply swap (x_1, y_1) with (x_2, y_2) such that the slope is positive). 

In the low case, we can implement the first octant algorithm. We only need to change the direction that we increment our y variable; if y2 > y1, then increment by +1, else increment by -1. 

In the high case, we can convert the steep slope to a low slope by simply traversing along the y axis and incremnting our x variable, effectively swapping our x and y axes. Thus, the algorithm is analogous to the low case. 