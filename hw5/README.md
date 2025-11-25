## Running the program 
`cd` into `hw5` and run `make` to compile everything. Then, run `./smooth ./data/your_scene.txt xres yres h`. To apply implicit fairing, press the `i` on your keyboard! (It may take a few seconds for the smoothing to be applied.)

### Building the F operator
This `build_F_operator` function builds off of the provided `build_B_operator`, with the biggest change being that we have to include the 1/2A factor in the discrete Laplacian for F. In addition, we must compute a Laplacian for every vertex, which we do by traversing through all vertices using the halfedge data structures. To ensure that the Laplacian property where the sum of all entries in each row is 0, we change the diagonals to be the negative of the sum of all other entries in the row (we don't care about diagonals otherwise). 
