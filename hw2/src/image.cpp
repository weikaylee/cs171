#include <iostream>
#include "image.hpp"
#include "object.hpp"

void get_ppm(int xres, int yres, vector<vector<Color>> grid) { 
    cout << "P3" << endl;
    cout << xres << " " << yres << endl;
    cout << "255" << endl;

    for (int i = 0; i < yres; i++) {        
        for (int j = 0; j < xres; j++) {
            int r = int(255 * grid[i][j].r);
            int g = int(255 * grid[i][j].g);
            int b = int(255 * grid[i][j].b);

            cout << r << " " << g << " " << b << endl;
        }
    }
}
