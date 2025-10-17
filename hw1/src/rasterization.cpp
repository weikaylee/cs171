#include <cmath>
#include "rasterization.hpp" 

// first and eigth octants
void low_slope(int x1, int y1, int x2, int y2, int xres, int yres, vector<vector<int>>& grid) {
    int error = 0;
    int y = y1;
    int dx = x2 - x1;
    int dy = abs(y2 - y1);

    for (size_t x = x1; x <= x2; x++) {
        if (x >= 0 && x < xres && y >= 0 && y < yres) { // ignore anythign off screeen
            grid[y][x] = 1;
        }

        if (2 * (error + dy) < dx) {
            error += dy;
        }
        else {
            error += dy - dx;
            y += (y2 >= y1) ? 1 : -1;
        }
    }
}

// second and seventh octants
void high_slope(int x1, int y1, int x2, int y2, int xres, int yres, vector<vector<int>>& grid) {
    int error = 0;
    int x = x1;
    int dx = abs(x2 - x1);
    int dy = y2 - y1;

    for (size_t y = y1; y <= y2; y++) {
        if (x >= 0 && x < xres && y >= 0 && y < yres) {
            grid[y][x] = 1;
        }

        if (2 * (error + dx) < dy) {
            error += dx;
        }
        else {
            error += dx - dy;
            x += (x2 >= x1) ? 1 : -1;
        }
    }
}

// invalid points (x, y) are (-1, -1)
void bresenham(int x1, int y1, int x2, int y2,int xres, int yres, vector<vector<int>>& grid) {
    if (x1 == -1 || x2 == -1) {
        return;
    }

    if (abs(y2 - y1) <= abs(x2 - x1)) {
        if (x1 <= x2) {
            low_slope(x1, y1, x2, y2, xres, yres, grid);
        }
        else {
            low_slope(x2, y2, x1, y1, xres, yres, grid);
        }
    }
    else {
        if (y1 <= y2) {
            high_slope(x1, y1, x2, y2, xres, yres, grid);
        }
        else {
            high_slope(x2, y2, x1, y1, xres, yres, grid);
        }
    }
}
