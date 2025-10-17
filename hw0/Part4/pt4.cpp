#include <stdlib.h>
#include <assert.h>
#include <string>
#include <iostream> 
#include <vector> 
#include <sstream>
#include <fstream>
#include <unordered_map> 

using namespace std; 

int main(int argc, char *argv[]) {
    int xres = atoi(argv[1]);
    int yres = atoi(argv[2]);

    string color_bg = "0 0 0";       
    string color_circle = "255 255 255"; 

    // compute circle diameter (half of the smaller dimension)
    int diameter = (xres <= yres) ? xres / 2 : yres / 2;
    float radius = diameter / 2.0f;

    // center coordinates
    float cx = xres / 2.0f;
    float cy = yres / 2.0f;

    // PPM header
    cout << "P3" << endl;
    cout << xres << " " << yres << endl;
    cout << "255" << endl;

    // iterate over every pixel
    for (int j = 0; j < yres; ++j) {        
        for (int i = 0; i < xres; ++i) {      
            // compute distance from center
            float dx = i - cx;
            float dy = j - cy;
            float dist = dx * dx + dy * dy;

            if (dist <= radius * radius) {
                cout << color_circle << endl;
            } 
            else {
                cout << color_bg << endl;
            }
        }
    }
}