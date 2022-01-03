//
// Created by sylvain on 29/01/18.
//

#ifndef NUMERISATIONPROJECT_C_SDL2TOOLS_H
#define NUMERISATIONPROJECT_C_SDL2TOOLS_H

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <time.h>

#include <iostream>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

typedef struct{
    char * title;
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float scale_x;
    float scale_y;
}GraphStructure;




class SDL2Tools {
private :

public :

    SDL2Tools();
    ~SDL2Tools();

    SDL_Window* creatSDLFrame(int widht, int height);
    void launchSDLCore(SDL_Window* frame1);
    void deleteSDLFrame(SDL_Window* frame1);

    void drawSDLGraph(SDL_Window* frame1, SDL_Renderer * renderer, GraphStructure structure);
    void drawSDLGraphLine(SDL_Window* frame1, SDL_Renderer * renderer, GraphStructure structure, float x1, float y1, float x2, float y2);
    void drawSDLGraphPoint(SDL_Window* frame1, SDL_Renderer * renderer, GraphStructure structure, float x1, float y1);
    void drawText(SDL_Renderer * renderer, char* text, int x, int y, int size);

};


#endif //NUMERISATIONPROJECT_C_SDL2TOOLS_H
