/**
 * @brief 
 * 
 * @author Sylvain Colomer
 * @date 23/04/19
 * @version 1.0
 */

#include "../include/Display_Panel.h"


Ncurses_Panel::Ncurses_Panel(int x, int y, int lines, int cols)
{
    Ncurses_Panel::x=x;
    Ncurses_Panel::y=y;
    Ncurses_Panel::lines=lines;
    Ncurses_Panel::cols=cols;

    window = subwin(stdscr, lines, cols, x, y); 
}

Ncurses_Panel::~Ncurses_Panel()
{
    delwin(window);
}

void Ncurses_Panel::updateSize(int x, int y, int lines, int cols)
{
    Ncurses_Panel::x=x;
    Ncurses_Panel::y=y;
    Ncurses_Panel::lines=lines;
    Ncurses_Panel::cols=cols;

    mvderwin(window, x, y);
    wresize(window, lines, cols);
}
