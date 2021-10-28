/**
 * @brief Class that model a Display_sdlFrame
 * 
 * @author Sylvain Colomer, Alexis
 * @date 10/10/19
 * @version 1.0
 */

#include <iostream>
#include <string>

using namespace std;

#include "../include/Display_MainFrame.h"

#include "../include/Constants.h"


// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

MainFrame::MainFrame(int task_period, int task_deadline)
    :Abstract_ThreadClass(task_period, task_deadline)
{

    // Init option
    initscr();
    start_color();
    noecho();
    curs_set(0);

    // Init color
    if (has_colors() == TRUE) 
    {
        init_pair(NCURSES_TEXT_DEFAULT, COLOR_WHITE, COLOR_BLACK);
        init_pair(NCURSES_TEXT_GREEN, COLOR_GREEN, COLOR_BLACK);
        init_pair(NCURSES_TEXT_RED, COLOR_RED, COLOR_BLACK);
        init_pair(NCURSES_TEXT_ORANGE, COLOR_YELLOW, COLOR_BLACK);
        init_pair(NCURSES_TEXT_BLUE, COLOR_BLUE, COLOR_BLACK);

        init_pair(NCURSES_WINDOW_1, COLOR_BLACK, COLOR_BLUE);
        init_pair(NCURSES_WINDOW_2, COLOR_BLACK, COLOR_MAGENTA);

    }
    else
    {
        cout<<"Your terminal does not support \n";
    }

    // Init element
    panel_header = new Ncurses_Panel(0, 0, 3, COLS);

    panel_state = new Ncurses_Panel(3, 0, 3, 20); 
    panel_thread = new Ncurses_Panel(3, 20, 3, COLS/2-20); 
    panel_comm = new Ncurses_Panel(6, 0, 3, COLS/2); 
    panel_drones = new Ncurses_Panel(9, 0, 3, COLS/2); 
    panel_details = new Ncurses_Panel(12, 0, LINES-12, COLS/2); 

    panel_log = new Ncurses_Panel(3, COLS/2, LINES-3, COLS/2); 
    
}

MainFrame::~MainFrame()
{
    delete panel_header;
    delete panel_drones;
    delete panel_log;
    endwin();   
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GLOBAL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void MainFrame::resize_all()
{

    panel_header->updateSize(0, 0, 3, COLS);

    panel_state->updateSize(3, 0, 3, 20); 
    panel_thread->updateSize(3, 20, 3, COLS/2-20);  
    panel_comm->updateSize(6, 0, 3, COLS/2); 
    panel_drones->updateSize(9, 0, 3, COLS/2); 
    panel_details->updateSize(12, 0, LINES-12, COLS/2); 

    panel_log->updateSize(3, COLS/2, LINES-3, COLS/2); 
}
void MainFrame::repaint()
{
    clear_all();
    resize_all();
    paint_all();
}
void MainFrame::refresh_all()
{
    refresh();
    wnoutrefresh(panel_header->window); 
    wnoutrefresh(panel_state->window); 
    wnoutrefresh(panel_thread->window);
    wnoutrefresh(panel_comm->window);
    wnoutrefresh(panel_drones->window); 
    wnoutrefresh(panel_details->window); 
    refresh_log();
    doupdate();
    
}
void MainFrame::clear_all(){
    erase();
    clear();
    endwin();
}

void MainFrame::paint_all()
{

    box(panel_header->window, ACS_VLINE, ACS_HLINE);
    mvwprintw(panel_header->window, 1, 1,"MAVLINK SERVER V1");
    mvwprintw(panel_header->window, 1, 24,"F1:HELP");
    mvwprintw(panel_header->window, 1, panel_header->cols-12,"ECHAP:QUIT");
    wnoutrefresh(panel_header->window); 

    paint_state();

    paint_threads();

    paint_comm();

    box(panel_drones->window, ACS_VLINE, ACS_HLINE);
    mvwprintw(panel_drones->window, 0, 1, "DRONES");
    wnoutrefresh(panel_drones->window); 

    paint_details();

    paint_log();


}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DETAILS %%%%%%%%%%%%%%%%%%%%%%%
void MainFrame::paint_details()
{
    box(panel_details->window, ACS_VLINE, ACS_HLINE);
    mvwprintw(panel_details->window, 0, 1, "DETAILS");
    refresh_details();
}
void MainFrame::refresh_details(){
    int cpt_lines = 1;    
    int lines_max = 0;
    int cols_max = 0;
    std::string buffer1="";

    getmaxyx(panel_details->window, lines_max, cols_max);

    for( auto itr = details_list.begin(); itr != details_list.end(); itr++)
    {
        erase_line(panel_details->window, cpt_lines);
        write_line(itr -> first+": "+std::get<0>(itr -> second), panel_details->window, cpt_lines, 1, std::get<1>(itr -> second));

        if(cpt_lines>=lines_max-2) break;
        cpt_lines++;
    }
    wrefresh(panel_details->window);
    wnoutrefresh(panel_details->window); 
}
void MainFrame::setDetail(std::string key, std::string value, int level)
{
    details_list[key] = std::make_pair(value, level);
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% STATE %%%%%%%%%%%%%%%%%%%%%%%
void MainFrame::paint_state()
{
    box(panel_state->window, ACS_VLINE, ACS_HLINE);
    write_line("STATE", panel_state->window, 0, 1, NCURSES_TEXT_DEFAULT);
    write_line(state, panel_state->window, 1, 1, NCURSES_TEXT_BLUE);
    wnoutrefresh(panel_state->window);
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% THREAD %%%%%%%%%%%%%%%%%%%%%%%
void MainFrame::paint_threads()
{
    box(panel_thread->window, ACS_VLINE, ACS_HLINE);
    write_line("THREADS STATE", panel_thread->window, 0, 1, NCURSES_TEXT_DEFAULT);
    write_line("Engine", panel_thread->window, 1, 1, NCURSES_TEXT_DEFAULT);
    write_line("X", panel_thread->window, 1, 8, thread_state["Engine"]);
    
    write_line("Keyboard", panel_thread->window, 1, 10, NCURSES_TEXT_DEFAULT); // Trick to win time
    write_line("X", panel_thread->window, 1, 19, thread_state["Keyboard"]);

    //write_line("Server_in", panel_thread->window, 1, 20, thread_state["Server_in"]);
    //write_line("Server_out", panel_thread->window, 1, 30, thread_state["Server_out"]);

    wrefresh(panel_thread->window);
    wnoutrefresh(panel_thread->window); 
}
void MainFrame::setThreadState(std::map<std::string, int> new_thread_state){
    thread_state = new_thread_state;
}
void MainFrame::setElement_ThreadState(std::string key, int value){
    thread_state[key] = value;
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% COMM %%%%%%%%%%%%%%%%%%%%%%%
void MainFrame::paint_comm()
{
    box(panel_comm->window, ACS_VLINE, ACS_HLINE);
    mvwprintw(panel_comm->window, 0, 1, "COMMUNICATION");

    wrefresh(panel_comm->window);
    wnoutrefresh(panel_comm->window); 
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOGGER %%%%%%%%%%%%%%%%%%%%%%
void MainFrame::paint_log()
{
    box(panel_log->window, ACS_VLINE, ACS_HLINE);
    mvwprintw(panel_log->window, 0, 1, "LOGS");
    refresh_log();
}
void MainFrame::refresh_log()
{
    int cpt_lines = 1;    
    int lines_max = 0;
    int cols_max = 0;
    std::string buffer1="";

    getmaxyx(panel_log->window, lines_max, cols_max);

    for( std::pair<std::string, int> log : log_list )
    {
        //erase_line(panel_log, cpt_lines);
        write_line(log.first, panel_log->window, cpt_lines, 1, log.second);

        if(cpt_lines>=lines_max-2) break;
        cpt_lines++;
    }
    wrefresh(panel_log->window);
    wnoutrefresh(panel_log->window); 
}
/**
 * The log are sort to 
 */
void MainFrame::add_log(std::string text, int color)
{
    if(log_list.size()>NCURSES_LOG_MAX){
        log_list.erase(log_list.begin());
    }
    log_list.insert(log_list.end(), std::make_pair(text, color));

    werase(panel_log->window);
    paint_log();
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%% PRIMITIVE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void MainFrame::write_line(std::string text, WINDOW *window, int lines, int cols , int color)
{  
    wattron(window, COLOR_PAIR(color));
    mvwprintw(window, lines, cols, text.c_str());
    wattroff(window, COLOR_PAIR(color));
}

void MainFrame::erase_line(WINDOW *window, int lines)
{
    int lines_max = 0;
    int cols_max = 0;
    std::string buffer1="";

    getmaxyx(window, lines_max, cols_max);

    for(int i=0; i<lines_max-2; i++)
    {
        buffer1+=" ";
    }

    mvwprintw(window, lines, 1, buffer1.c_str());
}


void MainFrame::quit_menu()
{
    exit(0);
}

void MainFrame::run() 
{
    long long currentThreadDelay;
    std::string buffer1 = "";

    gettimeofday(&begin, 0);
    gettimeofday(&front_checkpoint, 0);

    while(isRunFlag())
    {
        usleep(task_period);

        gettimeofday(&end_checkpoint, 0);
        currentThreadDelay=(end_checkpoint.tv_sec-front_checkpoint.tv_sec) * 1000000L + (end_checkpoint.tv_usec-front_checkpoint.tv_usec);

        if (currentThreadDelay > task_period )
        {
            gettimeofday(&front_checkpoint, 0);

            if (currentThreadDelay > task_period + task_deadline)
            {
            }
            else 
            {
                repaint();
            }
        }
    }
}

int MainFrame::colornum(int fg, int bg)
{
    int B, bbb, ffff;

    B = 1 << 7;
    bbb = (7 & bg) << 4;
    ffff = 7 & fg;

    return (B | bbb | ffff);
}

void MainFrame::setCommunicationType(int communication_type)
{
    MainFrame::communication_type = communication_type;
}

int MainFrame::getCommunicationType()
{
    return MainFrame::communication_type;
}


