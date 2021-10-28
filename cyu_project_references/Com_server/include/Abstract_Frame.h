/**
 * @author Sylvain Colomer
 * @date 17/08/18.
 */

#ifndef ABSTRACT_FRAME_H_
#define ABSTRACT_FRAME_H_

#include <iostream>
#include <string>
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <thread>
#include <mutex>


/**
 * Abstract class use to contruct a common structure for display classes. Now only a prototype.
 */
class Abstract_Frame {

private:
    /**
     * Different important vaar about the position of created screen
     */
    int screen_widht = -1;
    int screen_height = -1;
    int screen_x = -1;
    int screen_y = -1;

public :

    /**
     * Default destructor
     */
    virtual ~Abstract_Frame();

    /**
     * Default constructor of a frame. Only get user information about desired frame position and size  
     */
    Abstract_Frame(int screen_widht, int screen_height, int screen_x, int screen_y)
    {
        Abstract_Frame::screen_widht = screen_widht;
        Abstract_Frame::screen_height = screen_height;
        Abstract_Frame::screen_x = screen_x;
        Abstract_Frame::screen_y = screen_y;
    }

    /**
	 * Function that allow to construct with the different function the classes
	 */
    virtual void initFrame() = 0;

    /**
     * Method that set the style of the Frame. Generraly use a constant classes
     */
    virtual void setStyle() = 0;

    /**
     * Method that set the layout of the frame
     */
    virtual void setLayout() = 0;

    /**
     * Method that add object to the principal frame
     */
    virtual void  setComponent() = 0;

    /**
     * Method that add the different action Listener to current component
     */
    virtual void setAction() = 0;

    /**
     * Typical method that allow to display the frame
     */
    virtual void buildFrame() = 0;

    /**
     * Typical method that allow to erase cleanly the frame
     */
    virtual void eraseFrame() = 0;

};

#endif
