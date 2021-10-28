/**
 *  @author Sylvain Colomer
 *  @date 18/04/19.
 */

#ifndef CONSTANT_H
#define CONSTANT_H

// The following two non-standard baudrates should have been defined by the system
// If not, just fallback to number
#ifndef B460800
#define B460800 460800
#endif

#ifndef B921600
#define B921600 921600
#endif

// Status flags
#define SERIAL_PORT_OPEN   1;
#define SERIAL_PORT_CLOSED 0;
#define SERIAL_PORT_ERROR -1;

#endif 