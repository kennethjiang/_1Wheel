#ifndef _MEGA_MOTO_H
#define _MEGA_MOTO_H

// Should be called from main program's setup()
void megaMotoSetup();

// duty: 0-255, 0: stop; 255: maximum speed
void forward(int duty);

// duty: 0-255, 0: stop; 255: maximum speed
void reverse(int duty);

#endif
