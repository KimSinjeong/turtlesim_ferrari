#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>

// GLobal termios structs
static struct termios old_tio;
static struct termios new_tio;

// Initialize new terminal i/o settings
void init_termios(int echo)
{
    tcgetattr(0, &old_tio); // Grab old_tio terminal i/o setting
    new_tio = old_tio; // Copy old_tio to new_tio
    new_tio.c_lflag &= ~ICANON; // disable buffered i/o
    new_tio.c_lflag &= echo? ECHO : ~ECHO; // Set echo mode
    if (tcsetattr(0, TCSANOW, &new_tio) < 0) perror("tcsetattr ~ICANON");
    // Set new_tio terminal i/o setting
}

// Restore old terminal i/o settings
void reset_termios(void)
{
    tcsetattr(0, TCSANOW, &old_tio);
}

// Read one character without Enter key: Blocking
char getch(void)
{
    char ch = 0;
    if (read(0, &ch, 1) < 0) perror("Read Error"); // Read one character
    return ch;
}
