#ifndef TOOLS_H
#define TOOLS_H

#include <iostream>
#include <string>
#include <stdio.h>

//! Print text in color in the console. Only works in Linux.
/*!
  Ref: https://misc.flogisoft.com/bash/tip_colors_and_formatting
*/
#define PRINT_RED(...)        \
    {                         \
        printf("\033[1;31m"); \
        printf(__VA_ARGS__);  \
        printf("\033[0m\n");    \
    }

#define PRINT_GREEN(...)      \
    {                         \
        printf("\033[1;32m"); \
        printf(__VA_ARGS__);  \
        printf("\033[0m\n");    \
    }

#define PRINT_YELLOW(...)     \
    {                         \
        printf("\033[1;33m"); \
        printf(__VA_ARGS__);  \
        printf("\033[0m\n");    \
    }

#define PRINT_BLUE(...)       \
    {                         \
        printf("\033[1;34m"); \
        printf(__VA_ARGS__);  \
        printf("\033[0m\n");    \
    }
#define PRINT_MAGENTA(...)    \
    {                         \
        printf("\033[1;35m"); \
        printf(__VA_ARGS__);  \
        printf("\033[0m\n");    \
    }

#define PRINT_CYAN(...)       \
    {                         \
        printf("\033[1;36m"); \
        printf(__VA_ARGS__);  \
        printf("\033[0m\n");    \
    }

//! Print a progress bar with given progress. Remember to print a new line after progress reaches 1.0
inline void printProgressBar(float progress)
{
    int bar_width = 70;
    std::cout << "\r[";
    int pos = static_cast<int>(bar_width * progress);
    for (int i = 0; i < bar_width; ++i)
    {
        if (i < pos)
            std::cout << "=";
        else if (i == pos)
            std::cout << ">";
        else
            std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << " %" << std::flush;
    if (progress == 1.0)
        std::cout << std::endl;
}

#endif
