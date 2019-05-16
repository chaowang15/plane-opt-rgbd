#ifndef TOOLS_H
#define TOOLS_H

#include <iostream>
#include <string>

//! Print text in color in the console
/*!
  Ref: https://misc.flogisoft.com/bash/tip_colors_and_formatting
*/
inline void printInColor(const std::string& str, const std::string& color)
{
    std::string prefix("\e[1;37m"), suffix("\e[0m");
    if (color == "red")
        prefix[5] = '1';
    else if (color == "green")
        prefix[5] = '2';
    else if (color == "yellow")
        prefix[5] = '3';
    else if (color == "blue")
        prefix[5] = '4';
    else if (color == "magenta")
        prefix[5] = '5';
    else if (color == "cyan")
        prefix[5] = '6';
    else if (color == "white")
        prefix[5] = '7';
    else
        prefix = suffix = "";
    cout << prefix << str << suffix << endl;
}

inline void printInRed(const std::string& str)
{
    printInColor(str, "red");
}

inline void printInGreen(const std::string& str)
{
    printInColor(str, "green");
}

inline void printInYellow(const std::string& str)
{
    printInColor(str, "yellow");
}

inline void printInBlue(const std::string& str)
{
    printInColor(str, "blue");
}

inline void printInMagenta(const std::string& str)
{
    printInColor(str, "magenta");
}

inline void printInCyan(const std::string& str)
{
    printInColor(str, "cyan");
}

#endif
