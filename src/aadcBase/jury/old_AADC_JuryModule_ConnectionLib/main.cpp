/**
 *
 * Implementing a very simple example to show the creation and general usage
 * of a system in terms of the connection library.
 *
 * @file
 * Copyright &copy; Audi Electronics Venture GmbH. All rights reserved
 *
 * $Author: spiesra $
 * $Date: 2017-04-26 13:57:27 +0200 (Mi, 26 Apr 2017) $
 * $Revision: 62571 $
 *
 */

#include <math.h>
#include <string.h> //memset under Linux
#include <iostream>
#ifdef WIN32
    #include <windows.h>
#else
    #include <unistd.h>
#endif
#include "stdafx.h"     //necessary includes and use namespace connectionlib

#include "./coder_description.h"
#include "cJuryModule.h"
#include <QApplication>






int main(int argc, char* argv[])
{
    QApplication a(argc, argv);
    cJuryModule oJuryModule;

    oJuryModule.show();
    return a.exec();
    
}
