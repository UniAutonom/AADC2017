/**
 *
 * Standard includes
 *
 * @file
 * Copyright &copy; Audi Electronics Venture GmbH. All rights reserved
 *
 * $Author: spiesra $
 * $Date: 2017-04-26 13:57:27 +0200 (Mi, 26 Apr 2017) $
 * $Revision: 62571 $
 *
 */

#ifndef _STD_INCLUDES_H_
#define _STD_INCLUDES_H_

#include <connectionlib.h>
using namespace connectionlib;

struct tAADC_Maneuver
{
    int nId;
    std::string action;
};

struct tSector
{
    int id;
    std::vector<tAADC_Maneuver> lstManeuvers;
};


#endif

