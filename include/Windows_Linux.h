#pragma once
#ifndef WINDOWS_LINUX_H
#define WINDOWS_LINUX_H

//#define SINGLE_ROBOT_MODE
//#define WINDOWS
#define LINUX

#ifdef WINDOWS
#include <direct.h>
#else 
#include <sys/stat.h>
#include <sys/types.h>
#include <cstddef>
#endif

#include <iostream>
//#define ENABLE_FULL_CHECKING

#endif // !WINDOWS_LINUX_H

