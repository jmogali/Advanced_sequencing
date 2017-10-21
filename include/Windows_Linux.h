#pragma once
#ifndef WINDOWS_LINUX_H
#define WINDOWS_LINUX_H

#define WINDOWS
//#define LINUX

#ifdef WINDOWS
#include <direct.h>
#else 
#include <sys/stat.h>
#include <sys/types.h>
#include <cstddef>
#endif

#endif // !WINDOWS_LINUX_H

