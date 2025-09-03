#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define TestLoadRobotModule_DLLIMPORT __declspec(dllimport)
#  define TestLoadRobotModule_DLLEXPORT __declspec(dllexport)
#  define TestLoadRobotModule_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define TestLoadRobotModule_DLLIMPORT __attribute__((visibility("default")))
#    define TestLoadRobotModule_DLLEXPORT __attribute__((visibility("default")))
#    define TestLoadRobotModule_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define TestLoadRobotModule_DLLIMPORT
#    define TestLoadRobotModule_DLLEXPORT
#    define TestLoadRobotModule_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef TestLoadRobotModule_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define TestLoadRobotModule_DLLAPI
#  define TestLoadRobotModule_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef TestLoadRobotModule_EXPORTS
#    define TestLoadRobotModule_DLLAPI TestLoadRobotModule_DLLEXPORT
#  else
#    define TestLoadRobotModule_DLLAPI TestLoadRobotModule_DLLIMPORT
#  endif // TestLoadRobotModule_EXPORTS
#  define TestLoadRobotModule_LOCAL TestLoadRobotModule_DLLLOCAL
#endif // TestLoadRobotModule_STATIC