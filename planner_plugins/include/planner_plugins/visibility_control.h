#ifndef PLANNER_PLUGINS__VISIBILITY_CONTROL_H_
#define PLANNER_PLUGINS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PLANNER_PLUGINS_EXPORT __attribute__ ((dllexport))
    #define PLANNER_PLUGINS_IMPORT __attribute__ ((dllimport))
  #else
    #define PLANNER_PLUGINS_EXPORT __declspec(dllexport)
    #define PLANNER_PLUGINS_IMPORT __declspec(dllimport)
  #endif
  #ifdef PLANNER_PLUGINS_BUILDING_LIBRARY
    #define PLANNER_PLUGINS_PUBLIC PLANNER_PLUGINS_EXPORT
  #else
    #define PLANNER_PLUGINS_PUBLIC PLANNER_PLUGINS_IMPORT
  #endif
  #define PLANNER_PLUGINS_PUBLIC_TYPE PLANNER_PLUGINS_PUBLIC
  #define PLANNER_PLUGINS_LOCAL
#else
  #define PLANNER_PLUGINS_EXPORT __attribute__ ((visibility("default")))
  #define PLANNER_PLUGINS_IMPORT
  #if __GNUC__ >= 4
    #define PLANNER_PLUGINS_PUBLIC __attribute__ ((visibility("default")))
    #define PLANNER_PLUGINS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PLANNER_PLUGINS_PUBLIC
    #define PLANNER_PLUGINS_LOCAL
  #endif
  #define PLANNER_PLUGINS_PUBLIC_TYPE
#endif

#endif  // PLANNER_PLUGINS__VISIBILITY_CONTROL_H_
