#ifndef EXAMPLE_PATH_PLANNER_PLUGIN__VISIBILITY_CONTROL_H_
#define EXAMPLE_PATH_PLANNER_PLUGIN__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define EXAMPLE_PATH_PLANNER_PLUGIN_EXPORT __attribute__ ((dllexport))
    #define EXAMPLE_PATH_PLANNER_PLUGIN_IMPORT __attribute__ ((dllimport))
  #else
    #define EXAMPLE_PATH_PLANNER_PLUGIN_EXPORT __declspec(dllexport)
    #define EXAMPLE_PATH_PLANNER_PLUGIN_IMPORT __declspec(dllimport)
  #endif
  #ifdef EXAMPLE_PATH_PLANNER_PLUGIN_BUILDING_LIBRARY
    #define EXAMPLE_PATH_PLANNER_PLUGIN_PUBLIC EXAMPLE_PATH_PLANNER_PLUGIN_EXPORT
  #else
    #define EXAMPLE_PATH_PLANNER_PLUGIN_PUBLIC EXAMPLE_PATH_PLANNER_PLUGIN_IMPORT
  #endif
  #define EXAMPLE_PATH_PLANNER_PLUGIN_PUBLIC_TYPE EXAMPLE_PATH_PLANNER_PLUGIN_PUBLIC
  #define EXAMPLE_PATH_PLANNER_PLUGIN_LOCAL
#else
  #define EXAMPLE_PATH_PLANNER_PLUGIN_EXPORT __attribute__ ((visibility("default")))
  #define EXAMPLE_PATH_PLANNER_PLUGIN_IMPORT
  #if __GNUC__ >= 4
    #define EXAMPLE_PATH_PLANNER_PLUGIN_PUBLIC __attribute__ ((visibility("default")))
    #define EXAMPLE_PATH_PLANNER_PLUGIN_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define EXAMPLE_PATH_PLANNER_PLUGIN_PUBLIC
    #define EXAMPLE_PATH_PLANNER_PLUGIN_LOCAL
  #endif
  #define EXAMPLE_PATH_PLANNER_PLUGIN_PUBLIC_TYPE
#endif

#endif  // EXAMPLE_PATH_PLANNER_PLUGIN__VISIBILITY_CONTROL_H_
