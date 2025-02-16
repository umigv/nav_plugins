#ifndef ASTAR_PLUGIN__VISIBILITY_CONTROL_H_
#define ASTAR_PLUGIN__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ASTAR_PLUGIN_EXPORT __attribute__ ((dllexport))
    #define ASTAR_PLUGIN_IMPORT __attribute__ ((dllimport))
  #else
    #define ASTAR_PLUGIN_EXPORT __declspec(dllexport)
    #define ASTAR_PLUGIN_IMPORT __declspec(dllimport)
  #endif
  #ifdef ASTAR_PLUGIN_BUILDING_LIBRARY
    #define ASTAR_PLUGIN_PUBLIC ASTAR_PLUGIN_EXPORT
  #else
    #define ASTAR_PLUGIN_PUBLIC ASTAR_PLUGIN_IMPORT
  #endif
  #define ASTAR_PLUGIN_PUBLIC_TYPE ASTAR_PLUGIN_PUBLIC
  #define ASTAR_PLUGIN_LOCAL
#else
  #define ASTAR_PLUGIN_EXPORT __attribute__ ((visibility("default")))
  #define ASTAR_PLUGIN_IMPORT
  #if __GNUC__ >= 4
    #define ASTAR_PLUGIN_PUBLIC __attribute__ ((visibility("default")))
    #define ASTAR_PLUGIN_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ASTAR_PLUGIN_PUBLIC
    #define ASTAR_PLUGIN_LOCAL
  #endif
  #define ASTAR_PLUGIN_PUBLIC_TYPE
#endif

#endif  // ASTAR_PLUGIN__VISIBILITY_CONTROL_H_
