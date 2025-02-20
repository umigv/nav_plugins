# nav_plugins

This repository contains the plugins used by `nav_infrastructure`. 

## Creating a planner plugin
The `planner_server` node within `nav_infrastructure` dynamically loads a planner plugin at runtime, making it easy to swap between different path planning algorithms. Here's how to create a path planner plugin:
- Navigate into the `planner_plugins` package:
    ```bash
    cd nav_plugins/planner_plugins
    ```
- Create a new header file for your plugin:
    ```bash
    touch include/planner_plugins/<plugin_name>.hpp
    ```
- Paste the following lines into your new header file, replacing `<PLUGIN_NAME>` and `<PluginName>` with the name of your plugin:
    ```cpp
    #ifndef PLANNER_PLUGINS__<PLUGIN_NAME>_HPP_ // TODO: Edit here!
    #define PLANNER_PLUGINS__<PLUGIN_NAME>_HPP_ // TODO: Edit here!

    #include "planner_plugins/visibility_control.h"
    #include "plugin_base_classes/path_planner.hpp"

    namespace planner_plugins
    {

    class <PluginName> : public plugin_base_classes::PathPlanner // TODO: Edit here!
    {
    public:
        std::vector<infra_common::CellCoordinate> find_path(const infra_common::Costmap &costmap, 
            const std::function<bool(int)> &drivable,
            const infra_common::CellCoordinate &start,
            const infra_common::CellCoordinate &goal) override;
    };

    }  // namespace planner_plugins

    #include <pluginlib/class_list_macros.hpp>

    PLUGINLIB_EXPORT_CLASS(planner_plugins::<PluginName>, plugin_base_classes::PathPlanner) // TODO: Edit here!

    #endif 
    ```
- Open `plugins.xml` and paste the following lines within the `<library>` tags, replacing `<PluginName>` with your plugin name and entering a description of your plugin
    ```xml
    <class type="planner_plugins::<PluginName>" base_class_type="plugin_base_classes::PathPlanner">
        <description>Plugin description here.</description>
  </class>
    ```
- Create a source file for your plugin:
    ```bash
    touch src/<plugin_name>.cpp
    ```
- Open `CMakeLists.txt`, and enter your new source file within the `add_library` call:
    ```cmake
    add_library(${PROJECT_NAME} 
        src/example_planner.cpp
        src/<plugin_name>.cpp
    )
    ```
- Using `example_planner.cpp` or `bfs_planner.cpp` as reference, implement the `find_path()` function
- When your plugin is finished, you'll need to test it using the `planner_server` within the `nav_infrastructure` repository
    - If you haven't already cloned the `nav_infrastructure` repository, do so now: https://github.com/umigv/nav_infrastructure
- Open `nav_infrastructure/planner_server/config/planner_server_params.yaml` and enter your plugin name into the `planner_plugin` parameter:
    ```yaml
    planner_plugin: "planner_plugins::<PluginName>"
    ```
- Now build both the `planner_server` and `planner_plugins` packages
- To test your plugin, run the `planner_server` and call the `navigate_to_goal` action (the `nav_infrastructure` repository contains instructions for this)
    - You should have some sort of unique print statement in your plugin's `find_path` function to verify that it runs correctly when the action is called
    - If the `planner_server` node starts correctly, the action succeeds, and you see your print statement, your plugin should be working correctly