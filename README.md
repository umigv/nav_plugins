# nav_plugins

This repository contains the plugins used by `nav_infrastructure`. 

## Creating a planner plugin
The `planner_server` node within `nav_infrastructure` dynamically loads a planner plugin at runtime, making it easy to swap between different path planning algorithms. Here's how to create a path planner plugin:
- Run the creation script
    ```bash
    chmod u+x create_planner.sh
    ./create_planner <base_name>
    ```
    The base name should be in lower_snake_case and include only the planner name. For example, if you want to create my_test_planner, run:
    ```
    ./create_planner my_test
    ```
- Go into `<name>_planner.cpp`, Using `straight_line_planner.cpp` or `bfs_planner.cpp` as reference, implement the `find_path()` function
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
