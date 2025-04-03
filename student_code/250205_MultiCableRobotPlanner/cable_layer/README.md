# Cable layer

Costmap layer plugin that marks the cable positions behind the robots on an additional costmap layer.

## Using the cable layer

For using the cable layer plugin, it must be added to the plugins in the global costmap config in /path/to/workspace/src/match_mobile_robotics/mir_navigation/config/Costmap/costmap_global_params.yaml

To add the plugin, insert the following two lines into costmap_global_params.yaml before the inflation layer:

    - name: cable_layer
      type: "cable_layer_namespace::CableLayer" # Custom Cable Layer Plugin 
