# formation_layer

## Overview

This package contains the code of the costmap layer : **formation_layer**. It is designed to be implemented in the global costmap of robots belonging to a multi-robot formation.  

**Author** : Rayen Hajji 

**Email** : rayen.hajji@stud.uni-hannover.de

## Usage

To add the formation_layer to the global costmap of mir robot as a plugin:
* Go to the robot navigation package `match_mobile_robotics/mir/mir_navigation` 
* In the global costmap paramter file `config/Costmap/costmap_global_params.yaml`, add the formation layer as a plugin, as follows :   
  
  `plugins:` 

    `- name: navigation_map`

      `type: "costmap_2d::StaticLayer"`

    `- name: obstacles`

      `type: "costmap_2d::VoxelLayer"`

    **`- name: formation_layer`**

      `type: "formation_layer_namespace::FormationLayer"`
      
    `- name: inflation`

      `type: "costmap_2d::InflationLayer"` 

## Config files

* **FormationLayer.cfg**: this file contains : 
    * the configuration parameter of the formation layer ('**formation_layer_enabled**')
    * the configuration parameter of the formation properties calculation ('**formation_properties_generated**')
    * the configuration parameters of the transported object and its corner coordinates ('**transport_object_included**'    '**corner_point_1_x**',  '**corner_point_1_x**', etc). 

## Nodes

**Published Topics**

* "**formation_footprint**" (geometry_msgs::PolygonStamped)
    contains a polygon that defines the calculated formation footprint by the layer.
* "**minimum_enclosing_circle**" (visualization_msgs::Marker)
    contains a marker to visualize the minimum enclosing circle of the formation.
* "**minimum_enclosing_circle_center**" (geometry_msgs::PointStamped)
    contains a point that defines the center of the minimum enclosing circle of the formation.    
* "**bounding_box**" (geometry_msgs::PolygonStamped)
    contains a polygon that defines the update bounds of the formaiton layer.
* "**transported_object_corners**" (visualization_msgs::MarkerArray)
    contains an array of markers to visualize the corners of the transported object.

**Service Client/Service**
* "**mec_Info**" (visualization_msgs::MarkerArray)
    provides the center and the radius od the minimum enclosing circle of the formation.

