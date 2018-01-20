install range for scr-104 

rosmake rangee




costmap_common_params.yaml
增加以下内容
sonar_layer:  
  enabled:            true  
  clear_threshold:    0.46  
  mark_threshold:     0.98  
  topics: ["/UltraSoundPublisher"]  
  clear_on_max_reading: true  

local_costmap_params.yaml
增加以下内容
   plugins:
   - {name: static_layer,    type: "costmap_2d::StaticLayer"}
   - {name: obstacle_layer,  type: "costmap_2d::ObstacleLayer"}
   - {name: inflation_layer, type: "costmap_2d::InflationLayer"}
   - {name: sonar_layer,     type: "range_sensor_layer::RangeSensorLayer"}  
