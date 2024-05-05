# aruco
This project was made during Robonautica competition held by IISc Bangalore which mainly involved using ros ,gazebo sim for slam and path planning
PRoject involves using a tortoise bot designed by Rigbetel labs to navigate around the environmet and scan the aruco markers and store the pose of robot to be used as waypoints in future navigation task
Then implemented slam using gmapping and saved the map
Used the map to implement move_base in ros for obstacle avoidance and follow the waypoints which were stored while scanning the aruco markers
