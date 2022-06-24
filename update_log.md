This file is used to record the update events.

Format: date, author.

Example: 2022-06-19, Jianye
# 2022-06-23, David and Jianye
- Created two functions `priority_assignment` and `priority_assignment_parl` to assign priorities for different strategies.
- Adapted `pb_controller_parl` for different priority assignment strategies

# 2022-06-23, Jianye
- Add more elements to ui
- Adapt trim set ID to be successive. The ID of newly added trim should also have successive number to the existing IDs!

# 2022-06-22, Jianye and David
- Let circle scenario run

# 2022-06-19, David
- Changes for CPM Lab mode
    - Implemented automated path update for autonomous and manual vehicles
    - Added first mixed traffic collision avoidance mode: manual vehicles get highest priority, collision avoidance based on coupling adjacency
    - Added second mixed traffic collision avoidance mode: new controller `pb_controller_mixed_traffic.m`
        - autonomous vehicles add reachable set of manual vehicles as dynamic obstacles
        - collision avoidance between autonomous vehicles based on coupling adjacency
    - New class `G29ForceFeedback.m`: send custom ROS messages to Logitech G29 to set position and torque

# 2022-06-19, Jianye
- Turned `get_road_data.m` to a class called `RoadData`, while the functionality is unchanged, but returns a object of this class containing all the necessary road information and some methods are added.
- Created a class called `ControllResultsInfo`, which is used to initialize the variable `info` which is used to store the results of sub-controller.
- Improved the fallback function called `pb_controller_fallback.m`
    - Before improvement: one vehicle triggers fallback, all other vehicles must take fallback
    - After improvement: only the vehicle triggering fallback and other vehicles which have directed or undirected couplings with this vehicle will take fallbacks. All other vehicles plan trajectory in a normal way.