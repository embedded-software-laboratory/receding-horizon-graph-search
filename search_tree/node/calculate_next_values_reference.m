function [g_value, h_value] = calculate_next_values_reference(g_value, init_pose, target_pose, next_pose)
    distance = distance_reference(init_pose, target_pose, next_pose);
    g_value = g_value + distance;
    h_value = cost_to_go(init_pose, target_pose, next_pose);
end