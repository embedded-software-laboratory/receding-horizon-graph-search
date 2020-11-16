function [g_value, h_value] = calculate_next_values_reference(g_value, init_pose, target_pose, next_pose)
    g_value = g_value + distance_reference(init_pose, target_pose, next_pose);
    h_value = 0;
end