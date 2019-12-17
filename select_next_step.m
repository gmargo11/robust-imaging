function [next_step, is_visible] = select_next_step(vehicle_loc, psi_primitives, step_size, v, max_disturbance, obstacles, xo, yo)

% change coordinate frame so vehicle is at [0, 0, 0]
obstacles = obstacles - [0 vehicle_loc(1) vehicle_loc(2)];
xo = xo - vehicle_loc(1);
yo = yo - vehicle_loc(2);

obstacles = obstacles * [1 0 0;
                         0 cos(-vehicle_loc(3)*pi/180) -sin(-vehicle_loc(3)*pi/180);
                         0 sin(-vehicle_loc(3)*pi/180) cos(-vehicle_loc(3)*pi/180)];
observe_pos = [xo yo] * [cos(-vehicle_loc(3)*pi/180) -sin(-vehicle_loc(3)*pi/180);
                         sin(-vehicle_loc(3)*pi/180) cos(-vehicle_loc(3)*pi/180)];
xo = observe_pos(1); yo = observe_pos(2);

safe_commands = [];
vis_commands = [];
for psi_des = psi_primitives
    flag_safe = true;
    flag_vis = true;
    for obstacle = obstacles.'
        [safe, safetyCert] = check_collision(step_size, v, max_disturbance, psi_des, obstacle);
        [visible, visibilityCert] = check_occlusion(step_size, v, max_disturbance, psi_des, xo, yo, obstacle);
        if ~safe
            flag_safe = false;
            flag_vis = false;
            break;
        end
        if ~visible
            flag_vis = false;
        end
    end
    if flag_safe
        safe_commands = cat(1, safe_commands, psi_des);
        if flag_vis
            vis_commands = cat(1, vis_commands, psi_des);
        end
    end
end

safe_commands
vis_commands

if size(vis_commands) > 0
    next_step = vis_commands(1);
    is_visible = true;
elseif size(safe_commands) > 0
    next_step = safe_commands(1);
    is_visible = false;
else
    next_step = "No robustly safe step found!";
    is_visible = false;
end

end
