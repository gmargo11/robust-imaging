% Obstacle set 
g_obs=@(x, y) (0.5^2-(x - 3.0)^2 - (y - 2.0)^2) * ...
              (0.5^2-(x - 1.0)^2 - (y - 2.0)^2);

% Target set
target=[2; 3.3];

% Generate visibility set samples
visible_samples = [target];
for i=1:1000
    sample = rand(2, 1) * 4;
    slope = (sample(2) - target(2)) / (sample(1) - target(1));
    line =@(x) slope * (x - sample(1)) + sample(2);
    options = optimset('Display', 'off');
    [x,fval,exitflag] = fzero(@(x) g_obs(x, line(x)), 3, optimset('Display', 'off'));
    if exitflag ~= 1 || (sample(1) < x && target(1) < x) || (sample(1) > x && target(1) > x) || ...
            (sample(2) < line(x) && target(2) < line(x)) || (sample(2) > line(x) && target(2) > line(x))
        visible_samples = cat(2, visible_samples, sample);
    end
end

figure
hold on;
% plot sampled visible points
scatter(visible_samples(1, :), visible_samples(2, :));

%plot circles
ang=0:0.01:2*pi; 
xp=0.5*cos(ang);
yp=0.5*sin(ang);
plot(3+xp,2+yp);
plot(1+xp, 2+yp);

%fplot(g_obs)