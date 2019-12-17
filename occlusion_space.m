function [h1, h2, h3] = plot_occlusion_space(xo, yo, r, xc, yc)
    %g_obs=@(x, y) (r^2-(x - xc)^2 - (y - yc)^2) ;

    % define tangent lines
    d = ((xo-xc)^2 + (yo-yc)^2)^0.5;
    l = (d^2 - r^2)^0.5;

    [xout, yout] = circcirc(xo, yo, l, xc, yc, r) %intersection on two circles

    
    %theta1 = atan2(yout(1)-yo, xout(1)-xo);
    %theta2 = atan2(yout(2)-yo, xout(2)-xo);
    %thetas = sort([theta1, theta2]);
    %h1 = @(x, y) atan2((y-yo),(x-xo)) - thetas(1);
    %h2 = @(x, y) -atan2((y-yo),(x-xo)) + thetas(2);
    %h3 = @(x, y) (y-yo)^2 + (x-xo)^2 - l^2;
    
    %k1 = (yout(1)-yo)/(xout(1)-xo);
    %k2 = (yout(2)-yo)/(xout(2)-xo);
    %ks = sort([k1, k2]);
    %h1 = @(x, y) (y-yo)/(x-xo) - ks(1) * (x-xo);
    %h2 = @(x, y) -(y-yo) + ks(2) * (x-xo);
    %h3 = @(x, y) (y-yo)^2 + (x-xo)^2 - l^2;
    
    h1 = @(x, y) - (xout(1)-xo) * (y-yo) + (yout(1)-yo) * (x-xo)
    h2 = @(x, y) (xout(2)-xo) * (y-yo) - (yout(2)-yo) * (x-xo)
    h3 = @(x, y) (xout(2)-xout(1)) * (y-yout(1)) - (yout(2)-yout(1)) * (x-xout(1))
    %h3 = @(x, y) (y-yo)^2 + (x-xo)^2 - l^2;
    
    for i = 1:1000
    x = rand(2, 1);
    if h1(x(1), x(2)) > 0 && h2(x(1), x(2)) > 0 && h3(x(1), x(2)) > 0
        scatter(x(1), x(2), 'r');
    end
    end
    
    
    sdpvar x y
    h1 =  - (xout(1)-xo) * (y-yo) + (yout(1)-yo) * (x-xo);
    h2 = (xout(2)-xo) * (y-yo) - (yout(2)-yo) * (x-xo);
    h3 = (xout(2)-xout(1)) * (y-yout(1)) - (yout(2)-yout(1)) * (x-xout(1));

    
end