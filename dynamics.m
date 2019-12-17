% define initial states
tf=5; y0 = [0;0;0]; v=1; psi_des= 45*pi/180;
tspan = [0 tf]; [t,y] = ode45(@(t,y) [-v*sin(y(3))+0.2*rand-0.1;v*cos(y(3));-50*(y(3)-psi_des)], tspan, y0);
%plot(y(:,1),y(:,2),'LineWidth',2);xlabel('x');ylabel('y');
%hold on; ax = gca; ax.FontSize = 30;
%str1 = '$(x_t,y_t,\psi_t)$';text(0,0,str1,'Interpreter','latex','FontSize',30)


% define dynamics
syms x y psi omega
f1 = -v * sin(psi) + omega;
f2 = v * cos(psi);
u = -50 * (psi-psi_des); f3 = u;
% Taylor expansion of nonlinear dynamics to order 3 polynomial
fT1 = taylor(f1, psi, 'Order', 3);
fT2 = taylor(f2, psi, 'Order', 3);
fT3 = f3;


% prescribe sequence of control inputs
psi_des_cmd = [0.0, 0.0, 0.1, 0.05, 0.0, -0.3];

% simulate path under noisy dynamics
y0 = [0;0;0];
path = [y0];
dt = 1.0;
for i = 1:size(psi_des_cmd, 2)
    tspan = [dt*i-1; dt*i];
    [t,ys] = ode45(@(t,y) [-v*sin(y(3))+0.2*rand-0.1;v*cos(y(3));-50*(y(3)-psi_des_cmd(1, i))], tspan, path(:, end));
    path = cat(2, path, ys');
end
plot(path(1, :), path(2, :))
