function sdpPP = poly_occlusion(h1, h2, h3)

% order of polynomial 
d=15;
d_sos=d;



% variables
sdpvar x y

% polynomial  with unknown coefficients coef
[P,co_p] = polynomial([x y],d);

% Simple Box
B1=(x+1)*(1-x); 
B2=(y+1)*(1-y);

% Compute Occlusion Space
% observation point
%xo = 2; yo = 3.3;
% circle
%r = 0.5; xc = 1.0; yc = 2.0;

%[h1, h2, h3] = occlusion_space(xo, yo, r, xc, yc);
%[b1, b2, b3] = occlusion_space(xo, yo, r, 3, 2);

%for i = 1:1000
%    x = rand(2, 1) * 5;
%    if h1(x(1), x(2)) > 0 && h2(x(1), x(2)) > 0 && h3(x(1), x(2)) > 0
%        scatter(x(1), x(2), 'r');
%    end
%    if b1(x(1), x(2)) > 0 && b2(x(1), x(2)) > 0 && b3(x(1), x(2)) > 0
%        scatter(x(1), x(2), 'b');
%    end
%end


% Generate Occluded Points
s=3000;
xs=rand(1, s) * 5;
ys=rand(1, s) * 5;

%occluded = or((and(and(h1(xs, ys) > 0, h2(xs, ys) > 0), h3(xs, ys) > 0)), ...
%           (and(and(b1(xs, ys) > 0, b2(xs, ys) > 0), b3(xs, ys) > 0)));
occluded = and(and(h1(xs, ys) > 0, h2(xs, ys) > 0), h3(xs, ys) > 0);

xs=xs(occluded)/5;ys=ys(occluded)/5;
plot(xs,ys,'.');hold on
xlim([0, 1])
ylim([0, 1])

% Linear Constraints
F=[];pow=[];for i=0:d; pow=[pow;genpow(2,i)];end
for i=1:size(xs,2)
F=[F, sum(co_p.*(xs(i).^pow(:,1)).*(ys(i).^pow(:,2)))>=1];
end

% SOS Constraints
[s1,c1] = polynomial([x y],d_sos);
[s2,c2] = polynomial([x y],d_sos);
F = [F, sos(P-[s1]*B1-[s2]*B2), sos(s1), sos(s2)];

% Integral
Int=int(P,[x y],[-1 -1],[1 1]);


ops = sdpsettings('solver','mosek');
[sol,v,Q]=solvesos(F, Int,[],[c1;c2;co_p]);


%% Results
% coefficients of polynomial
C=value(co_p');

% Polynomial
syms x y
PP= sum(C'.*(x).^pow(:,1).*(y).^pow(:,2));
[x,y]=meshgrid([-0.99:0.01:0.99],[-0.99:0.01:0.99]);
surf(x,y,eval(PP),'FaceColor','blue','FaceAlpha',0.5,'EdgeColor','none','FaceLighting','phong');hold on
camlight; lighting gouraud

[x,y]=meshgrid([-0.99:0.01:0.99],[-0.99:0.01:0.99]);
surf(x,y,ones(size(eval(PP))),'FaceColor','red','FaceAlpha',0.5,'EdgeColor','none','FaceLighting','phong');hold on
contour(x,y,eval(PP),'ShowText','on')

figure(2);plot(xs,ys,'.');hold on
contour(x,y,eval(PP),[1 1],'--rs',...
    'LineWidth',2)

sdpvar x y
sdpPP= sum(C'.*(x).^pow(:,1).*(y).^pow(:,2));

end