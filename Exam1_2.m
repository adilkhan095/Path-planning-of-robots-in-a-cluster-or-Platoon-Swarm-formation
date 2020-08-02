function dx= Exam1_2(~,x)
% defining the values of goals, obstacles, kg and ko
xG= [10;10];
x1= [3;4];
x2= [8;5];
KG= 85;
Ko= 50;
%calculating the range and the attractive forces 
rG= sqrt((xG(1)+x(1))^2+(xG(2)-x(2))^2);
FGx= KG*(xG(1)-x(1))/rG;
FGy= KG*(xG(2)-x(2))/rG;
%calculating the range and the repulsive forces wrt to first obstacle
ri1= sqrt((x1(1)-x(1))^2+(x1(2)-x(2))^2);
Fi1x= -Ko*(x1(1)-x(1))/ri1^2;
Fi1y= -Ko*(x1(2)-x(2))/ri1^2;
%calculating the range and the repulsive forces wrt to the 2nd obstacle
ri2= sqrt((x2(1)-x(1))^2+(x2(2)-x(2))^2);
Fi2x= -Ko*(x2(1)-x(1))/ri2^2;
Fi2y= -Ko*(x2(2)-x(2))/ri2^2;
%cummulating the forces
Fx= (FGx+Fi1x+Fi2x);
Fy= (FGy+Fi1y+Fi2y);
%angle of the force as seen from the robot 
alpha= atan(Fy/Fx);
%defining the values of wheel speed 'v', wheel base 'L'& 'K'
v=1; L=2;
K=3;
%Taking the value of phi where theta is x(3)
ph= K*(alpha-x(3));
%defining the dynamics of xdot, ydot and theta dot
dx=[v*cos(ph)*cos(x(3));v*cos(ph)*sin(x(3));v*sin(ph)/L];
end


