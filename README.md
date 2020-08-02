# Path-planning-of-robots-in-a-cluster-or-Platoon-Swarm-formation
This project was created to as a part of exam for coursework for UTA EE 5322 under Dr Frank Lewis
HOW TO WRITE THE CODE: 
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
Fi1x= -Ko*(x1(1)-x(1))/ri1^3;
Fi1y= -Ko*(x1(2)-x(2))/ri1^3;
%calculating the range and the repulsive forces wrt to the 2nd obstacle
ri2= sqrt((x2(1)-x(1))^2+(x2(2)-x(2))^2);
Fi2x= -Ko*(x2(1)-x(1))/ri2^3;
Fi2y= -Ko*(x2(2)-x(2))/ri2^3;
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




function[t,x] = Exam1_22()
%defining the values of goals, obstacles, kg and ko
time= [0 20];
x0= [0;0;0];
xG= [10;10];
x1= [3;4];
x2= [8;5];
KG= 60;
Ko= 100;
% using the ode23 solver to call the function from 'Exam1_2'
[t,x]= ode23('Exam1_2',time,x0);
% plotting the resulting trajectory in (x,y) plane 
plot(x(:,1),x(:,2),'k',xG(1),xG(2),'*g',x1(1),x1(2),'xb',x2(1),x2(2),'xb')
axis([0 14 0 14])
xlabel('x axis')
ylabel('y axis')
legend('path of the robot')
end
