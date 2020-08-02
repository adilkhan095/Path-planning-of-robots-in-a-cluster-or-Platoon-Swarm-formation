function[t,x] = Exam1_22()
%defining the values of goals, obstacles, kg and ko
time= [0 2045];
x0= [0;0;0];
xG= [10;10];
x1= [3;4];
x2= [8;5];
KG= 25;
Ko= 55;
% using the ode23 solver to call the function from 'Exam1_2'
[t,x]= ode23('Exam1_2',time,x0);
% plotting the resulting trajectory in (x,y) plane 
plot(x(:,1),x(:,2),'k',xG(1),xG(2),'*g',x1(1),x1(2),'xb',x2(1),x2(2),'xb')
axis([0 14 0 14])
xlabel('x axis')
ylabel('y axis')
legend('path of the robot')
end