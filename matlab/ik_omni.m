th=0;
v=10;
l=.1;
Rot=[cos(th) -sin(th) 0;sin(th) cos(th) 0;0 0 1]
J=[sqrt(3)/2 -.5 -l;0 1 -l;-sqrt(3)/2 -.5 -l]
e_dot=[v*cos(th) v*sin(th) th]
a= J*Rot
V=a*e_dot
%%
th=0;
d=-10;
l=.1;
x=d*cosd(th);
y=d*sind(th);
d1=(x*((sqrt(3)/2)*cosd(th))-(.5*sind(th)))-(y*((sqrt(3)/2)*sind(th)+(.5*cosd(th))))-(l*th)
d2=(x*sind(th))+(y*cosd(th))-(l*th)
d3=(-x*((sqrt(3)/2)*cosd(th))-(.5*sind(th)))+(y*((sqrt(3)/2)*sind(th)+(.5*cosd(th))))-(l*th)
%%
d1=-8.66;
d2=0;
d3=8.66;
th=0;
l=.1;
x=(d1*((cosd(th)/sqrt(3))-(sind(th)/3)))+(d2*(2*sind(th)/3))-(d3*((cosd(th)/sqrt(3))+(sind(th)/3)))
y=(d1*((-sind(th)/sqrt(3))-(cosd(th)/3)))+(d2*2*cosd(th)/3)+(d3*((sind(th)/sqrt(3))-(cosd(th)/3)))
th=(-1/(3*l))*(d1+d2+d3)