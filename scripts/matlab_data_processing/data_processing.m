sick_data=load('sick_data.dat');
t=sick_data(:,1);
s1=sick_data(:,2);
s2=sick_data(:,3);
s3=sick_data(:,4);
s4=sick_data(:,5);
s5=sick_data(:,6);
s6=sick_data(:,7);
s7=sick_data(:,8);
s8=sick_data(:,9);
s9=sick_data(:,10);
s10=sick_data(:,11);
s11=sick_data(:,12);
s12=sick_data(:,13);
s13=sick_data(:,14);
s14=sick_data(:,15);
s15=sick_data(:,16);
s16=sick_data(:,17);
s17=sick_data(:,18);
s18=sick_data(:,19);
s19=sick_data(:,20);


for n=1:19
    x(:,n)=sick_data(:,n+1)*cos(5*pi*(n-1)/180);
    y(:,n)=sick_data(:,n+1)*sin(5*pi*(n-1)/180);
end

plot3(x(:,1),y(:,1),t,x(:,2),y(:,2),t,x(:,3),y(:,3),t,x(:,4),y(:,4),t,x(:,5),y(:,5),t,x(:,6),y(:,6),t,x(:,7),y(:,7),t,x(:,8),y(:,8),t,x(:,9),y(:,9),t,x(:,10),y(:,10),t,......
    x(:,11),y(:,11),t,x(:,12),y(:,12),t,x(:,13),y(:,13),t,x(:,14),y(:,14),t,x(:,15),y(:,15),t,x(:,16),y(:,16),t,x(:,17),y(:,17),t,x(:,18),y(:,18),t,x(:,19),y(:,19),t)

