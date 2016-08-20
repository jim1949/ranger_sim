% sick_data=load('pedestrains.dat');
% pedestrains_readings.write("%2.6f %2.6f %2.6f %2.6f %2.6f %2.6f \n" %(rospy.get_time(), car_x, car_y, car_theta, car_ctrlSpeed, car_ctrlSteer))
% if not readings:
%        for value in readings:
%                 pedestrains_readings.write("%2.4f, " %value)
%         pedestrains_readings.write("\n")
t=sick_data(:,20);
car_x=sick_data(:,21);
car_y=sick_data(:,22);
car_theta=sick_data(:,23);
car_ctrlSpeed=sick_data(:,24);
car_ctrlSteer=sick_data(:,25);
s1=sick_data(:,1);
s2=sick_data(:,2);
s3=sick_data(:,3);
s4=sick_data(:,4);
s5=sick_data(:,5);
s6=sick_data(:,6);
s7=sick_data(:,7);
s8=sick_data(:,8);
s9=sick_data(:,9);
s10=sick_data(:,10);
s11=sick_data(:,11);
s12=sick_data(:,12);
s13=sick_data(:,13);
s14=sick_data(:,14);
s15=sick_data(:,15);
s16=sick_data(:,16);
s17=sick_data(:,17);
s18=sick_data(:,18);
s19=sick_data(:,19);



for m=1:length(s1)
    for n=1:19
    if sick_data(m,n)<15
        x(m,n)=sick_data(m,n)*cos(5*pi*(n-1)/180);
        y(m,n)=sick_data(m,n)*sin(5*pi*(n-1)/180);
%             x(m,n)=x(m,n)*cos(car_ctrlSteer(m))+car_x(m);
%             y(m,n)=y(m,n)*sin(car_ctrlSteer(m))+car_y(m);
        x(m,n)=sick_data(m,n)*cos(5*pi*(n-1)/180+car_theta(m))+car_x(m);
        y(m,n)=sick_data(m,n)*sin(5*pi*(n-1)/180+car_theta(m))+car_y(m);
%             x(m,n)=car_x(m);
%             y(m,n)=car_y(m);
        x1(m,n)=min(sick_data(m,:))*cos(5*pi*(n-1)/180+car_theta(m))+car_x(m);
        y1(m,n)=min(sick_data(m,:))*sin(5*pi*(n-1)/180+car_theta(m))+car_y(m);
    else
        x(m,n)=0;
        y(m,n)=0;
        x1(m,n)=0;
        y1(m,n)=0;
    end

    end
    
end


figure(1)
plot3(x(:,1),y(:,1),t,x(:,2),y(:,2),t,x(:,3),y(:,3),t,x(:,4),y(:,4),t,x(:,5),y(:,5),t,x(:,6),y(:,6),t,x(:,7),y(:,7),t,x(:,8),y(:,8),t,x(:,9),y(:,9),t,x(:,10),y(:,10),t,......
    x(:,11),y(:,11),t,x(:,12),y(:,12),t,x(:,13),y(:,13),t,x(:,14),y(:,14),t,x(:,15),y(:,15),t,x(:,16),y(:,16),t,x(:,17),y(:,17),t,x(:,18),y(:,18),t,x(:,19),y(:,19),t)


figure(2)
plot(car_x,car_y)

figure(3)
plot(x(:,1),y(:,1), x(:,2),y(:,2), x(:,3),y(:,3), x(:,4),y(:,4), x(:,5),y(:,5), x(:,6),y(:,6), x(:,7),y(:,7), x(:,8),y(:,8), x(:,9),y(:,9), x(:,10),y(:,10), x(:,11),y(:,11), x(:,12),y(:,12), x(:,13),y(:,13), x(:,14),y(:,14), x(:,15),y(:,15), x(:,16),y(:,16), x(:,17),y(:,17), x(:,18),y(:,18), x(:,19),y(:,19))

figure(4)
plot3(x1(:,1),y1(:,1),t,x1(:,2),y1(:,2),t,x1(:,3),y1(:,3),t,x1(:,4),y1(:,4),t,x1(:,5),y1(:,5),t,x1(:,6),y1(:,6),t,x1(:,7),y1(:,7),t,x1(:,8),y1(:,8),t,x1(:,9),y1(:,9),t,x1(:,10),y1(:,10),t,......
    x1(:,11),y1(:,11),t,x1(:,12),y1(:,12),t,x1(:,13),y1(:,13),t,x1(:,14),y1(:,14),t,x1(:,15),y1(:,15),t,x1(:,16),y1(:,16),t,x1(:,17),y1(:,17),t,x1(:,18),y1(:,18),t,x1(:,19),y1(:,19),t)

% v=sick_data(:,5)
% % plot3(s2,v,t)
% plot(t,v)
