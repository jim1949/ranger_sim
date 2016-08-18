sick_data=load('testdata8.dat');
% results_file_handle.write("%2.6f %2.6f %2.6f %2.6f %2.6f %2.6f \n" %(rospy.get_time(), car_x, car_y, car_theta, car_ctrlSpeed, car_ctrlSteer))

t=sick_data(:,1);
s1=sick_data(:,2);
s2=sick_data(:,3);


% plot3(s1,s2,t)

v=sick_data(:,5)
% plot3(s2,v,t)
plot(t,v)