#euler from quaterion function
def eulerfromquaterion(q0,q1,q2,q3):
	yaw=math.atan2(2*(q0*q3+q1*q2),1-2*(q2^2+q3^2))
	pitch=math.asin(2*(q0*q2-q3*q1))
	roll=math.atan2(2*(q0*q1+q2*q3),1-2*(q1^2+q2^2))
	return yaw,pitch,roll

yaw,pitch,roll = eulerfromquaterion(orientation.x,orientation.y,orientation.z,orientation.w)