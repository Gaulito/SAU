
#ifndef INC_TRAJECTORY_H_
#define INC_TRAJECTORY_H_

typedef struct {
	float x;
	float y;
	float z;
	float phi;
	float Rxz;
	float alpha;
	float Ry;
	char type;
} POINT_TR;

void Vslet_Trajectory(double alpha0, double phi0, double x0, double y0, double z0);

POINT_TR Get_point_traj(int Npoint, char UpT);


#endif /* INC_TRAJECTORY_H_ */
