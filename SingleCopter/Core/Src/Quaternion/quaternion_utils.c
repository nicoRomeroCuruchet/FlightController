#include <math.h>
#include "Quaternion/quaternion_utils.h"


void get_rotation_from_quaternion(Quaternion* q, float R[3][3])
{
	R[0][0] = 1 - 2*(q->y)*(q->y) - 2*(q->z)*(q->z);
	R[0][1] = 2*(q->x)*(q->y) - 2*(q->w)*(q->z);
	R[0][2] = 2*(q->x)*(q->z) + 2*(q->w)*(q->y);

	R[1][0] = 2*(q->x)*(q->y) + 2*(q->w)*(q->z);
	R[1][1] = 1 - 2*(q->x)*(q->x) - 2*(q->z)*(q->z);
	R[1][2] = 2*(q->y)*(q->z) - 2*(q->w)*(q->x);

	R[2][0] = 2*(q->x)*(q->z) - 2*(q->w)*(q->y);
	R[2][1] = 2*(q->w)*(q->x) + 2*(q->y)*(q->z);
	R[2][2] = 1 - 2*(q->x)*(q->x) - 2*(q->y)*(q->y);
}

void get_rotation_from_euler_angles(float angles[3], float R[3][3])
{
	float phi   = angles[0];
	float theta = angles[1];
	float psi   = angles[2];

	float cos_phi = cosf(phi);
	float sin_phi = sinf(phi);

	float cos_theta = cosf(theta);
	float sin_theta = sinf(theta);

	float cos_psi = cosf(psi);
	float sin_psi = sinf(psi);

	R[0][0] = cos_psi * cos_theta;
	R[0][1] = sin_phi * sin_theta * cos_psi - sin_psi * cos_phi;
	R[0][2] = sin_phi * sin_psi + sin_theta * cos_phi * cos_psi;

	R[1][0] = sin_psi * cos_theta;
	R[1][1] = sin_phi * sin_psi * sin_theta + cos_phi * cos_psi;
	R[1][2] = -sin_phi * cos_psi + sin_psi * sin_theta * cos_phi;

	R[2][0] = -sin_theta;
	R[2][1] = sin_phi * cos_theta;
	R[2][2] = cos_phi * cos_theta;
}


void  get_omega_body_frame(float angles_rates[3], float angles[3], float W[3])
{
	float phi = angles[0];
	float theta = angles [1];

	float sin_phi = sinf(phi);
	float cos_phi = cosf(phi);

	float cos_theta = cosf(theta);
	float sin_theta = sinf(theta);

	float phi_dot   = angles_rates[0];
	float theta_dot = angles_rates[1];
	float psi_dot   = angles_rates[2];

	W[0] = phi_dot - psi_dot*sin_theta;
	W[1] = psi_dot*sin_phi*cos_theta + theta_dot*cos_theta;
	W[2] = psi_dot*cos_phi*cos_theta - theta_dot*sin_phi;
}

// e_r
void get_rotation_error(float R_d[3][3],  float R[3][3], float result[3])
{
	float tmp_res[3][3];
	for(int i=0; i<3;i++)
	{
		for(int j=0; j<3;j++)
		{
			// R_d.T * R:
			float r1 = R_d[0][i]*R[0][j] + R_d[1][i]*R[1][j] + R_d[2][i]*R[2][j];
			// R.T * R_d:
			float r2 = R[0][i]*R_d[0][j] + R[1][i]*R_d[1][j] + R[2][i]*R_d[2][j];
			// R_d.T * R - R.T * R_d:
			tmp_res[i][j] = 0.5*(r1-r2);
		}
	}
	result[0] =  tmp_res[2][1];
	result[1] =  tmp_res[0][2];
	result[2] =  tmp_res[0][1];
}

// e_w
void get_omega_error(float W[3], float W_d[3], float R_d[3][3], float R[3][3], float result[3])
{
	float RTRd [3][3];
	// R.T * R_d:
	for(int i=0; i<3;i++)
	{
		for(int j=0; j<3;j++)
			RTRd[i][j] = R[0][i]*R_d[0][j] + R[1][i]*R_d[1][j] + R[2][i]*R_d[2][j];
	}
	// W - (R.T*R_d)*W_d
	result[0] = W[0] - (RTRd[0][0]*W_d[0] + RTRd[0][1]*W_d[1] + RTRd[0][2]*W_d[2]);
	result[1] = W[1] - (RTRd[1][0]*W_d[0] + RTRd[1][1]*W_d[1] + RTRd[1][2]*W_d[2]);
	result[2] = W[2] - (RTRd[2][0]*W_d[0] + RTRd[2][1]*W_d[1] + RTRd[2][2]*W_d[2]);
}
