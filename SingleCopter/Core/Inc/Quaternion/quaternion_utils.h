#ifndef QUATERNION_UTILS_H
#define QUATERNION_UTILS_H


typedef struct {
    float w;  // Scalar part
    float x;  // First component of the vector part
    float y;  // Second component of the vector part
    float z;  // Third component of the vector part
} Quaternion;


void get_rotation_from_quaternion(Quaternion* q, float R[3][3]);

void get_rotation_from_euler_angles(float angles[3], float R[3][3]);

void  get_omega_body_frame(float angles_rates[3], float angles[3], float W[3]);

void get_rotation_error(float R_d[3][3],  float R[3][3], float result[3][3]);

void get_rotation_error(float R_d[3][3],  float R[3][3], float result[3][3]);

#endif
