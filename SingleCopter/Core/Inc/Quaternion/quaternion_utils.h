#ifndef QUATERNION_UTILS_H
#define QUATERNION_UTILS_H


typedef struct {
    float w;  // Scalar part
    float x;  // First component of the vector part
    float y;  // Second component of the vector part
    float z;  // Third component of the vector part
} Quaternion;


void get_angles(Quaternion* q, float angles[3]);


#endif
