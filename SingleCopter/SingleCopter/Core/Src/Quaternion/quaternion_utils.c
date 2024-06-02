#include <math.h>
#include "Quaternion/quaternion_utils.h"






void get_angles(Quaternion* q, float res[3])
{

	float q0 = q->w;
	float q1 = q->x;
	float q2 = q->y;
	float q3 = q->w;

	res[0] = atan2f(q0 * q1 +  q2 * q3, 0.5 - (q1 * q1 + q2 * q2) );
	res[1] = asinf(2  * (q0 * q2 - q3 * q1 )); 								// TODO
	res[2] = atan2f(q0 * q3 + q1* q2, 0.5 - 1 * (q2 * q2 + q3 * q3)   );

}
