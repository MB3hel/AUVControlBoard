#include <angles.h>
#include <math.h>


////////////////////////////////////////////////////////////////////////////////
/// Euler operations
////////////////////////////////////////////////////////////////////////////////

void euler_deg2rad(euler_t *dest, euler_t *src){
    if(src->is_deg){
        dest->pitch = src->pitch * ((float)M_PI) / 180.0f;
        dest->roll = src->roll * ((float)M_PI) / 180.0f;
        dest->yaw = src->yaw * ((float)M_PI) / 180.0f;
    }else{
        dest->pitch = src->pitch;
        dest->roll = src->roll;
        dest->yaw = src->yaw;
    }
    dest->is_deg = false;
}

void euler_rad2deg(euler_t *dest, euler_t *src){
    if(!src->is_deg){
        dest->pitch = src->pitch / ((float)M_PI) * 180.0f;
        dest->roll = src->roll / ((float)M_PI) * 180.0f;
        dest->yaw = src->yaw / ((float)M_PI) * 180.0f;
    }else{
        dest->pitch = src->pitch;
        dest->roll = src->roll;
        dest->yaw = src->yaw;
    }
    dest->is_deg = true;
}

void euler_to_quat(quaternion_t *dest, euler_t *src){
    euler_t src_rad;
    euler_deg2rad(&src_rad, src);
    float cr = cosf(src->roll / 2.0f);
    float sr = sinf(src->roll / 2.0f);
    float cp = cosf(src->pitch / 2.0f);
    float sp = sinf(src->pitch / 2.0f);
    float cy = cosf(src->yaw / 2.0f);
    float sy = sinf(src->yaw / 2.0f);
    dest->w = cy * cp * cr - sy * sp * sr;
    dest->x = cy * cr * sp - sy * cp * sr;
    dest->y = cy * cp * sr + sy * cr * sp;
    dest->z = cy * sp * sr + sy * cp * cr;
}



////////////////////////////////////////////////////////////////////////////////
/// Quaternion operations
////////////////////////////////////////////////////////////////////////////////

void quat_multiply_scalar(quaternion_t *dest, quaternion_t *a, float b){
    dest->w = a->w * b;
    dest->x = a->x * b;
    dest->y = a->y * b;
    dest->z = a->z * b;
}

void quat_divide_scalar(quaternion_t *dest, quaternion_t *a, float b){
    dest->w = a->w / b;
    dest->x = a->x / b;
    dest->y = a->y / b;
    dest->z = a->z / b;
}

void quat_multiply(quaternion_t *dest, quaternion_t *a, quaternion_t *b){
    // Note: using temp w, x, y, z vars because dest may be the same as a or b
    float w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
    float x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
    float y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x;
    float z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w;
    dest->w = w;
    dest->x = x;
    dest->y = y;
    dest->z = z;
}

void quat_inverse(quaternion_t *dest, quaternion_t *src){
    float mag;
    quat_magnitude(&mag, src);
    quat_conjugate(dest, src);
    quat_divide_scalar(dest, dest, mag);
}

void quat_conjugate(quaternion_t *dest, quaternion_t *src){
    dest->w = src->w;
    dest->x = -src->x;
    dest->y = -src->y;
    dest->z = -src->z;
}

void quat_magnitude(float *dest, quaternion_t *src){
    *dest = sqrtf(src->w*src->w + src->x*src->x + src->y*src->y + src->z*src->z);
}

void quat_dot(float *dest, quaternion_t *a, quaternion_t *b){
    *dest = a->w*b->w + a->x*b->x + a->y*b->y + a->z*b->z;
}

void quat_to_euler(euler_t *dest, quaternion_t *src){
    dest->is_deg = false;
    dest->pitch = asinf(2.0f * (src->y*src->z + src->w*src->x));

    float pitchdeg = 180.0f * dest->pitch / ((float)M_PI);
    if(fabsf(90.0f - fabsf(pitchdeg)) < 0.1f){
        // Pitch is +/- 90 degrees
        // This is gimbal lock scenario
        // Roll and yaw mean the same thing
        // roll + yaw = 2 * atan2(q.y, q.w)
        // Can split any way (not unique)
        dest->yaw = 2.0f * atan2f(src->y, src->w);
        dest->roll = 0.0f;
    }else{
        float roll_numer = 2.0f * (src->w*src->y - src->x*src->z);
        float roll_denom = 1.0f - 2.0f * (src->x*src->x + src->y*src->y);
        dest->roll = atan2f(roll_numer, roll_denom);

        float yaw_numer = 2.0f * (src->x*src->y - src->w*src->z);
        float yaw_denom = 1.0f - 2.0f * (src->x*src->x + src->z*src->z);
        dest->yaw = atan2f(yaw_numer, yaw_denom);
    }
}
