#include <angles.h>
#include <math.h>


////////////////////////////////////////////////////////////////////////////////
/// Euler operations
////////////////////////////////////////////////////////////////////////////////

void euler_deg2rad(euler_t *dest, euler_t *src){
    if(src->is_deg){
        dest->pitch = src->pitch * M_PI / 180.0f;
        dest->roll = src->roll * M_PI / 180.0f;
        dest->yaw = src->yaw * M_PI / 180.0f;
    }else{
        dest->pitch = src->pitch;
        dest->roll = src->roll;
        dest->yaw = src->yaw;
    }
    dest->is_deg = false;
}

void euler_rad2deg(euler_t *dest, euler_t *src){
    if(!src->is_deg){
        dest->pitch = src->pitch / M_PI * 180.0f;
        dest->roll = src->roll / M_PI * 180.0f;
        dest->yaw = src->yaw / M_PI * 180.0f;
    }else{
        dest->pitch = src->pitch;
        dest->roll = src->roll;
        dest->yaw = src->yaw;
    }
    dest->is_deg = true;
}

void euler_to_quaternion(quaternion_t *dest, euler_t *src){
    euler_t src_rad;
    euler_deg2rad(&src_rad, src);
    float cr = cosf(src_rad.roll * 0.5f);
    float sr = sinf(src_rad.roll * 0.5f);
    float cp = cosf(src_rad.pitch * 0.5f);
    float sp = sinf(src_rad.pitch * 0.5f);
    float cy = cosf(src_rad.yaw * 0.5f);
    float sy = sinf(src_rad.yaw * 0.5f);
    dest->w = cp * cr * cy + sp * sr * sy;
    dest->x = sp * cr * cy - cp * sr * sy;
    dest->y = cp * sr * cy + sp * cr * sy;
    dest->z = cp * cr * sy - sp * sr * cy;
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

