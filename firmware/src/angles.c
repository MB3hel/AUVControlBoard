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

void euler_to_quat(quaternion_t *dest, euler_t *src){
    euler_t src_rad;
    euler_deg2rad(&src_rad, src);
    dest->w = cosf(src_rad.pitch/2.0f) * cosf(src_rad.roll/2.0f) * cosf(src_rad.yaw/2.0f) + sinf(src_rad.pitch/2.0f) * sinf(src_rad.roll/2.0f) * sinf(src_rad.yaw/2.0f);
    dest->x = sinf(src_rad.pitch/2.0f) * cosf(src_rad.roll/2.0f) * cosf(src_rad.yaw/2.0f) - cosf(src_rad.pitch/2.0f) * sinf(src_rad.roll/2.0f) * sinf(src_rad.yaw/2.0f);
    dest->y = cosf(src_rad.pitch/2.0f) * sinf(src_rad.roll/2.0f) * cosf(src_rad.yaw/2.0f) + sinf(src_rad.pitch/2.0f) * cosf(src_rad.roll/2.0f) * sinf(src_rad.yaw/2.0f);
    dest->z = cosf(src_rad.pitch/2.0f) * cosf(src_rad.roll/2.0f) * sinf(src_rad.yaw/2.0f) - sinf(src_rad.pitch/2.0f) * sinf(src_rad.roll/2.0f) * cosf(src_rad.yaw/2.0f);
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
    float t0 = 2.0f * (src->w * src->x + src->y * src->z);
    float t1 = 1.0f - 2.0f * (src->x * src->x + src->y * src->y);
    dest->pitch = atan2f(t0, t1);
    float t2 = 2.0f * (src->w * src->y - src->z * src->x);
    t2 = (t2 > 1.0f) ? 1.0f : t2;
    t2 = (t2 < -1.0f) ? -1.0f : t2;
    dest->roll = asinf(t2);
    float t3 = 2.0f * (src->w * src->z + src->x * src->y);
    float t4 = 1.0f - 2.0f * (src->y * src->y + src->z * src->z);
    dest->yaw = atan2f(t3, t4);
}

void quat_flip_x(quaternion_t *src, quaternion_t *dest){
    // Flip the x axis
    src->w = dest->w;
    src->x = -dest->x;
    src->y = dest->y;
    src->z = dest->z;

    // Now have reverse handedness since odd number axis inversions
    // Thus flip all axes
    src->w = src->w;
    src->x = -src->x;
    src->y = -src->y;
    src->z = -src->z;
}

void quat_flip_y(quaternion_t *src, quaternion_t *dest){
    // Flip the y axis
    src->w = dest->w;
    src->x = dest->x;
    src->y = -dest->y;
    src->z = dest->z;

    // Now have reverse handedness since odd number axis inversions
    // Thus flip all axes
    src->w = src->w;
    src->x = -src->x;
    src->y = -src->y;
    src->z = -src->z;
}

void quat_flip_z(quaternion_t *src, quaternion_t *dest){
    // Flip the z axis
    src->w = dest->w;
    src->x = dest->x;
    src->y = dest->y;
    src->z = -dest->z;

    // Now have reverse handedness since odd number axis inversions
    // Thus flip all axes
    src->w = src->w;
    src->x = -src->x;
    src->y = -src->y;
    src->z = -src->z;
}
