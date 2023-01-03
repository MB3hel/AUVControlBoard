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

// TODO

