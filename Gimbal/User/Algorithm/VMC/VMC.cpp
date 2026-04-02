#include "../../Algorithm/VMC/VMC.hpp"

void ALG::VMC::VMC::FK()
{
    x = L1*cosf(theta1) + L2*cosf(theta1+theta2);
    z = L1*sinf(theta1) + L2*sinf(theta1+theta2);
}

void ALG::VMC::VMC::Jacobian()
{
    J00 = -L1*sinf(theta1) - L2*sinf(theta1+theta2);
    J01 = -L2*sinf(theta1+theta2);
    J10 = L1*cosf(theta1) + L2*cosf(theta1+theta2);
    J11 = L2*cosf(theta1+theta2);
}

// void ALG::VMC::VMC::FictitiousForce(float target_pitch)
// {
//     float target_pitch_rad = target_pitch * 3.14159265f / 180.0f;

//     x_dot = J00 * theta1_dot + J01 * theta2_dot;
//     z_dot = J10 * theta1_dot + J11 * theta2_dot;

//     target_theta = 3.14159265f - (gama - target_pitch_rad); 
//     target_x = L1*cosf(target_theta) + L2*cosf(target_pitch_rad);
//     target_z = L1*sinf(target_theta) + L2*sinf(target_pitch_rad);

//     Fx = Kx * (target_x - x) + Bx * (0 - x_dot);
//     Fz = Kz * (target_z - z) + Bz * (0 - z_dot);
// }

void ALG::VMC::VMC::FictitiousForce(float target_pitch2, float target_pitch1)
{
    float target_pitch_rad = target_pitch2 * 3.14159265f / 180.0f;
    float theta_fixed = (95.0f + target_pitch1) * 3.14159265f / 180.0f;

    x_dot = J00 * theta1_dot + J01 * theta2_dot;
    z_dot = J10 * theta1_dot + J11 * theta2_dot;

    target_x = L1 * cosf(theta_fixed) + L2 * cosf(target_pitch_rad);
    target_z = L1 * sinf(theta_fixed) + L2*sinf(target_pitch_rad);
    
    // target_x = L2 * cosf(target_pitch_rad);
    // target_z = L1 + L2*sinf(target_pitch_rad);

    Fx = Kx * (target_x - x) + Bx * (0 - x_dot);
    Fz = Kz * (target_z - z) + Bz * (0 - z_dot);
}

void ALG::VMC::VMC::VMC_Update(float target_pitch2, float target_pitch1)
{
    FK();
    Jacobian();
    FictitiousForce(target_pitch2, target_pitch1);

    T1 = J00 * Fx + J10 * Fz;
    T2 = J01 * Fx + J11 * Fz;
}
