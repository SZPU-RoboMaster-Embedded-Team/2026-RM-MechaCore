#ifndef VMC_HPP
#define VMC_HPP

#include "math.h"

namespace ALG::VMC
{
    class VMC
    {
        public:
            VMC(float l1, float l2, float kx, float kz, float bx, float bz)
            {
                x = 0;
                z = 0;
                L1 = l1;
                L2 = l2;
                Kx = kx;
                Kz = kz;
                Bx = bx;
                Bz = bz;
                L = sqrtf(L1 * L1 + L2 * L2);
                gama = acosf((L1*L1 + L2*L2 - L*L) / (2.0f*L1*L2));
            }

            void FK();
            void Jacobian();
            void FictitiousForce(float target_pitch2, float target_pitch1);
            void VMC_Update(float target_pitch2, float target_pitch1);

            void Settheta(float theta1_, float theta2_)
            {
                theta1 = theta1_ * 3.14159265f / 180.0f;
                theta2 = theta2_ * 3.14159265f / 180.0f;
            }
            void Settheta_dot(float velocity1, float velocity2)
            {
                theta1_dot = velocity1;
                theta2_dot = velocity2; 
            }

            float GetT1() { return T1; }
            float GetT2() { return T2; }
            
        private:
            float x, x_dot;
            float z, z_dot;
            float L1, L2;
            float theta1, theta2;
            float J00, J01, J10, J11;
            float Kx, Kz;
            float Bx, Bz;
            float target_x, target_z;
            float theta1_dot, theta2_dot;
            float Fx, Fz;
            float T1, T2;
            float L, gama, target_theta;
    };
}

#endif // !VMC_HPP
