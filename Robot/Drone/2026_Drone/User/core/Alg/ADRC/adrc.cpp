#include "adrc.hpp"

float ALG::ADRC::FirstLADRC::LADRC_1(float input, float feedback)
{
    LESO_1(feedback);
    LSEF_1(input);
    std::clamp(U, GetMin(), GetMax());
    return U;
}

void ALG::ADRC::FirstLADRC::LSEF_1(float target)
{
    KP = GetWc();

    U0 = KP * (target - Z1);
    U = (U0 - Z2) / GetB0();
}

void ALG::ADRC::FirstLADRC::LESO_1(float feedback)
{
    Beta1 = 2.0f * GetW0();
    Beta2 = GetW0() * GetW0();

    E = feedback - Z1;
    
    Z1 += GetH() * (Beta1 * E + Z2 + GetB0() * U);
    Z2 += GetH() * (Beta2 * E);
}





float ALG::ADRC::SecondLADRC::LADRC_2(float input, float feedback)
{
    TD_2(input);
    LESO_2(feedback);
    LSEF_2();
    std::clamp(U, GetMin(), GetMax());
    return U;
}

void ALG::ADRC::SecondLADRC::LSEF_2()
{
    KP = GetWc() * GetWc();
    KD = 2.0f * GetWc();

    U0 = KP * (V1 - Z1) + KD * (V2 - Z2);
    U = (U0 - Z3) / GetB0();
}

void ALG::ADRC::SecondLADRC::LESO_2(float feedback)
{
    Beta1 = 3.0f * GetW0();
    Beta2 = 3.0f * GetW0() * GetW0();
    Beta3 = GetW0() * GetW0() * GetW0();

    E = feedback - Z1;

    Z1 += GetH() * (Z2 + Beta1 * E);
    Z2 += GetH() * (Z3 + GetB0() * U + Beta2 * E);
    Z3 += GetH() * (Beta3 * E);
}

void ALG::ADRC::SecondLADRC::TD_2(float input)
{ 
    float fh= -R * R * (V1 - input) - 2 * R * V2;
    V1 += V2 * GetH();
    V2 += fh * GetH();
}

