#ifndef MAB_COMMONS_KINEMATICS_H_
#define MAB_COMMONS_KINEMATICS_H_

#include "commons_math.hpp"
#include <eigen3/Eigen/Dense>
#include <iostream>
namespace commons
{
    namespace kinematics
    {
        const float bodyLen = 0.367f;   //distance from front motor to rear motor
        const float bodyWidth = 0.1f;   //distance from left motor to right motor

        // Leg lengths
        const float l1 = 0.0525f;   // Offset from J1 to J2 along [J1]X axis
        const float l2 = 0.18f;     // Length of femurr
        const float l3 = 0.177f;    // Length of tibia
        const float l4 = 0.119f;    // Offset from J1 to foot along [J1]Y axis

        const float l5 = 0.021;     // Offset from J2 to foot along X 
        const float l6 = 0.175;     // Offset foot height


        const Eigen::Vector3f bodyToLegRotationEuler[4] = //Same as in URDF
        {
            Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            Eigen::Vector3f(M_PI, 0.0f, 0.0f),
            Eigen::Vector3f(M_PI, M_PI, 0.0f),
            Eigen::Vector3f(0.0f, M_PI, 0.0f),
        };
        const Eigen::Quaternionf bodyToLegRotoationQuat[4] = 
        {
            math::quatFromRpy(bodyToLegRotationEuler[0]),
            math::quatFromRpy(bodyToLegRotationEuler[1]),
            math::quatFromRpy(bodyToLegRotationEuler[2]),
            math::quatFromRpy(bodyToLegRotationEuler[3])
        };
        const Eigen::Matrix3f bodyToLegRotationRotMat[4] = 
        {
            commons::math::rotMatFromRpy(bodyToLegRotationEuler[0]),
            commons::math::rotMatFromRpy(bodyToLegRotationEuler[1]),
            commons::math::rotMatFromRpy(bodyToLegRotationEuler[2]),
            commons::math::rotMatFromRpy(bodyToLegRotationEuler[3])
        };
        const Eigen::Vector3f bodyToLegTranslation[4] = 
        {
            Eigen::Vector3f( bodyLen / 2.0f,    -bodyWidth / 2.0f,  0.0f),
            Eigen::Vector3f( bodyLen / 2.0f,     bodyWidth / 2.0f,  0.0f),
            Eigen::Vector3f(-bodyLen / 2.0f,     bodyWidth / 2.0f,  0.0f),
            Eigen::Vector3f(-bodyLen / 2.0f,    -bodyWidth / 2.0f,  0.0f),
        };

        inline Eigen::Vector3f calculateFkFromLegOrigin(const Eigen::Vector3f& leg_angles, int leg_no)
        {
            float s1 = sin(leg_angles(0));
            float s2 = sin(leg_angles(1));

            float c1 = cos(leg_angles(0));
            float c2 = cos(leg_angles(1));

            float j4 = leg_angles(1) + leg_angles(2);    // Angle between l3 and planar X axis
            float s4 = sin(j4);
            float c4 = cos(j4);

            Eigen::Vector3f fk;
            fk.x() = -l2 * c2 + l3*c4 + l1;
            fk.y() = (l2 * s2 - l3 * s4) * s1 - l4 * c1;
            fk.z() = -(l2 * s2 - l3 * s4) * c1 - l4 * s1;

            //Compensating for flipped Z axis
            if ( (leg_no == 1) || (leg_no == 3) )
            {
                fk.z() = -fk.z();
            }
            return fk;
        }
        inline Eigen::Vector3f calculateFkFromBody(const Eigen::Vector3f& leg_angles, int leg_no)
        {
            Eigen::Vector3f fk = calculateFkFromLegOrigin(leg_angles,leg_no);

            fk = commons::kinematics::bodyToLegRotationRotMat[leg_no] * fk;
            if(leg_no == 1 || leg_no == 3)
                fk.z() = -fk.z();
            fk.x() = fk.x() + commons::kinematics::bodyToLegTranslation[leg_no].x();
            fk.y() = fk.y() + commons::kinematics::bodyToLegTranslation[leg_no].y();
            fk.z() = fk.z() + commons::kinematics::bodyToLegTranslation[leg_no].z();
            return fk;
        }
        inline Eigen::Matrix3f calculateJacobianT(const Eigen::Vector3f& leg_angles, int leg_no)
        {
            float s1 = sinf(leg_angles(0));
            float s2 = sinf(leg_angles(1));

            float c1 = cosf(leg_angles(0));
            float c2 = cosf(leg_angles(1));

            float j4 = leg_angles(1) + leg_angles(2);    // Angle between l3 and planar X axis
            float s4 = sinf(j4);
            float c4 = cosf(j4);

            //Jacobian
            Eigen::Matrix3f J;
            if ( leg_no == 0 || leg_no == 2)
            {
            J <<    0.0f,                                   l2 * s2 - l3*s4,            -l3*s4,
                    l4*s1 + (l2 *s2 -l3 *s4) * c1,          (l2 * c2 - l3 * c4) * s1,   -l3 * s1 * c4,
                    -l4 * c1 - (-l2 * s2 + l3 * s4) * s1,   (-l2*c2+l3*c4)*c1,          l3*c1*c4;
            }
            // //Compensating for flipped Z axis
            if ( (leg_no == 1) || (leg_no == 3) )
            {
            J <<    0.0f,                                   l2 * s2 - l3*s4,            -l3*s4,
                    l4*s1 + (l2 *s2 -l3 *s4) * c1,          (l2 * c2 - l3 * c4) * s1,   -l3 * s1 * c4,
                    l4 * c1 - (l2 * s2 - l3 * s4) * s1,     (l2*c2-l3*c4)*c1,           -l3*c1*c4;
            }
            
            return J.transpose();
        }
        
        inline Eigen::Vector3f calculateIkFromLegOrigin(Eigen::Vector3f& footPosCart, int leg_no, bool kneeBackwards)
        {           
            if(footPosCart.squaredNorm() > 0.335*0.335)
                footPosCart = footPosCart.normalized() * 0.335f;
                
            footPosCart = bodyToLegRotationRotMat[leg_no] * footPosCart;    //rotate the vector according to leg
            float x = footPosCart.x(), y = footPosCart.y(), z = footPosCart.z();
            float legL1 = commons::kinematics::l2;
            float legL2 = commons::kinematics::l3;
            float theta1 = 0.0f, theta2 = 0.0f, theta3 = 0.0f;
            float yz_offset = commons::kinematics::l4;
            float x_offset = commons::kinematics::l1;
            if (x == 0.0f)  x = 0.0001f;
            if (y == 0.0f)  y = 0.0001f;          
            if (z == 0.0f)  z = 0.0001f;
                
            float l_zy = sqrtf32(y*y + z*z);
            x = x - x_offset;

            // Theta 1 - hip - computation
            float t1_a = asinf32(yz_offset / l_zy);
            float t1_b = asinf32(y / l_zy);
            theta1 = (t1_a + t1_b);

            // Theta 3 - knee - computation
            float l_t1t2 = sqrtf32(l_zy * l_zy - yz_offset * yz_offset);
            l_t1t2 = sqrtf32 (l_t1t2 * l_t1t2 + x*x);   // Z compensation from X displacement
            theta3 = acosf32( (l_t1t2 * l_t1t2 - legL1*legL1 - legL2*legL2) / (-2.0f * legL1 * legL2) );
            
            //Theta 2 - thigh
            float t2_a = asinf32( x/l_t1t2 );
            float t2_b = acosf32( (legL2*legL2 - legL1*legL1 - l_t1t2*l_t1t2) / ( - 2.0f * legL1 * l_t1t2) );

            if (kneeBackwards)
            {
                theta2 = M_PI_2 + t2_a - t2_b;
                theta3 = -theta3;
            }
            else
                theta2 = M_PI_2 + t2_a + t2_b;

            if(leg_no == 1 || leg_no == 3)
            {
                theta1 = -theta1;
                theta2 = -theta2;
                theta3 = -theta3;
            }
            float pix2 = M_PI * 2;
            if(theta1 > M_PI)   theta1 -= pix2;
            if(theta2 > M_PI)   theta2 -= pix2;
            if(theta3 > M_PI)   theta3 -= pix2;
                
            if(std::isnan(theta1))  theta1 = 0.0f;
            if(std::isnan(theta2))  theta2 = 0.0f;
            if(std::isnan(theta3))  theta3 = 0.0f;             
            return Eigen::Vector3f(theta1, theta2, theta3);
        }
        inline Eigen::Vector3f calculateIkFromBody(const Eigen::Vector3f& footPosCart, int legNum)
        {
            Eigen::Translation3f translation(-bodyToLegTranslation[legNum]);
            Eigen::Vector3f pCart = translation * footPosCart;
            switch (legNum)
            {
            case 0:     //FR leg
            {
                return calculateIkFromLegOrigin(pCart, legNum, true);
                break;
            }
            case 1:     //FL leg
            {
                return calculateIkFromLegOrigin(pCart, legNum, true);
                break;
            }
            case 2:     //RL leg
            {
                return calculateIkFromLegOrigin(pCart, legNum, true);
                break;
            }
            case 3:     //RR leg
            {
                return calculateIkFromLegOrigin(pCart, legNum, true);
                break;
            }
            default:
                return Eigen::Vector3f::Zero();
                break;
            }
        }
    }
}

#endif