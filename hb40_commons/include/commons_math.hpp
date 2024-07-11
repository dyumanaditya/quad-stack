#ifndef MAB_COMMONS_MATH_H_
#define MAB_COMMONS_MATH_H_

#include "eigen3/Eigen/Dense"

typedef uint32_t u32;
typedef Eigen::MatrixXf mX;
typedef Eigen::Matrix3f m3;
typedef Eigen::Matrix<float, 12, 12> m12;
typedef Eigen::Matrix<float, 18, 18> m18;
typedef Eigen::Matrix<float, 28, 28> m28;
typedef Eigen::VectorXf vX;
typedef Eigen::Vector<float, 3> v3;
typedef Eigen::Vector<float, 4> v4;
typedef Eigen::Vector<float, 6> v6;
typedef Eigen::Vector<float, 12> v12;
typedef Eigen::Vector<float, 28> v28;
typedef Eigen::Quaternionf quat;
typedef v3 v3world;
typedef v3 v3body;
typedef v3 v3rpy;

namespace commons
{
	namespace math
	{
		inline v3body worldToBody(const m3& rBody, const v3world& p)
		{
			return rBody.transpose() * p;
		}
		inline v3body worldToBody(const m3& rBody, const v3world& bodyPos, const v3world& p)
		{
			return rBody.transpose() * (p - bodyPos);
		}
		inline v3body bodyToWorld(const m3& rBody, const v3world& bodyPos, const v3world& p)
		{
			return bodyPos + rBody * p;
		}
		inline m3 rotMatFromQuat(const quat& quat) { return quat.toRotationMatrix(); }
		inline m3 rotMatFromRpy(const v3& rpy)
		{
			quat q = Eigen::AngleAxisf(rpy.x(), v3::UnitX()) * Eigen::AngleAxisf(rpy.y(), v3::UnitY()) *
					 Eigen::AngleAxisf(rpy.z(), v3::UnitZ());
			return q.toRotationMatrix();
		}
		inline m3 rotMatFromRpy(float r, float p, float y)
		{
			quat q = Eigen::AngleAxisf(r, v3::UnitX()) * Eigen::AngleAxisf(p, v3::UnitY()) *
					 Eigen::AngleAxisf(y, v3::UnitZ());
			return q.toRotationMatrix();
		}
		inline v3 rpyFromRotMat(const m3& mat)
		{
			v3 eulerXYZ;
			eulerXYZ(0) = atan2(mat(2, 1), mat(2, 2)); // Roll
			eulerXYZ(1) = -asin(mat(2, 0));			   // Pitch
			eulerXYZ(2) = atan2(mat(1, 0), mat(0, 0)); // Yaw
			return eulerXYZ;
		}
		inline v3 rpyFromQuat(quat q) { return rpyFromRotMat(q.toRotationMatrix()); }
		inline quat quatFromRotMat(const m3& mat)
		{
			quat q(mat);
			return q;
		}
		inline quat quatFromRpy(const v3& rpy)
		{
			quat q(rotMatFromRpy(rpy));
			return q;
		}
		inline m3 crossMatrix(const v3& vec)
		{
			return (m3() << 0.f, -vec(2), vec(1), vec(2), 0.f, -vec(0), -vec(1), vec(0), 0.f)
				.finished();
		}
		inline v3 matrixLogRot(const m3& R)
		{
			v3 omega;
			double theta;
			double tmp = (R(0, 0) + R(1, 1) + R(2, 2) - 1.) / 2.;
			if (tmp >= 1.)
				theta = 0.;
			else if (tmp <= -1.)
				theta = M_PI;
			else
				theta = acos(tmp);
			omega << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
			if (theta > 10e-5)
				omega *= theta / (2. * sin(theta));
			else
				omega /= 2.;
			return omega;
		}
		inline quat quatProduct(const quat& q1, const quat& q2)
		{
			float a1 = q1.w();
			float b1 = q1.x();
			float c1 = q1.y();
			float d1 = q1.z();

			float a2 = q2.w();
			float b2 = q2.x();
			float c2 = q2.y();
			float d2 = q2.z();

			quat q;
			// Hamilton product - wikipedia
			q.w() = a1 * a2 - b1 * b2 - c1 * c2 - d1 * d2;
			q.x() = a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2;
			q.y() = a1 * c2 - b1 * d2 + c1 * a2 + d1 * b2;
			q.z() = a1 * d2 + b1 * c2 - c1 * b2 + d1 * a2;

			return q;
		}
		/*!
		 * @brief Compute the pseudo inverse of a matrix
		 * @param matrix : input matrix
		 * @param sigmaThreshold : threshold for singular values being zero
		 * @return output matrix
		 */
		inline mX pseudoInverse(const mX& matrix, double sigmaThreshold)
		{
			mX invMatrix;
			if ((1 == matrix.rows()) && (1 == matrix.cols()))
			{
				invMatrix.resize(1, 1);
				if (matrix.coeff(0, 0) > sigmaThreshold)
					invMatrix.coeffRef(0, 0) = 1.f / matrix.coeff(0, 0);
				else
					invMatrix.coeffRef(0, 0) = 0.f;
				return invMatrix;
			}
			Eigen::JacobiSVD<mX> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
			int const nrows(svd.singularValues().rows());
			mX invS;
			invS = mX::Zero(nrows, nrows);
			for (int ii(0); ii < nrows; ++ii)
				if (svd.singularValues().coeff(ii) > sigmaThreshold)
					invS.coeffRef(ii, ii) = 1.f / svd.singularValues().coeff(ii);
			invMatrix = svd.matrixV() * invS * svd.matrixU().transpose();
			return invMatrix;
		}
	} // namespace math
} // namespace commons

#endif
