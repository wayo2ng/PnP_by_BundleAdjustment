#include<iostream>
#include<vector>
#include<Eigen/Core>
#include<Sophus/se3.hpp>
#include<opencv2/opencv.hpp>

class BundleAdjustment
{
public:
	Eigen::Matrix<double, 2, 6>findPoseJacobian(Eigen::Matrix<double, 4, 4>&Pose, Eigen::Vector3d& P)
	{
		Eigen::Matrix<double, 2, 3>J = Eigen::Matrix<double, 2, 6>::Zero(2, 6);
		Eigen::Matrix<double, 4, 1>Pc;
		Pc << P(0), P(1), P(2), P(3);
		Pc = Pose * Pc;//相机坐标系下的点
		double fx = K(0, 0);
		double fy = K(1, 1);
		double X, Y, Z;
		X = Pc(0);
		Y = Pc(1);
		Z = Pc(2);
		J(0, 0) = fx / Z;
		J(0, 2) = -fx * X / (Z*Z);
		J(0, 3) = -(fx * X*Y) / (Z*Z);
		J(0, 4) = fx + fx * X*X / Z;
		J(0, 5) = -fx * Y / Z;
		J(1, 1) = fy / Z;
		J(1, 2) = -1 * ((fy*Y) / (Z*Z));
		J(1, 3) = -fy - ((fy*Y*Y) / (Z*Z));
		J(1, 4) = (fy*X*Y) / (Z*Z);
		J(1, 5) = (fy*X) / Z;
		return J;
	}

	Eigen::Matrix<double, 2, 3>findPointJacobian(Eigen::Matrix<double, 4, 4>&Pose, Eigen::Vector3d &P)
	{
		Eigen::Matrix<double, 2, 6>J = Eigen::Matrix<double, 2, 6>::Zero(2, 6);
		Eigen::Matrix<double, 4, 1>Pc;
		Pc << P(0), P(1), P(2), P(3);
		Pc = Pose * Pc;//相机坐标系下的点
		double fx = K(0, 0);
		double fy = K(1, 1);
		double X, Y, Z;
		X = Pc(0);
		Y = Pc(1);
		Z = Pc(2);
		J(0, 0) = fx / Z;
		J(0, 2) = -fx * X / (Z*Z);
		J(1, 1) = fy / Z;
		J(1, 2) = -1 * ((fy*Y) / (Z*Z));

		return J * Pose.block(0, 0, 3, 3);
	}

	Eigen::MatrixXd findWholeJacobian(const Eigen::Matrix3Xd &x, Eigen::Matrix<double, 3, 3>&K)
	{
		Eigen::Matrix3Xd J;
		int size_p = (x.rows() - 6) / 3;//坐标点的数量
		J.resize(2 * size_p, 3 * size_p + 6);
		J.setZero();
		Eigen::VectorXd vec_pose(6); vec_pose = x.block(0, 0, 6, 1);
		Eigen::Matrix<double, 4, 4>Pose = Sophus::SE3<double>::exp(vec_pose).matrix();
		for (int i = 0; i < size_p; i++)
		{
			Eigen::Vector3d p = x.block(6 + 3 * i, 0, 3, 1);
			J.block(2 * i, 0, 2, 6) = findPoseJacobian(Pose, p, K);
			J.block(2 * i, 6 + 3 * i, 2, 3) = findPointJacobian(Pose, p, K);
		}
		return J;
	}

	Eigen::MatrixXd findCostFunction(const Eigen::Matrix3Xd &x, std::vector<cv::Point2d>&points_2d, Eigen::Matrix<double, 3, 3>&K)
	{
		Eigen::MatrixXd e;//重投影误差
		int size_P = (x.rows() - 6) / 3;
		if (size_P != points_2d.size())
		{
			std::cout << "3D点与2D点数目不匹配！" << std::endl;
			return e;
		}
		e.resize(size_P, 1);
		e.setZero();
		double fx = K(0, 0);
		double fy = K(1, 1);
		double cx = K(0, 2);
		double cy = K(1, 2);
		Eigen::VectorXd vec_pose(6); vec_pose = x.block(0, 0, 6, 1);
		Eigen::Matrix<double, 4, 4>Pose = Sophus::SE3<double>::exp(vec_pose).matrix();
		for (int i = 0; i < size_P; i++)
		{
			Eigen::Matrix<double, 4, 1>point_3d;
			point_3d << x(6 + 2 * i, 3 * i), x(6 + 2 * i, 3 * i + 1), x(6 + 2 * i, 3 * i + 2), 1;
			Eigen::Matrix<double, 4, 1> Pc = Pose * point_3d;
			e(2 * i) = points_2d[i].x - (fx * Pc(0) / Pc(2) + cx);
			e(2 * i + 1) = points_2d[i].y - (fy * Pc(1) / Pc(2) + cy);
		}
		return e;
	}

	void Optimie(Eigen::Matrix<double, 4, 4> &T, cv::Mat &img)
	{
		Eigen::MatrixXd x;//待优化的状态量
		int size_p = points_3d.size();
		x.resize(6 + 3 * size_p, 1);
		x.setZero();
		Sophus::SE3<double>Pose(T);
		x.block(0, 0, 6, 1) = Pose.log();//位姿的李代数
		for (int i = 0; i < size_p; i++)
		{
			x(6 + 3 * i) = points_3d[i].x;
			x(6 + 3 * i + 1) = points_3d[i].y;
			x(6 + 3 * i + 2) = points_3d[i].z;
		}
		for (int i = 0; i < max_interator; i++)
		{

		}
	}
private:
	int max_interator;
	Eigen::Matrix<double, 3, 3> K;//相机内参
	std::vector<cv::Point2d>points_2d;//观测点
	std::vector<cv::Point2d>points_3d;//三维点

};

