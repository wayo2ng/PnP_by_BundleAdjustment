#ifndef KEYPOINT_H
#define KEYPOINT_H
#include <Eigen/Core>

class KeyPoint
{
private:
	int _u, _v;
	int _id;
	int _num;//�����ĵ����Ȳ����ĵ�ĸ���
	double _angle;//��ת��

public:
	KeyPoint() {}
	KeyPoint(int u,int v,int id,int num):_u(u),_v(v),_id(id),_num(num){}
	~KeyPoint() {}

	inline void setU(int u) { _u = u; }
	inline void setV(int v) { _v = v; }
	inline void setId(int id) { _id = id; }
	inline void setNum(int num) { _num = num; }
	inline void setAngle(int angle) { _angle = angle; }
	inline int getId() { return _id; };
	inline int getNum() { return _num; }
	inline double getAngle() { return _angle; }
	inline Eigen::Vector2d getPointCoordi() { return Eigen::Vector3d(_u, _v); }
};
#endif
