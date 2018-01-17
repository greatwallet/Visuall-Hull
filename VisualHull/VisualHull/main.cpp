#pragma warning(disable:4819)
#pragma warning(disable:4244)
#pragma warning(disable:4267)
#define _SILENCE_STDEXT_HASH_DEPRECATION_WARNINGS

#include <time.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <limits>
#include <list>
//#include <hash_set>
#include <queue>
#include <vector>
#include <fstream>



// 用于判断投影是否在visual hull内部
//相机模块、及其拍摄照片
struct Projection
{
	//相机内外参矩阵的乘积
	Eigen::Matrix<float, 3, 4> m_projMat;
	cv::Mat m_image;
	//阈值（可设置为1~254之间任意数）
	const uint m_threshold = 125;

	bool outOfRange(int x, int max)
	{
		return x < 0 || x >= max;
	}
	//形参为空间坐标

	/*	给定三维空间点P(X, Y, Z)，可通过摄像机参数M = KR，获得该点落在图像上的位置p(x, y)
	*	返回值为布尔量，若为白色（轮廓内）则为true,否则为false
	*/
	bool checkRange(float x, float y, float z)
	{
		Eigen::Vector3f vec3 = m_projMat * Eigen::Vector4f(x, y, z, 1);
		//得到[su,sv,s]-->得到像素坐标的s倍
		//得到u\v的值，为相机投影像素坐标
		int indX = vec3[1] / vec3[2];	//此处存疑?
		int indY = vec3[0] / vec3[2];	

		if (outOfRange(indX, m_image.size().height) || outOfRange(indY, m_image.size().width))
			return false;
		//读入灰度图，判断像素点是黑是白
		//判断所得到的照片坐标的灰度值,若大于阈值则为黑色,否则为白色;(阈值为1~254的任意值)
		return m_image.at<uchar>((uint)(vec3[1] / vec3[2]), (uint)(vec3[0] / vec3[2])) > m_threshold;
		
	}
};

// 使得某一维度的坐标移动
struct CoordinateInfo
{
	int m_resolution;	//分辨率，指将量程划分的范围个数
	float m_min;		//最小值
	float m_max;		//最大值

	//坐标变换
	float index2coor(int index)
	{
		return float(m_min + index * (m_max - m_min) / m_resolution);
	}


	CoordinateInfo(int resolution = 10, float min = 0.0, float max = 100.0)
		: m_resolution(resolution)
		, m_min(min)
		, m_max(max)
	{
	}
};

//点结构
struct Point
{
	int _x; int _y; int _z;
	inline Point(int x = 0, int y = 0, int z = 0)
		:_x(x), _y(y), _z(z) {}
	inline bool operator == (const Point &right)const
	{
		return (_x == right._x) && (_y == right._y) && (_z == right._z);
	}
};

//从正方体向外延伸的八个向量
const static Point Vector8[8] =
{ Point(1, 1, 1),Point(1, 1, -1),Point(1, -1, 1),Point(1, -1, -1)
,Point(-1, 1, 1), Point(-1, 1, -1), Point(-1, -1, 1), Point(-1, -1, -1) };

//用于广度遍历的18个向量
const static Point Vector18[18] =
{
	Point(1,0,0),Point(-1,0,0),	//前后
	Point(0,1,0),Point(0,-1,0),	//左右
	Point(0,0,1),Point(0,0,-1),	//上下
	//dz=1上的四个方向
	Point(1,0,1),Point(0,1,1),Point(-1,0,1),Point(0,-1,1),
	//dz=0上的四个方向
	Point(1,1,0),Point(-1,1,0),Point(-1,-1,0),Point(1,-1,0),
	//dz=-1上的四个方向
	Point(1,0,-1),Point(0,1,-1),Point(-1,0,-1),Point(0,-1,-1),

};


class Model
{
public:
	//一个二维布尔型矩阵
	typedef std::vector<std::vector<bool>> Pixel;
	//三维布尔型矩阵
	typedef std::vector<Pixel> Voxel;

	//构造函数，3个形参分别为X,Y,Z坐标的精度（单位个数）
	Model(int resX = 100, int resY = 100, int resZ = 100);
	~Model();

	//找到第一个Hull上的点
	Point getFirstH();

	void saveModel(const char* pFileName);
	void saveModelWithNormal(const char* pFileName);
	void loadMatrix(const char* pFileName);
	void loadImage(const char* pDir, const char* pPrefix, const char* pSuffix);
	void getModel();
	void getSurface();
	Eigen::Vector3f getNormal(int indX, int indY, int indZ);

	
private:
	CoordinateInfo m_corrX;
	CoordinateInfo m_corrY;
	CoordinateInfo m_corrZ;

	bool out_of_range(int x, int y, int z);
	bool innerH(int x, int y, int z);
	bool innerH(const Point &p);

	int m_neiborSize;

	//相机向量
	std::vector<Projection> m_projectionList;

	std::vector<Eigen::Vector3f> neiborList;	//临近区域
	std::vector<Eigen::Vector3f> innerList;		//内部区域

	Voxel m_voxel;		//是否为内部点
	Voxel m_surface;	//表面情况

	Voxel m_discovered;	//是否已经遍历过
};

Model::Model(int resX, int resY, int resZ)
	: m_corrX(resX, -5, 5)
	, m_corrY(resY, -10, 10)
	, m_corrZ(resZ, 15, 30)
{
	//邻域面积为精度/100
	if (resX > 100)
		m_neiborSize = resX / 100;
	else
		m_neiborSize = 1;


	//voxel是x的精度*y的精度*z的精度的矩阵，初始化为true
	m_voxel = Voxel(m_corrX.m_resolution, Pixel(m_corrY.m_resolution, std::vector<bool>(m_corrZ.m_resolution, false)));
	m_surface = m_voxel;
	m_discovered = m_voxel;
}

Model::~Model()
{
}

//寻找第一个在表面或内部上的点
Point Model::getFirstH()
{
	int prejectionCount = m_projectionList.size();
	
	//在8个直线上直线上先搜索
	for (int i = 0; i < 8; i++)
	{
		//在原点处开始遍历
		int x = m_corrX.m_resolution >> 1;
		int y = m_corrY.m_resolution >> 1;
		int z = m_corrZ.m_resolution >> 1;
		while (!out_of_range(x, y, z))
		{
			//index2coor转换为空间中的位置
			float coorX = m_corrX.index2coor(x);
			float coorY = m_corrY.index2coor(y);
			float coorZ = m_corrZ.index2coor(z);
			//若找到内部点，则立刻返回
			if (innerH(x,y,z))return Point(x, y, z);
			x += Vector8[i]._x; y += Vector8[i]._y; z += Vector8[i]._z;
		}
	}

	
	exit(-1);
}

void Model::saveModel(const char* pFileName)
{
	//输出信息
	std::ofstream fout(pFileName);

	//三重循环，输出轮廓像黑色部分的坐标信息
	for (int indexX = 0; indexX < m_corrX.m_resolution; indexX++)
		for (int indexY = 0; indexY < m_corrY.m_resolution; indexY++)
			for (int indexZ = 0; indexZ < m_corrZ.m_resolution; indexZ++)
				if (m_surface[indexX][indexY][indexZ])
				{
					float coorX = m_corrX.index2coor(indexX);
					float coorY = m_corrY.index2coor(indexY);
					float coorZ = m_corrZ.index2coor(indexZ);
					fout << coorX << ' ' << coorY << ' ' << coorZ << std::endl;
				}
}

void Model::saveModelWithNormal(const char* pFileName)
{
	std::ofstream fout(pFileName);

	float midX = m_corrX.index2coor(m_corrX.m_resolution / 2);
	float midY = m_corrY.index2coor(m_corrY.m_resolution / 2);
	float midZ = m_corrZ.index2coor(m_corrZ.m_resolution / 2);

	for (int indexX = 0; indexX < m_corrX.m_resolution; indexX++)
		for (int indexY = 0; indexY < m_corrY.m_resolution; indexY++)
			for (int indexZ = 0; indexZ < m_corrZ.m_resolution; indexZ++)
				if (m_surface[indexX][indexY][indexZ])
				{
					float coorX = m_corrX.index2coor(indexX);
					float coorY = m_corrY.index2coor(indexY);
					float coorZ = m_corrZ.index2coor(indexZ);
					fout << coorX << ' ' << coorY << ' ' << coorZ << ' ';

					//输出移动单位坐标个数
					Eigen::Vector3f nor = getNormal(indexX, indexY, indexZ);
					fout << nor(0) << ' ' << nor(1) << ' ' << nor(2) << std::endl;
				}
}

void Model::loadMatrix(const char* pFileName)
{
	//输入信息
	std::ifstream fin(pFileName);

	int num;
	//相机内参
	Eigen::Matrix<float, 3, 3> matInt;
	//相机外餐
	Eigen::Matrix<float, 3, 4> matExt;
	//新相机
	Projection projection;
	while (fin >> num)
	{
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				fin >> matInt(i, j);

		float temp;
		fin >> temp;
		fin >> temp;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 4; j++)
				fin >> matExt(i, j);
		//相机参数
		projection.m_projMat = matInt * matExt;
		//新相机加入向量
		m_projectionList.push_back(projection);
	}
}

//prefix-前缀
//suffix-后缀

void Model::loadImage(const char* pDir, const char* pPrefix, const char* pSuffix)
{
	//相机数量
	int fileCount = m_projectionList.size();
	std::string fileName(pDir);
	fileName += '/';
	fileName += pPrefix;
	for (int i = 0; i < fileCount; i++)
	{
		std::cout << fileName + std::to_string(i) + pSuffix << std::endl;
		//读入图片
		m_projectionList[i].m_image = cv::imread(fileName + std::to_string(i) + pSuffix, CV_8UC1);
	}
}

//判断内部点、外部点
void Model::getModel()
{
	
	//相机数量
	int prejectionCount = m_projectionList.size();
	/*	用散列表M存储所有的体像素信息
	*	用队列Q存储待确认的外壳点
	*/	
//	std::hash_set<Point> M;
	std::queue<Point> Q;
	//先找到第一个在Hull上的点,已知此点为内部点（或表面点）
	Point orig = getFirstH();
	
	m_discovered[orig._x][orig._y][orig._z] = true;
//	M.insert(orig);
	Q.push(orig);

	while (!Q.empty())
	{
		Point v = Q.front(); Q.pop();
		m_discovered[v._x][v._y][v._z] = true;	//此点已被发现
		
		//表示该内部点周围有无外部点，若有，则为true，否则为false，初始值为false；
		bool near_outerH = false;
		for (int i = 0; i < 18; i++)
		{
			//确定邻域点
			Point nbr = Point(v._x + Vector18[i]._x, v._y + Vector18[i]._y, v._z + Vector18[i]._z);

			//若该邻域点未被遍历过
			if (!m_discovered[nbr._x][nbr._y][nbr._z])
			{
				m_discovered[nbr._x][nbr._y][nbr._z] = true;
				if (innerH(nbr))Q.push(nbr);
				else near_outerH = true;
			}
		}
		//转换为空间左边
		float coorX = m_corrX.index2coor(v._x);
		float coorY = m_corrY.index2coor(v._y);
		float coorZ = m_corrZ.index2coor(v._z);
		if (near_outerH)
		{
			m_surface[v._x][v._y][v._z] = true;
			//neiborList.push_back(Eigen::Vector3f(coorX, coorY, coorZ));
		}
		else 
		{
			m_voxel[v._x][v._y][v._z] = true;
			//innerList.push_back(Eigen::Vector3f(coorX, coorY, coorZ));
		}
	}
	
	//std::ofstream out("matrix_information1.txt");
	//out << "m_voxel: " << std::endl;
	//for (int x = 0; x < m_corrX.m_resolution; x++)
	//{
	//	out << "x = " << x << std::endl;
	//	for (int y = 0; y < m_corrY.m_resolution; y++)
	//	{
	//		for (int z = 0; z < m_corrZ.m_resolution; z++)
	//		{
	//			out << m_voxel[x][y][z] ;
	//		}
	//		out << std::endl;
	//	}
	//}
	//out << "m_surface: " << std::endl;
	//for (int x = 0; x < m_corrX.m_resolution; x++)
	//{
	//	out << "x = " << x << std::endl;
	//	for (int y = 0; y < m_corrY.m_resolution; y++)
	//	{
	//		for (int z = 0; z < m_corrZ.m_resolution; z++)
	//		{
	//			out << m_surface[x][y][z] ;
	//		}
	//		out << std::endl;
	//	}
	//}
	//out.close();
	
	//for (int indexX = 0; indexX < m_corrX.m_resolution; indexX++)
	//	for (int indexY = 0; indexY < m_corrY.m_resolution; indexY++)
	//		for (int indexZ = 0; indexZ < m_corrZ.m_resolution; indexZ++)
	//			for (int i = 0; i < prejectionCount; i++)
	//			{
	//				//index2coor转换为空间中的位置
	//				float coorX = m_corrX.index2coor(indexX);
	//				float coorY = m_corrY.index2coor(indexY);
	//				float coorZ = m_corrZ.index2coor(indexZ);
	//				/*	
	//				*	此处逻辑：当且仅当所有相机拍到当前空间点都为照片轮廓内部点，该点才是内部三维点
	//				*	只要有一个相机拍到其在外部（落在白色区域），则其为外部三维点
	//				*/	
	//				m_voxel[indexX][indexY][indexZ] = m_voxel[indexX][indexY][indexZ] 
	//					&& m_projectionList[i].checkRange(coorX, coorY, coorZ);
	//			}
}

//判断表面点
void Model::getSurface()
{
	// 邻域：	后、右、左、上、下、前。
	int dx[6] = { -1, 0, 0, 0, 0, 1 };
	int dy[6] = { 0, 1, -1, 0, 0, 0 };
	int dz[6] = { 0, 0, 0, 1, -1, 0 };

	// lambda表达式，用于判断某个点是否在Voxel的范围内（检查其是否越界）
	auto outOfRange = [&](int indexX, int indexY, int indexZ){
		return indexX < 0 || indexY < 0 || indexZ < 0
			|| indexX >= m_corrX.m_resolution
			|| indexY >= m_corrY.m_resolution
			|| indexZ >= m_corrZ.m_resolution;
	};

	for (int indexX = 0; indexX < m_corrX.m_resolution; indexX++)
		for (int indexY = 0; indexY < m_corrY.m_resolution; indexY++)
			for (int indexZ = 0; indexZ < m_corrZ.m_resolution; indexZ++)
			{
				//若该店已经是外部三维点，则其不可能是表面点
				if (!m_voxel[indexX][indexY][indexZ])
				{
					m_surface[indexX][indexY][indexZ] = false;
					continue;
				}
				//此处说明该点位内部点
				bool ans = false;
				for (int i = 0; i < 6; i++)
				{
					//若该点附近点有外部点，或者附近点越界，则说明该点是表面点
					ans = ans || outOfRange(indexX + dx[i], indexY + dy[i], indexZ + dz[i])
						|| !m_voxel[indexX + dx[i]][indexY + dy[i]][indexZ + dz[i]];
				}
				m_surface[indexX][indexY][indexZ] = ans;
			}
}

//确定表面法线方向
Eigen::Vector3f Model::getNormal(int indX, int indY, int indZ)
{
	//是否超出范围
	auto outOfRange = [&](int indexX, int indexY, int indexZ){
		return indexX < 0 || indexY < 0 || indexZ < 0
			|| indexX >= m_corrX.m_resolution
			|| indexY >= m_corrY.m_resolution
			|| indexZ >= m_corrZ.m_resolution;
	};

	std::vector<Eigen::Vector3f> neiborList;	//临近区域
	std::vector<Eigen::Vector3f> innerList;		//内部区域

	//在邻域中寻找
	for (int dX = -m_neiborSize; dX <= m_neiborSize; dX++)
		for (int dY = -m_neiborSize; dY <= m_neiborSize; dY++)
			for (int dZ = -m_neiborSize; dZ <= m_neiborSize; dZ++)
			{
				//若就在原点，则跳过
				if (!dX && !dY && !dZ)
					continue;
				//临近点坐标
				int neiborX = indX + dX;
				int neiborY = indY + dY;
				int neiborZ = indZ + dZ;

				if (!outOfRange(neiborX, neiborY, neiborZ))
				{
					//转换为空间左边
					float coorX = m_corrX.index2coor(neiborX);
					float coorY = m_corrY.index2coor(neiborY);
					float coorZ = m_corrZ.index2coor(neiborZ);
					if (m_surface[neiborX][neiborY][neiborZ])
						neiborList.push_back(Eigen::Vector3f(coorX, coorY, coorZ));
					else if (m_voxel[neiborX][neiborY][neiborZ])
						innerList.push_back(Eigen::Vector3f(coorX, coorY, coorZ));
				}
			}

	Eigen::Vector3f point(m_corrX.index2coor(indX), m_corrY.index2coor(indY), m_corrZ.index2coor(indZ));

	//构造（3*表面点数量）的矩阵
	Eigen::MatrixXf matA(3, neiborList.size());
	for (int i = 0; i < neiborList.size(); i++)
		//记录方向向量
		matA.col(i) = neiborList[i] - point;
	//构建可求解的方阵
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver(matA * matA.transpose());
	//求解特征值（小于3个）
	Eigen::Vector3f eigenValues = eigenSolver.eigenvalues();
	int indexEigen = 0;
	//👇这个模块的作用是？
	{	if (abs(eigenValues[1]) < abs(eigenValues[indexEigen]))
			indexEigen = 1;
		if (abs(eigenValues[2]) < abs(eigenValues[indexEigen]))
			indexEigen = 2;
	}
	//返回中的某一列（为什么该向量就是法线向量？）
	Eigen::Vector3f normalVector = eigenSolver.eigenvectors().col(indexEigen);
	//innerCenter向量的向量的意思是？
	Eigen::Vector3f innerCenter = Eigen::Vector3f::Zero();
	//对于innerList中的每一个字符
	for (auto const& vec : innerList)//此语句的含义是？
		innerCenter += vec;
	innerCenter /= innerList.size();
	//确定法线正负方向（这句的含义是？）
	if (normalVector.dot(point - innerCenter) < 0)
		normalVector *= -1;
	return normalVector;
}

bool Model::out_of_range(int x, int y, int z)
{
	return (x < 0) || (y < 0) || (z < 0)
		|| (x >= m_corrX.m_resolution)
		|| (y >= m_corrY.m_resolution)
		|| (z >= m_corrZ.m_resolution);
}

bool Model::innerH(int x, int y, int z)
{
	//相机数量
	int prejectionCount = m_projectionList.size();
	//对邻域结点进行检查
	for (int i = 0; i < prejectionCount; i++)
	{
		//index2coor转换为空间中的位置
		float coorX = m_corrX.index2coor(x);
		float coorY = m_corrY.index2coor(y);
		float coorZ = m_corrZ.index2coor(z);
		/*
		*	此处逻辑：当且仅当所有相机拍到当前空间点都为照片轮廓内部点，该点才是内部三维点
		*	只要有一个相机拍到其在外部（落在白色区域），则其为外部三维点
		*/
		if (!m_projectionList[i].checkRange(coorX, coorY, coorZ))return false;
	}
	return true;
}

bool Model::innerH(const Point & p)
{
	return innerH(p._x,p._y,p._z);
}

const int RESILUTION = 300;
int main(int argc, char** argv)
{
	//计时函数
	clock_t t = clock();

	// 分别设置xyz方向的Voxel分辨率
	//此处更改，原先是300
	Model model(RESILUTION, RESILUTION, RESILUTION);

	// 读取相机的内外参数
	model.loadMatrix("../../calibParamsI.txt");

	// 读取投影图片
	model.loadImage("../../wd_segmented", "WD2_", "_00020_segmented.png");

	// 得到Voxel模型，用于判断内部
	model.getModel();
	std::cout << "get model done\n";

	//// 获得Voxel模型的表面，判断表面点函数
	//model.getSurface();
	//std::cout << "get surface done\n";

	// 将模型导出为xyz格式，输出并保存
	model.saveModel("../../WithoutNormal.xyz");
	std::cout << "save without normal done\n";

	model.saveModelWithNormal("../../WithNormal.xyz");
	std::cout << "save with normal done\n";

	system("PoissonRecon.x64 --in ../../WithNormal.xyz --out ../../mesh.ply");
	std::cout << "save mesh.ply done\n";

	t = clock() - t;
	std::cout << "time: " << (float(t) / CLOCKS_PER_SEC) << "seconds\n";

	return (0);
}