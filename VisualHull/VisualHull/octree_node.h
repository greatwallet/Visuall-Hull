    /* 
    //八叉树节点类，用头节点代表八叉树 
    //采用opengl右手坐标系，靠近原点的那个角为锚点，方便计算 
    //本八叉树的策略是：1，一次划分所有节点，是满树；2，当立方体空间完全包含某物体才剔除,当立方体空间与某物体相交或者完全包含时才查询；3，对象放在完全包含它的区域叶子节点内，非根节点不存储对象，默认为物体不可能跨多个叶子节点，都在一个叶子节点的空间范围内部，未考虑交叉的情况 
    */  
    #pragma once  
    #include <list>  
      
    //八叉树节点类型  
typedef enum
{
	ROOT = -1,              // 根节点  
	BOTTOM_LEFT_FRONT,      // 下左前   
	BOTTOM_RIGHT_FRONT,     // 下右前   
	BOTTOM_LEFT_BACK,       // 下左后   
	BOTTOM_RIGHT_BACK,      // 下右后   
	TOP_LEFT_FRONT,         // 上左前   
	TOP_RIGHT_FRONT,        // 上右前   
	TOP_LEFT_BACK,          // 上左后   
	TOP_RIGHT_BACK          // 上右后     
} OctreeType;
     
struct Point
{
	int x; int y; int z;
	Point(int x = 0, int y = 0, int z = 0)
		:x(x), y(y), z(z) {};
};
  //  template <class T>  
    class OctreeNode  
    {  
    public:  
        OctreeNode(double _x,double _y,double _z,double _xSize,double _ySize,double _zSize,OctreeType _octreeNodeType,int _level,int _maxLevel);  
        //~OctreeNode();   
		//建立八叉树，划分到所有子节点  
		void BuildTree(int level); 
		//返回方块中心点
		Point center();
		//插入对象
 //       void InsertObject(T *object); 
		//查询对象,获得一片区域里的对象链表,考虑包含或相交,由于  
 //       std::list<T *> GetObjectsAt(double px,double py,double pz,double x_size,double y_size,double z_size); 
		//删除对象，删除一片区域里的对象，此处只考虑完全包含的  
 //       void RemoveObjectsAt(double px,double py,double pz,double x_size,double y_size,double z_size); 

		//判断某个区域是否包含某对象  
 //       bool IsContain(double px,double py,double pz,double x_size,double y_size,double z_size,T *object) const; 
		//重载，判断某个区域是否包含某个节点  
 //       bool IsContain(double px,double py,double pz,double x_size,double y_size,double z_size,OctreeNode<T> *octreeNode) const;
		//判断某个区域是否与节点相交，如果相交，则查询时要递归到其子节点  
//        bool IsInterSect(double px,double py,double pz,double x_size,double y_size,double z_size,OctreeNode<T> *octreeNode) const; 
/*    private: */ 
		//节点存储的对象列表  
 //       std::list<T *> objectList;   
        //节点属性   
        OctreeType octreeNodeType;  
        int x;  
        int y;  
        int z;  
        double xSize;  
        double ySize;  
        double zSize;  
        int level;  
        static int maxLevel;  
        //子节点,根据opengl坐标系，依次坐标增大  
		OctreeNode *children_node[8];
/*        OctreeNode *bottom_left_front_node;  
        OctreeNode *bottom_right_front_node;  
        OctreeNode *bottom_left_back_node;  
        OctreeNode *bottom_right_back_node;  
        OctreeNode *top_left_front_node;  
        OctreeNode *top_right_front_node;  
        OctreeNode *top_left_back_node;  
        OctreeNode *top_right_back_node; */ 
	};