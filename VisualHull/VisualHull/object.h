    #pragma once  
    /* 
    //被管理的对象类 
    */  
    class Object  
    {  
    public:  
        Object(double _x,double _y,double _z,double _xSize,double _ySize,double _zSize);  
        ~Object();  
    public:  
        //对象的属性，例如坐标和长宽高，以左上角为锚点  
        double x;  
        double y;  
        double z;  
        double xSize;  
        double ySize;  
        double zSize;  
    };  