//相关头文件

//C/C++系统文件

//第三方库文件
#include <opencv/cv.hpp>
#include <opencv2/core.hpp>

#include <pcl-1.9/pcl/pcl_base.h>

//项目内文件

class JoinPointcloud
{
private:
    /* data */
public:
    JoinPointcloud(/* args */);
    ~JoinPointcloud();
    void JoinPointcloud::CombinePointcloud();
    void JoinPointcloud::ReadFrame();
    void JoinPointcloud::CvMat2Eigen();
};