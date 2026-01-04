#include<librealsense2/rs.hpp>
#include<librealsense2/rsutil.h>

#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>
#include<string>


#include<ros/ros.h> //ros标准库头文件
#include <cv_bridge/cv_bridge.h>
#include <QDebug>

#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;

//#include<librealsense2/rs.hpp>
//#include<librealsense2/rsutil.h>


int pos_x=0;     //鼠标位置 全局变量
int pos_y=0;


//获取深度像素对应长度单位（米）的换算比例
float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}
//深度图对齐到彩色图函数
Mat align_Depth2Color(Mat depth,Mat color,rs2::pipeline_profile profile){
    //声明数据流
    auto depth_stream=profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream=profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    //获取内参
    const auto intrinDepth=depth_stream.get_intrinsics();
    const auto intrinColor=color_stream.get_intrinsics();

    //直接获取从深度摄像头坐标系到彩色摄像头坐标系的欧式变换矩阵
    //auto  extrinDepth2Color=depth_stream.get_extrinsics_to(color_stream);
    rs2_extrinsics  extrinDepth2Color;
    rs2_error *error;
    rs2_get_extrinsics(depth_stream,color_stream,&extrinDepth2Color,&error);

    //平面点定义
    float pd_uv[2],pc_uv[2];
    //空间点定义
    float Pdc3[3],Pcc3[3];

    //获取深度像素与现实单位比例（D435默认1毫米）
    float depth_scale = get_depth_scale(profile.get_device());
    int y=0,x=0;
    //初始化结果
    //Mat result=Mat(color.rows,color.cols,CV_8UC3,Scalar(0,0,0));
    Mat result=Mat(color.rows,color.cols,CV_16U,Scalar(0));
    //对深度图像遍历
    for(int row=0;row<depth.rows;row++){
        for(int col=0;col<depth.cols;col++){
            //将当前的(x,y)放入数组pd_uv，表示当前深度图的点
            pd_uv[0]=col;
            pd_uv[1]=row;
            //取当前点对应的深度值
            uint16_t depth_value=depth.at<uint16_t>(row,col);
            //换算到米
            float depth_m=depth_value*depth_scale;
            //将深度图的像素点根据内参转换到深度摄像头坐标系下的三维点
            rs2_deproject_pixel_to_point(Pdc3,&intrinDepth,pd_uv,depth_m);
            //将深度摄像头坐标系的三维点转化到彩色摄像头坐标系下
            rs2_transform_point_to_point(Pcc3,&extrinDepth2Color,Pdc3);
            //将彩色摄像头坐标系下的深度三维点映射到二维平面上
            rs2_project_point_to_pixel(pc_uv,&intrinColor,Pcc3);

            //取得映射后的（u,v)
            x=(int)pc_uv[0];
            y=(int)pc_uv[1];
//            if(x<0||x>color.cols)
//                continue;
//            if(y<0||y>color.rows)
//                continue;
            //最值限定
            x=x<0? 0:x;
            x=x>depth.cols-1 ? depth.cols-1:x;
            y=y<0? 0:y;
            y=y>depth.rows-1 ? depth.rows-1:y;

            result.at<uint16_t>(y,x)=depth_value;
        }
    }
    //返回一个与彩色图对齐了的深度信息图像
    return result;
}

void measure_distance(Mat &color,Mat depth,cv::Size range,rs2::pipeline_profile profile,  int x, int y)
{
    //获取深度像素与现实单位比例（D435默认1毫米）
    float depth_scale = get_depth_scale(profile.get_device());
    //定义测量位置
    // cv::Point center(color.cols/2,color.rows/2);
    cv::Point center(x,  y);
    cout<<"x:"<<x<<";     y"<<y<<endl;

    //定义计算距离的范围
    cv::Rect RectRange(center.x-range.width/2,center.y-range.height/2,range.width,range.height);
    //遍历该范围
    float distance_sum=0;
    int effective_pixel=0;
    for(int y=RectRange.y;y<RectRange.y+RectRange.height;y++){
        for(int x=RectRange.x;x<RectRange.x+RectRange.width;x++){
            //如果深度图下该点像素不为0，表示有距离信息
            if(depth.at<uint16_t>(y,x)){
                distance_sum+=depth_scale*depth.at<uint16_t>(y,x);
                effective_pixel++;
            }
        }
    }
    cout<<"遍历完成，有效像素点:"<<effective_pixel<<endl;
    float effective_distance=distance_sum/effective_pixel;
    cout<<"目标距离："<<effective_distance<<" m"<<endl;
    char distance_str[30];
    sprintf(distance_str,"The distance is:%f m",effective_distance);
    cv::rectangle(color,RectRange,Scalar(0,0,255),2,8);
    cv::putText(color,(string)distance_str,cv::Point(color.cols*0.02,color.rows*0.05),
                cv::FONT_HERSHEY_PLAIN,2,Scalar(0,255,0),2,8);
}



// 鼠标选点
void on_mouse(int EVENT, int x, int y, int flags, void* userdata)
{
    // cout<<"   测试测试   "<<endl;

    Mat hh;
    hh = *(Mat*)userdata;
    char Txt_Point[20];
    Point pp(x, y);

    switch (EVENT)
    {
        case  EVENT_LBUTTONDOWN:     //左键单击
        {
            sprintf(Txt_Point, "(%d,%d)", x, y);
            putText(hh, Txt_Point, pp,	FONT_HERSHEY_SIMPLEX, 2.0, Scalar(0, 0, 255), 2, 8);      //依次参数：图片，带put内容，位置，字体，字体大小，颜色，粗细，线型
            circle(hh, pp, 8, Scalar(0, 0, 255), 2, CV_AA, 0);//画圆依次参数：图片，圆心位置，半径，颜色，线宽(如<0则实心)，线型，圆心和半径小数点位数

            pos_x=x;
            pos_y=y;
        }
        break;
    }
    imshow("color_Image", hh);
}



/* ===================================  取出颜色  =================================================================*/
//截取图像中红色区域
Mat colorFilter(CvMat *inputImage, CvMat *&outputImage)   // (CvMat *inputImage, CvMat *&outputImage)
{
    //inputImage为指向输入图片地址的指针,outputImage 为指向一个cvMat的指针
    int i, j;
    IplImage* image = cvCreateImage(cvGetSize(inputImage), 8, 3);  //指向空图像的指针
    cvGetImage(inputImage, image);
    IplImage* hsv = cvCreateImage(cvGetSize(image), 8, 3);
    cvCvtColor(image, hsv, CV_BGR2HSV);              //BGR转换成HSV空间  cvCvtColor
    int width = hsv->width;
    int height = hsv->height;
    for (i = 0; i < height; i++)
        for (j = 0; j < width; j++)
        {
            CvScalar sval = cvGet2D(hsv, i, j);//获取像素点为（j, i）点的HSV的值
            /*
                opencv 的H范围是0~180，红色的H范围大概是(0~8)∪(160,180)
                S是饱和度，一般是大于一个值,S过低就是灰色（参考值S>80)，
                V是亮度，过低就是黑色，过高就是白色(参考值220>V>50)。
            */
            CvScalar s;

                if (!( ((sval.val[0] > 0) && (sval.val[0] < 8)) || ((sval.val[0] > 120) && (sval.val[0] < 180)) &&  (sval.val[1] > 150) ))
                {
                    s.val[0] = 255;  //其他以外的都白色（255,255,255）或黑色（0,0,0）
                    s.val[1] = 255;
                    s.val[2] = 255;
                    cvSet2D(hsv, i, j, s);    //把新图像s的值，赋给hsv指针所指向的图像
                }


        }
    outputImage = cvCreateMat(hsv->height, hsv->width, CV_8UC3);   //cvCreateMat
    cvConvert(hsv, outputImage);         //把IplImage图像hsv的值转给矩阵outputImage
    Mat output = cvarrToMat(hsv);
    return output;
}







//=====================================================================================

//=====================================================================================

//=====================================================================================
//=====================================================================================
int main()
{

    //深度图像颜色map
    rs2::colorizer c;                          // Helper to colorize depth images

    //创建数据管道
    rs2::pipeline pipe;
    rs2::config pipe_config;
    pipe_config.enable_stream(RS2_STREAM_DEPTH,1280,720,RS2_FORMAT_Z16,30);          //XIE 图像尺寸 640,480
    pipe_config.enable_stream(RS2_STREAM_COLOR,1280,720,RS2_FORMAT_BGR8,30);

    //start()函数返回数据管道的profile
    rs2::pipeline_profile profile = pipe.start(pipe_config);

    //定义一个变量去转换深度到距离
    float depth_clipping_distance = 1.f;

    //声明数据流
    auto depth_stream=profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream=profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    //获取内参
    auto intrinDepth=depth_stream.get_intrinsics();
    auto intrinColor=color_stream.get_intrinsics();

    //直接获取从深度摄像头坐标系到彩色摄像头坐标系的欧式变换矩阵
    auto  extrinDepth2Color=depth_stream.get_extrinsics_to(color_stream);

//    const char* depth_win="depth_Image";
//    namedWindow(depth_win,WINDOW_AUTOSIZE);
    const char* color_win="color_Image";                // 窗口名字
    namedWindow(color_win,CV_WINDOW_NORMAL);  //WINDOW_AUTOSIZE
    resizeWindow(color_win, 720, 480);
    moveWindow(color_win, 0, 100);

    while ( cvGetWindowHandle(color_win) ) // Application still alive?
    {
        //堵塞程序直到新的一帧捕获
        rs2::frameset frameset = pipe.wait_for_frames();
        //取深度图和彩色图
        rs2::frame color_frame = frameset.get_color_frame();//processed.first(align_to);
        rs2::frame depth_frame = frameset.get_depth_frame();
//        rs2::frame depth_frame_4_show = frameset.get_depth_frame().apply_filter(c);
        //获取宽高
        const int depth_w=depth_frame.as<rs2::video_frame>().get_width();
        const int depth_h=depth_frame.as<rs2::video_frame>().get_height();
        const int color_w=color_frame.as<rs2::video_frame>().get_width();
        const int color_h=color_frame.as<rs2::video_frame>().get_height();

        //创建OPENCV类型 并传入数据
        Mat depth_image(Size(depth_w,depth_h),
                                CV_16U,(void*)depth_frame.get_data(),Mat::AUTO_STEP);
//        Mat depth_image_4_show(Size(depth_w,depth_h),
//                                CV_8UC3,(void*)depth_frame_4_show.get_data(),Mat::AUTO_STEP);
        Mat color_image(Size(color_w,color_h),
                                CV_8UC3,(void*)color_frame.get_data(),Mat::AUTO_STEP);

        //获取鼠标点击点坐标
//        setMouseCallback("color_Image", on_mouse, &color_image);

//        if (pos_x >0 )
//        {
//            cout<<"x:"<<pos_x<<";  y="<<pos_y<<endl;

//            //实现深度图对齐到彩色图
//            Mat result=align_Depth2Color(depth_image,color_image,profile);
//            //测量距离
//            measure_distance(color_image,result,cv::Size(5,5),profile, pos_x, pos_y);

//            pos_x=0;
//            pos_y=0;
//        }

//        //显示
//        const char* depth_win="depth_Image";
//        namedWindow(depth_win,WINDOW_AUTOSIZE);
//        const char* color_win="color_Image";                // 窗口名字
//        namedWindow(color_win,WINDOW_AUTOSIZE);
//        imshow(depth_win,depth_image_4_show);
//        imshow(color_win,color_image);
//        // imshow("result",result);




        //=========   寻找圆形 - myself     ===============

            Mat image = color_image.clone();
            Mat image_draw=color_image;   //color_image.clone()

            //高斯滤波
            GaussianBlur(image, image, Size(5, 5), 0, 0, BORDER_DEFAULT);


//            CvMat color_image_temp=image;
//            CvMat *output;
//            Mat img_colorfilter = colorFilter(&color_image_temp, output);
//            //显示图像
//            imshow("img_colorfilter", img_colorfilter);

//            image = img_colorfilter;


            //Canny检测
            Mat canny_out;
            Canny(image, canny_out, 100, 100, 3);
//            namedWindow( "canny_out", 0 );
//            imshow("canny_out", canny_out);



            //寻找轮廓
            vector<vector<Point>> contours;
            vector<Vec4i> hierachy;
            findContours(canny_out, contours, hierachy, RETR_EXTERNAL, CHAIN_APPROX_TC89_L1, Point(-1,-1));
            //轮廓的四种寻找方式： - RETR_LIST：所有轮廓属于同一层级 - RETR_TREE: 完整建立轮廓的各属性
            //                 - RETR_EXTERNAL: 只寻找最高层级的轮廓 - RETR_CCOMP: 所有轮廓分2个层级，不是外界就是最里层
            drawContours(image, contours, -1, Scalar(0,0,255), 2, 8, hierachy);

//            namedWindow( "img_contours", 0 );
//            imshow("img_contours", image);

            //定义圆形、旋转矩形的存储容器
            vector<vector<Point>> contours_ploy(contours.size());
            vector<Point2f> circle_centers(contours.size());
            vector<float> circle_radius(contours.size());

            qDebug("contours.size()=%ld", contours.size());

            //将结果放到各自的容器中
            for (size_t i = 0; i< contours.size(); i++)
            {
//              qDebug("hierachy[%ld]:%ld, %ld, %ld, %ld", i, hierachy[i][0], hierachy[i][1], hierachy[i][2], hierachy[i][3]);
                //当前轮廓 i 的后一个轮廓、前一个轮廓、父轮廓和内嵌轮廓的编号索引

                approxPolyDP(contours[i], contours_ploy[i], 5, true);    //精度 arcLength(Mat(contours[i]), true)*0.05
                //                qDebug("contours_ploy[%ld].size()=%ld", i, contours_ploy[i].size());
                minEnclosingCircle(contours_ploy[i], circle_centers[i], circle_radius[i]);
            }


            qDebug("image_draw.rows=%d, col=%d", image_draw.rows, image_draw.cols);   //行列相反
            IplImage tmp=IplImage(image);//转为指针
            CvArr* arr = (CvArr*)&tmp;//转为指针
            IplImage* hsv = cvCreateImage(cvGetSize(arr), 8, 3);;
            cvCvtColor(arr, hsv, CV_BGR2HSV);              //BGR转换成HSV空间  cvCvtColor


            RNG rng(12345);
             // 对圆形进行判断，  尺寸，位置，颜色
            //绘图圆形

            for (size_t i = 0; i<contours.size(); i++)
            {
                int draw_now = 0;
                // 判断尺寸  过小过大不考虑
                if ( circle_radius[i]>5  &&  circle_radius[i]< image_draw.cols/15 )     // image_draw.cols/20
                {

                    //判断颜色
                    float rate, num=0;
                    int row_low = circle_centers[i].y - circle_radius[i];
                    if (row_low < 0)row_low = 0;
                    int row_high = circle_centers[i].y + circle_radius[i];
                    if (row_high > image_draw.rows) row_high = image_draw.rows;   //763
                    int col_low = circle_centers[i].x - circle_radius[i];
                    if (col_low < 0)col_low = 0;
                    int col_high = circle_centers[i].x + circle_radius[i];
                    if (col_high > image_draw.cols) col_high = image_draw.cols;


                    for (int i = row_low; i < row_high; i++)
                    {
                        for (int j = col_low; j < col_high; j++)
                        {
                            //double inorNot = pointPolygonTest(RotatedRect_ploy[i], (i,j) , false);
                            //返回-1（在contour外部）、0（在contour上）、1（在contour内部
                            CvScalar sval = cvGet2D(hsv, i, j);//获取像素点为（j, i）点的HSV的值
//                            if (   (sval.val[1] > 180) && (sval.val[1] < 230) && (sval.val[2] > 120)&& (sval.val[2] < 200)    )     //
                            if ( ((sval.val[0] > 0) && (sval.val[0] < 8)) || ((sval.val[0] > 120) && (sval.val[0] < 180)) &&  (sval.val[1] > 150))
                                // 黄色  (sval.val[0] > 11) && (sval.val[0] < 34)
                                // 红色   ((sval.val[0] > 0) && (sval.val[0] < 8)) || ((sval.val[0] > 120) && (sval.val[0] < 180))
                                // 黑色  (sval.val[2] < 50)&&(sval.val[1] < 50)
                            {
                                num=num+1;
                            }
                        }

                    }
//                    rate = num / ( 4*circle_radius[i]*circle_radius[i] );
                    rate = 4/3.14159*num / ((row_high - row_low)*(col_high - col_low));   //矩形和圆形

                    if (rate>0.5)
                    {
                        draw_now = 1;
                        qDebug("GET!! num=%f, rate=%f", num, rate);
                        qDebug("   i=%ld,row_high=%d, row_low=%d,col_high=%d, col_low=%d", i,row_high, row_low,col_high, col_low);
                        qDebug("   圆心和半径：circle_centers[i].x=%lf, y=%lf, r=%lf", circle_centers[i].x, circle_centers[i].y,circle_radius[i]);
                        CvScalar sval2 = cvGet2D(hsv, circle_centers[i].y, circle_centers[i].x);//获取像素点为（x,y）点的HSV的值
                        qDebug("   圆心处HSV:sval.val[0,1,2] = %f, %f, %f", sval2.val[0], sval2.val[1], sval2.val[2]);
                    }
//                    else
//                         qDebug("NO GET!!");

                }

                if (draw_now)
                {
                    Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
                    circle(image_draw, circle_centers[i], circle_radius[i], color, 3, 8);

                    //实现深度图对齐到彩色图
                    Mat result=align_Depth2Color(depth_image,color_image,profile);
                    //测量距离
                    measure_distance(image_draw,result,cv::Size(5,5),profile, circle_centers[i].x, circle_centers[i].y);


                }
            }

//            namedWindow("image_draw", 0);
            imshow("color_Image", image_draw);
            imwrite("/home/qxs-kjsp/图片/image_draw.jpg", image_draw);





        waitKey(300);    //XIE   刷新 40ms
    }
    return 0;
}












////=========================================================================================================================

////========================================================================================================================
//int main(int /*argc*/, char** /*argv*/)
//{
//    static const char* names[] = { "/home/qxs-kjsp/图片/2.png", "/home/qxs-kjsp/图片/3.png",
//                                   0 };
//    help();

//    for( int i = 0; names[i] != 0; i++ )
//    {
//        Mat image_origin = imread(names[i], 1);
//        if( image_origin.empty() )
//        {
//            cout << "Couldn't load " << names[i] << endl;
//            continue;
//        }
////        namedWindow( "image_origin", 0 );
////        imshow("image_origin", image_origin);
////        resizeWindow("image_origin", IMG_W, IMG_H);




//        //=========   寻找圆形 - myself     ===============

//            Mat image = image_origin.clone();

//            //高斯滤波
//            GaussianBlur(image, image, Size(3, 3), 0, 0, BORDER_DEFAULT);
//            //Canny检测
//            //        int edgeThresh =100;
//            Mat canny_out;
//            Canny(image, canny_out, 100, 100, 3);
//            namedWindow( "canny_out", 0 );
//            imshow("canny_out", canny_out);



//            //寻找轮廓
//            vector<vector<Point>> contours;
//            vector<Vec4i> hierachy;
//            findContours(canny_out, contours, hierachy, RETR_EXTERNAL, CHAIN_APPROX_TC89_L1, Point(-1,-1));
//            //轮廓的四种寻找方式： - RETR_LIST：所有轮廓属于同一层级 - RETR_TREE: 完整建立轮廓的各属性
//            //                 - RETR_EXTERNAL: 只寻找最高层级的轮廓 - RETR_CCOMP: 所有轮廓分2个层级，不是外界就是最里层
//            drawContours(image, contours, -1, Scalar(0,0,255), 2, 8, hierachy);

////            namedWindow( "img_contours", 0 );
////            imshow("img_contours", image);

//            //定义圆形、旋转矩形的存储容器
//            vector<vector<Point>> contours_ploy(contours.size());
//            vector<Point2f> circle_centers(contours.size());
//            vector<float> circle_radius(contours.size());

//            qDebug("contours.size()=%ld", contours.size());

//            //将结果放到各自的容器中
//            for (size_t i = 0; i< contours.size(); i++)
//            {
////              qDebug("hierachy[%ld]:%ld, %ld, %ld, %ld", i, hierachy[i][0], hierachy[i][1], hierachy[i][2], hierachy[i][3]);
//                //当前轮廓 i 的后一个轮廓、前一个轮廓、父轮廓和内嵌轮廓的编号索引

//                approxPolyDP(contours[i], contours_ploy[i], 5, true);    //精度 arcLength(Mat(contours[i]), true)*0.05
//                //                qDebug("contours_ploy[%ld].size()=%ld", i, contours_ploy[i].size());
//                minEnclosingCircle(contours_ploy[i], circle_centers[i], circle_radius[i]);
//            }


//            Mat image_draw=image_origin.clone();
//            qDebug("image_draw.rows=%d, col=%d", image_draw.rows, image_draw.cols);   //行列相反
//            IplImage tmp=IplImage(image_origin);//转为指针
//            CvArr* arr = (CvArr*)&tmp;//转为指针
//            IplImage* hsv = cvCreateImage(cvGetSize(arr), 8, 3);;
//            cvCvtColor(arr, hsv, CV_BGR2HSV);              //BGR转换成HSV空间  cvCvtColor


//            RNG rng(12345);
//             // 对圆形进行判断，  尺寸，位置，颜色
//            //绘图圆形

//            for (size_t i = 0; i<contours.size(); i++)
//            {
//                int draw_now = 0;
//                // 判断尺寸  过小过大不考虑
//                if ( circle_radius[i]>5  &&  circle_radius[i]< 15 )     // image_draw.cols/20
//                {

//                    //判断颜色
//                    float rate, num=0;
//                    int row_low = circle_centers[i].y - circle_radius[i];
//                    if (row_low < 0)row_low = 0;
//                    int row_high = circle_centers[i].y + circle_radius[i];
//                    if (row_high > image_draw.rows)row_high = image_draw.rows;   //763
//                    int col_low = circle_centers[i].x - circle_radius[i];
//                    if (col_low < 0)col_low = 0;
//                    int col_high = circle_centers[i].x + circle_radius[i];
//                    if (col_high > image_draw.rows)col_high = image_draw.rows;

//                    for (int i = row_low; i < row_high; i++)
//                    {
//                        for (int j = col_low; j < col_high; j++)
//                        {
//                            //double inorNot = pointPolygonTest(RotatedRect_ploy[i], (i,j) , false);
//                            //返回-1（在contour外部）、0（在contour上）、1（在contour内部
//                            CvScalar sval = cvGet2D(hsv, i, j);//获取像素点为（j, i）点的HSV的值
//                            if (  (sval.val[1] > 180) && (sval.val[1] < 230) && (sval.val[2] > 120)&& (sval.val[2] < 200)   )     //
//                                // 黄色  (sval.val[0] > 11) && (sval.val[0] < 34)
//                                // 红色   ((sval.val[0] > 0) && (sval.val[0] < 8)) || ((sval.val[0] > 120) && (sval.val[0] < 180))
//                                // 黑色  (sval.val[2] < 50)&&(sval.val[1] < 50)
//                            {
//                                num=num+1;
//                            }
//                        }

//                    }
////                    rate = num / ( 4*circle_radius[i]*circle_radius[i] );
//                    rate = 4/3.14159*num / ((row_high - row_low)*(col_high - col_low));   //矩形和圆形

//                    if (rate>0.6)
//                    {
//                        draw_now = 1;
//                        qDebug("GET!! num=%f, rate=%f", num, rate);
//                        qDebug("   i=%ld,row_high=%d, row_low=%d,col_high=%d, col_low=%d", i,row_high, row_low,col_high, col_low);
//                        qDebug("   圆心和半径：circle_centers[i].x=%lf, y=%lf, r=%lf", circle_centers[i].x, circle_centers[i].y,circle_radius[i]);
//                        CvScalar sval2 = cvGet2D(hsv, circle_centers[i].y, circle_centers[i].x);//获取像素点为（x,y）点的HSV的值
//                        qDebug("   圆心处HSV:sval.val[0,1,2] = %f, %f, %f", sval2.val[0], sval2.val[1], sval2.val[2]);
//                    }
//                    else
//                         qDebug("NO GET!!");

//                }

//                if (draw_now)
//                {
//                    Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
//                    circle(image_draw, circle_centers[i], circle_radius[i], color, 3, 8);
//                }
//            }

//            namedWindow("image_draw", 0);
//            imshow("image_draw", image_draw);
//            imwrite("/home/qxs-kjsp/图片/image_draw.jpg", image_draw);


////            image.release();


//        int c = waitKey();
//        if( (char)c == 27 )
//            break;
//    }

//    return 0;
//}
