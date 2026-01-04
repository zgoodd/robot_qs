// The "Square Detector" program.
// It loads several images sequentially and tries to find squares in
// each image

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <math.h>
#include <string.h>
#include <QDebug>

using namespace cv;
using namespace std;

#define IMG_W   640   // 1280    //640
#define IMG_H    480    //760       480
int ChooseObject =0;

/*
    opencv 的H范围是0~180，红色的H范围大概是(0~8)∪(160,180)
    S是饱和度，一般是大于一个值,S过低就是灰色（参考值S>80)，
    V是亮度，过低就是黑色，过高就是白色(参考值220>V>50)。

CvScalar s;
if (!( (sval.val[0] > 11) && (sval.val[0] < 34) && (sval.val[1] > 150) ))
    // 黄色  (sval.val[0] > 11) && (sval.val[0] < 34)
    // 红色   ((sval.val[0] > 0) && (sval.val[0] < 8)) || ((sval.val[0] > 120) && (sval.val[0] < 180))
    // 黑色  (sval.val[2] < 50)&&(sval.val[1] < 50)
*/



static void help()
{
    cout <<
    "\nA program using pyramid scaling, Canny, contours, contour simpification and\n"
    "memory storage to find squares in a list of images\n"
    "Returns sequence of squares detected on the image.\n"
    "the sequence is stored in the specified memory storage\n"
    "Call:\n"
    "./squares\n"
    "Using OpenCV version %s\n" << CV_VERSION << "\n" << endl;
}


int thresh = 50, N = 5;
//const char* wndname = "Square Detection Demo";

// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
static double angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
static void findSquares( const Mat& image, vector<vector<Point> >& squares )
{
    squares.clear();

//s    Mat pyr, timg, gray0(image.size(), CV_8U), gray;

    // down-scale and upscale the image to filter out the noise
    //pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
    //pyrUp(pyr, timg, image.size());

    // blur will enhance edge detection
    Mat timg(image);
    medianBlur(image, timg, 9);
    Mat gray0(timg.size(), CV_8U), gray;

    vector<vector<Point> > contours;

    // find squares in every color plane of the image
    for( int c = 0; c < 3; c++ )
    {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);

        // try several threshold levels
        for( int l = 0; l < N; l++ )
        {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                Canny(gray0, gray, 5, thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                dilate(gray, gray, Mat(), Point(-1,-1));
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/N;
            }


            // Detect edges using Threshold
              threshold( gray, gray, thresh, 255, THRESH_BINARY );

//              namedWindow( "Detect edges using Threshold", 0 );
//              imshow("Detect edges using Threshold", gray);
//              resizeWindow("Detect edges using Threshold", IMG_W, IMG_H);


            // find contours and store them all as a list
            findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

            vector<Point> approx;

            // test each contour
            for( size_t i = 0; i < contours.size(); i++ )
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if( approx.size() == 4 &&
                    fabs(contourArea(Mat(approx))) > 500 &&      // contourArea像素数  1500
                    isContourConvex(Mat(approx)) )    //是否凸包
                {
                    double maxCosine = 0;

                    for( int j = 2; j < 5; j++ )
                    {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if( maxCosine < 0.4 )    //0.3
                        squares.push_back(approx);
                }
            }
        }
    }
}

// the function draws all the squares in the image
static void drawSquares( Mat& image, const vector<vector<Point> >& squares )
{
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];

        int n = (int)squares[i].size();
        //dont detect the border
        if (p-> x > 3 && p->y > 3)
          polylines(image, &p, &n, 1, true, Scalar(0,255,0), 3, LINE_AA);
    }

    imshow("Square Detection Demo", image);
}



void cannyEdges(const Mat& image,  vector<vector<Point> > contours, int N)
{
    contours.clear();
    // blur will enhance edge detection
    Mat timg(image);  //软复制
    medianBlur(image, timg, 9);
    Mat gray0(timg.size(), CV_8U), gray;

    for( int c = 0; c < 3; c++ )
    {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);

        // try several threshold levels
        for( int l = 0; l < N; l++ )
        {
            vector<vector<Point> > contours_temp;
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                Canny(gray0, gray, 5, thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                dilate(gray, gray, Mat(), Point(-1,-1));
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/N;
            }


            // Detect edges using Threshold
            threshold( gray, gray, thresh, 255, THRESH_BINARY );

            // find contours and store them all as a list
            findContours(gray, contours_temp,  RETR_LIST, CHAIN_APPROX_SIMPLE);
//            contours.push_back(contours_temp);

        }
    }

}





// 判断点是否在矩形中
bool isPointInRect(RotatedRect rectangle, Point2f point)
{
    //转化为轮廓
    Point2f corners[4];
    rectangle.points(corners);
    Point2f *lastItemPointer = (corners+sizeof corners/sizeof corners[0]);
    vector<Point2f> contour(corners,lastItemPointer);
    //判断
    double indicator = pointPolygonTest(contour,point,true);
    if (indicator >= 0) return true;
    else return false;
}


bool isPointInRect(RotatedRect rect, Point P)
{
    Point2f pot[4];
    rect.points(pot);
    Point A = pot[0];
    Point B = pot[1];
    Point C = pot[2];
    Point D = pot[3];

    int x = P.x;
    int y = P.y;
    int a = (B.x - A.x)*(y - A.y) - (B.y - A.y)*(x - A.x);
    int b = (C.x - B.x)*(y - B.y) - (C.y - B.y)*(x - B.x);
    int c = (D.x - C.x)*(y - C.y) - (D.y - C.y)*(x - C.x);
    int d = (A.x - D.x)*(y - D.y) - (A.y - D.y)*(x - D.x);
//    qDebug("a=%d, b=%d, c=%d, d=%d ,x=%d, y=%d,  A=%d %d",a,b,c,d,x,y, A.x, A.y);


    if((a >= 0 && b >= 0 && c >= 0 && d >= 0) || (a <= 0 && b <= 0 && c <= 0 && d <= 0)) {
        return true;
    }

//      AB X AP = (b.x - a.x, b.y - a.y) x (p.x - a.x, p.y - a.y) = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
//      BC X BP = (c.x - b.x, c.y - b.y) x (p.x - b.x, p.y - b.y) = (c.x - b.x) * (p.y - b.y) - (c.y - b.y) * (p.x - b.x);
    return false;
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
            CvScalar s_hsv = cvGet2D(hsv, i, j);//获取像素点为（j, i）点的HSV的值
            /*
                opencv 的H范围是0~180，红色的H范围大概是(0~8)∪(160,180)
                S是饱和度，一般是大于一个值,S过低就是灰色（参考值S>80)，
                V是亮度，过低就是黑色，过高就是白色(参考值220>V>50)。
            */
            CvScalar s;

            if (!( /*(s_hsv.val[0] < 60)&&*/(s_hsv.val[1] < 50)&&(s_hsv.val[2] < 50) ))
                // 黄色  (s_hsv.val[0] > 11) && (s_hsv.val[0] < 34)
                // 红色   ((s_hsv.val[0] > 0) && (s_hsv.val[0] < 8)) || ((s_hsv.val[0] > 120) && (s_hsv.val[0] < 180))
                // 黑色  (s_hsv.val[2] < 50)&&(s_hsv.val[1] < 50)
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











//=========================================================================================================================

//========================================================================================================================
int main(int /*argc*/, char** /*argv*/)
{
    static const char* names[] = { "/home/qxs-kjsp/图片/1.jpg",
                                   "/home/xzy/imgs/2.jpg","/home/xzy/imgs/4.jpg",
                                   0 };
    help();

    for( int i = 0; names[i] != 0; i++ )
    {
        Mat image_origin = imread(names[i], 1);
        if( image_origin.empty() )
        {
            cout << "Couldn't load " << names[i] << endl;
            continue;
        }
//        namedWindow( "image_origin", 0 );
//        imshow("image_origin", image_origin);
//        resizeWindow("image_origin", IMG_W, IMG_H);



//        CvMat color_image_temp=image_origin;
//        CvMat *output;
//        Mat img_colorfilter = colorFilter(&color_image_temp, output);
//        //显示图像
//        imshow("img_colorfilter", img_colorfilter);




        //=========   寻找矩形 - 原始例程     ===============
        if ( 0 )
        {
            const char *wndname = "Square Detection Demo";
            namedWindow( wndname, CV_WINDOW_NORMAL );   //WINDOW_AUTOSIZE
            resizeWindow(wndname, IMG_W, IMG_H);

            vector<vector<Point> > squares;
            Mat image = image_origin.clone();   //深拷贝 各自独立
            findSquares(image, squares);
            drawSquares(image, squares);
            //imwrite( "out", image );
            image.release();  //释放
        }

        //================  分离HSV =============
        if ( 0 )
        {
            Mat src, hsv, dst;
            src = image_origin.clone();
            namedWindow("input", 0);
            imshow("input", src);
            cvtColor(src, hsv, COLOR_BGR2HSV);
            dst.create(hsv.size(), hsv.depth());
            //分离Hue/色相通道
            int ch[] = {0, 0};
            mixChannels(&hsv, 1, &dst, 1, ch, 1);
            namedWindow("H channel", 0);
            imshow("H channel", dst);
            //分离Saturation/饱和度通道
            int ch1[] = {1, 0};
            mixChannels(&hsv, 1, &dst, 1, ch1, 1);
            namedWindow("S channel", 0);
            imshow("S channel", dst);
            //分离Value/色调通道
            int ch2[] = {2, 0};
            mixChannels(&hsv, 1, &dst, 1, ch2, 1);
            namedWindow("V channel", 0);
            imshow("V channel", dst);

            src.release();
        }

        //=========   用各种图形去拟合     ===============
        if ( 0 )
        {
            Mat img = image_origin.clone();
                //    //将原图像转为灰度
                //    cvtColor(img, img, COLOR_RGB2GRAY);
            //        //滤波(降噪)
            //        blur(img, img, Size(3, 3));




            //高斯滤波
            GaussianBlur(img, img, Size(3, 3), 0, 0, BORDER_DEFAULT);
            //Canny检测
            //        int edgeThresh =100;
            Mat canny_out;
            Canny(img, canny_out, 100, 100, 3);
            namedWindow( "canny_out", 0 );
            imshow("canny_out", canny_out);

            //寻找轮廓
            vector<vector<Point>> contours;
            vector<Vec4i> hierachy;
            findContours(canny_out, contours, hierachy, RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(-1,-1));
            drawContours(img, contours, -1, Scalar(0,0,255), 1, 8, hierachy);

            //定义圆形、方形、旋转矩形、椭圆的存储容器
            vector<vector<Point>> contours_ploy(contours.size());
            vector<Rect> rects_ploy(contours.size());
            vector<Point2f> circle_centers(contours.size());
            vector<float> circle_radius(contours.size());
            vector<RotatedRect> RotatedRect_ploy;//注意：由于下面赋值的过程中有个点数大于5的条件，所以这里没有直接初始化，才有下面pushback的方法添加值。
            vector<RotatedRect> ellipse_ploy;//注意，这里是画椭圆，但是容器类型是 RotatedRect

            //将结果放到各自的容器中
            for (size_t i = 0; i< contours.size(); i++)
            {
                approxPolyDP(contours[i], contours_ploy[i], arcLength(Mat(contours[i]), true)*0.05, true);    //精度原 5
                rects_ploy[i] = boundingRect(contours_ploy[i]);
                minEnclosingCircle(contours_ploy[i], circle_centers[i], circle_radius[i]);

                if (contours_ploy[i].size() >5)   // >5
                {
                    RotatedRect temp1 = minAreaRect(contours_ploy[i]);
                    RotatedRect_ploy.push_back(temp1);

                    RotatedRect temp2 = fitEllipse(contours_ploy[i]);
                    ellipse_ploy.push_back(temp2);
                }
            }

            //定义最终绘图的图片
            //        Mat draw_rect(img.size(), img.type(), Scalar::all(0)),
            //            draw_rotateRect(img.size(), img.type(), Scalar::all(0)),
            //            draw_circle(img.size(), img.type(), Scalar::all(0)),
            //            draw_ellipse(img.size(), img.type(), Scalar::all(0));

            Mat draw_rect=image_origin.clone(),
                    draw_rotateRect=image_origin.clone(),
                    draw_circle=image_origin.clone(),
                    draw_ellipse=image_origin.clone();


            //绘图圆形、矩形
            RNG rng(12345);
            for (size_t i = 0; i<contours.size(); i++)
            {
                Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
                rectangle(draw_rect, rects_ploy[i], color, 2, 8);
                circle(draw_circle, circle_centers[i], circle_radius[i], color, 2, 8);
            }
            namedWindow("draw_rect", 0);
            imshow("draw_rect", draw_rect);imwrite("/home/xzy/imgs/draw_rect.jpg", draw_rect);
            namedWindow("draw_circle", 0);
            imshow("draw_circle", draw_circle);imwrite("/home/xzy/imgs/draw_circle.jpg", draw_circle);

            //绘图椭圆形、旋转矩形
            Point2f pot[4];
            for (size_t i = 0; i<ellipse_ploy.size(); i++)
            {
                Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
                ellipse(draw_ellipse, ellipse_ploy[i], color, 2, 8);

                RotatedRect_ploy[i].points(pot);
                for(int j=0; j<4; j++)
                {
                    line(draw_rotateRect, pot[j], pot[(j+1)%4], color, 2, 8);
                }
            }
            namedWindow("draw_ellipse", 0);
            imshow("draw_ellipse", draw_ellipse);
            imwrite("/home/xzy/imgs/draw_ellipse.jpg", draw_ellipse);

            namedWindow("draw_rotateRect", 0);
            imshow("draw_rotateRect", draw_rotateRect);
            imwrite("/home/xzy/imgs/draw_rotateRect.jpg", draw_rotateRect);

            img.release();
        }


        //=========   寻找圆形 - Hough Circle 变换     ===============
        if (0)
        {
            Mat image = image_origin.clone();
            Mat gray;
            cvtColor(image, gray, COLOR_BGR2GRAY);
            medianBlur(gray, gray, 5);
            vector<Vec3f> circles;
            HoughCircles(gray, circles, HOUGH_GRADIENT, 1,
                         gray.rows/16, // change this value to detect circles with different distances to each other
                         100, 30, 20, 2000 // change the last two parameters
                                        // (min_radius & max_radius) to detect larger circles
                         );
            for( size_t i = 0; i < circles.size(); i++ )
            {
                Vec3i c = circles[i];
                circle( image, Point(c[0], c[1]), c[2], Scalar(0,0,255), 3, LINE_AA);
                circle( image, Point(c[0], c[1]), 2, Scalar(0,255,0), 3, LINE_AA);
            }
            namedWindow( "detected_circles", 0 );
            resizeWindow("detected_circles", IMG_W, IMG_H);
            imshow("detected_circles", image);

        }




        //=========   寻找圆形和矩形 - myself     ===============
        if ( 1 )
        {
            Mat image = image_origin.clone();


            Mat color_image=image;
            ChooseObject =0;  //黑色碳纤维管
            CvMat color_image_temp=color_image;
            CvMat *output;
            image = colorFilter(&color_image_temp, output);
            namedWindow( "colorFilter", 0 );
            imshow("colorFilter", image);


            //将原图像转为灰度
            cvtColor(image, image, COLOR_RGB2GRAY);
            //高斯滤波
//            GaussianBlur(image, image, Size(6, 6), 0, 0, BORDER_DEFAULT);
            dilate(image, image, Mat(), Point(-1,-1));

            medianBlur(image, image, 9);
            namedWindow( "COLOR_RGB2GRAY", 0 );
            imshow("COLOR_RGB2GRAY", image);


            //Canny检测
            Mat canny_out;
            Canny(image, canny_out, 20, 50, 3);  //50, 100
            // dilate canny output to remove potential holes between edge segments
//            dilate(canny_out, canny_out, Mat(), Point(-1,-1));

            namedWindow( "canny_out", 0 );
            imshow("canny_out", canny_out);

            // 提取轮廓  不同灰度等级，依次提取


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
            vector<RotatedRect> RotatedRect_ploy;//注意：由于下面赋值的过程中有个点数大于5的条件，所以这里没有直接初始化，才有下面pushback的方法添加值。

            qDebug("contours.size()=%ld", contours.size());

            //将结果放到各自的容器中
            for (size_t i = 0; i< contours.size(); i++)
            {
//              qDebug("hierachy[%ld]:%ld, %ld, %ld, %ld", i, hierachy[i][0], hierachy[i][1], hierachy[i][2], hierachy[i][3]);
                //当前轮廓 i 的后一个轮廓、前一个轮廓、父轮廓和内嵌轮廓的编号索引

                approxPolyDP(contours[i], contours_ploy[i], 5, true);    //精度 arcLength(Mat(contours[i]), true)*0.05
                //                qDebug("contours_ploy[%ld].size()=%ld", i, contours_ploy[i].size());
                minEnclosingCircle(contours_ploy[i], circle_centers[i], circle_radius[i]);
                if (contours_ploy[i].size() >5)   // >5
                {
                    RotatedRect temp1 = minAreaRect(contours_ploy[i]);
                    RotatedRect_ploy.push_back(temp1);
                }
            }



            Mat image_draw=image_origin.clone();
            qDebug("image_draw.rows=%d, col=%d", image_draw.rows, image_draw.cols);   //行列相反
            IplImage tmp=IplImage(image_origin);//转为指针
            CvArr* arr = (CvArr*)&tmp;//转为指针
            IplImage* hsv = cvCreateImage(cvGetSize(arr), 8, 3);;
            cvCvtColor(arr, hsv, CV_BGR2HSV);              //BGR转换成HSV空间  cvCvtColor


            RNG rng(12345);
             // 对圆形进行判断，  尺寸，位置，颜色
            //绘图圆形
#ifdef CC
            for (size_t i = 0; i<contours.size(); i++)
            {
                int draw_now = 0;
                // 判断尺寸  过小过大不考虑
                if ( circle_radius[i]>image_draw.cols/30  &&  circle_radius[i]<image_draw.cols/10 )
                {

                    //判断颜色
                    float rate, num=0;
                    int row_low = circle_centers[i].y - circle_radius[i];
                    if (row_low < 0)row_low = 0;
                    int row_high = circle_centers[i].y + circle_radius[i];
                    if (row_high > image_draw.rows)row_high = image_draw.rows;   //763
                    int col_low = circle_centers[i].x - circle_radius[i];
                    if (col_low < 0)col_low = 0;
                    int col_high = circle_centers[i].x + circle_radius[i];
                    if (col_high > image_draw.rows)col_high = image_draw.rows;

                    //                IplImage* image_draw_arr = image_draw;
                    //                CvSize zzz = cvGetSize(image_draw_arr);
                    //                IplImage* image_rate = cvCreateImage(zzz, 8, 3);  //指向空图像的指针
                    //                cvGetImage(image_draw_arr, image_rate);
                    for (int i = row_low; i < row_high; i++)
                    {
                        for (int j = col_low; j < col_high; j++)
                        {
                            //double inorNot = pointPolygonTest(RotatedRect_ploy[i], (i,j) , false);
                            //返回-1（在contour外部）、0（在contour上）、1（在contour内部
                            CvScalar sval = cvGet2D(hsv, i, j);//获取像素点为（j, i）点的HSV的值
                            if ( (sval.val[2] < 60)&&(sval.val[1] < 60) )     //
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

                    if (rate>0.7)
                    {
                        draw_now = 1;
                        qDebug("GET!! num=%f, rate=%f", num, rate);
                        qDebug("   i=%ld,row_high=%d, row_low=%d,col_high=%d, col_low=%d \n    circle_centers[i].x=%lf, y=%lf, r=%lf",
                               i,row_high, row_low,col_high, col_low, circle_centers[i].x, circle_centers[i].y,circle_radius[i]);
                    }
                    else
                         qDebug("NO GET!!");

                }

                if (draw_now)
                {
                    Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
                    circle(image_draw, circle_centers[i], circle_radius[i], color, 3, 8);
                }
            }

            namedWindow("image_draw", 0);
            imshow("image_draw", image_draw);imwrite("/home/xzy/imgs/image_draw.jpg", image_draw);

#endif



            // 对矩形进行判断，  尺寸，位置，颜色
            //绘图旋转矩形
            Point2f pot[4];
            int valid_num=0;
            for (size_t i = 0; i<RotatedRect_ploy.size(); i++)
            {
//                qDebug("RotatedRect_ploy[%ld]:\n  angle=%f\n  center.x=%lf\n  center.y=%lf\n  size.w=%lf\n  size.h=%lf",
//                       i, RotatedRect_ploy[i].angle,  RotatedRect_ploy[i].center.x, RotatedRect_ploy[i].center.y,
//                       RotatedRect_ploy[i].size.width, RotatedRect_ploy[i].size.height);

                int draw_now = 0;
                // 判断尺寸  过小过大不考虑
                if ( min(RotatedRect_ploy[i].size.width, RotatedRect_ploy[i].size.height)>image_draw.cols/50  &&
                     max(RotatedRect_ploy[i].size.width, RotatedRect_ploy[i].size.height)<image_draw.rows/2)
                {
                    // 判断长宽比
                    float ratioWH=max(RotatedRect_ploy[i].size.width, RotatedRect_ploy[i].size.height)/min(RotatedRect_ploy[i].size.width, RotatedRect_ploy[i].size.height);
                    if (  ratioWH>3 &&  ratioWH<15  )
                    {
                        //判断轮廓里颜色   旋转的矩形，怎么计算轮廓里像素占比？
                        float rate, num=0;
                        RotatedRect_ploy[i].points(pot);
                        int32_t minx = min( min(pot[0].x, pot[1].x), min(pot[2].x, pot[3].x));
                        if (minx<0) minx=0;
                        int32_t   maxx = max( max(pot[0].x, pot[1].x), max(pot[2].x, pot[3].x));
                        if (maxx > image_draw.cols)  maxx = image_draw.cols;
                        int32_t   miny = min( min(pot[0].y, pot[1].y), min(pot[2].y, pot[3].y));
                        if (miny<0) miny=0;
                        int32_t maxy = max( max(pot[0].y, pot[1].y), max(pot[2].y, pot[3].y));
                        if (maxy > image_draw.rows)  maxy = image_draw.rows;
                        qDebug("minx=%d, maxx=%d, miny=%d, maxy=%d,", minx, maxx,miny, maxy);
                        for (int i = minx; i < maxx; i++)
                        {
                            for (int j = miny; j < maxy; j++)
                            {
//                                bool result = isPointInRect(RotatedRect_ploy[i], Point(i, j));
//                                if (result)
//                                {

                                Point A = pot[0];
                                Point B = pot[1];
                                Point C = pot[2];
                                Point D = pot[3];

                                int x = i;
                                int y = j;
                                int a = (B.x - A.x)*(y - A.y) - (B.y - A.y)*(x - A.x);
                                int b = (C.x - B.x)*(y - B.y) - (C.y - B.y)*(x - B.x);
                                int c = (D.x - C.x)*(y - C.y) - (D.y - C.y)*(x - C.x);
                                int d = (A.x - D.x)*(y - D.y) - (A.y - D.y)*(x - D.x);
//                                qDebug("a=%d, b=%d, c=%d, d=%d ,x=%d, y=%d,  A=%d %d",a,b,c,d,x,y, A.x, A.y);

                                if((a >= 0 && b >= 0 && c >= 0 && d >= 0) || (a <= 0 && b <= 0 && c <= 0 && d <= 0))
                                {

                                    CvScalar sval = cvGet2D(hsv, j, i);//注：后面2参数y轴，即height；x轴，即width。
                                    //                                s.val[0] 代表src图像BGR中的B通道的值
                                    if ( (sval.val[2] < 60)&&(sval.val[1] < 60) )
                                    {
                                        num=num+1;
                                    }
                                }
                            }
                        }
//                        rate = num / ( (maxx-minx)*(maxy-miny) );
                        rate = num / ( RotatedRect_ploy[i].size.width * RotatedRect_ploy[i].size.height );
                        qDebug("RotatedRect: num=%f, rate=%f", num, rate);
                        if (rate>0.3)
                        {
                            draw_now = 1;
                            valid_num++;
                            qDebug("    GET!! valid_num = %d \n", valid_num);
                        }
                        else
                            qDebug("NO GET!! \n");

                    }

                }


                if (draw_now)   //draw_now
                {
                    Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
                    RotatedRect_ploy[i].points(pot);

                    for(int j=0; j<4; j++)  line(image_draw, pot[j], pot[(j+1)%4], color, 9, 8);

//                     qDebug("RotatedRect_ploy[%ld].size.width=%f, height=%f",
//                             RotatedRect_ploy[i].size.width, RotatedRect_ploy[i].size.height);
                }
            }
            namedWindow( "image_draw", 0 );
            imshow("image_draw", image_draw);
            imwrite("/home/xzy/imgs/image_draw.jpg", image_draw);

            image.release();
        }






        int c = waitKey();
        if( (char)c == 27 )
            break;
    }

    return 0;
}
