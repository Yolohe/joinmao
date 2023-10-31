#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <boost/format.hpp>  // for formating strings
#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>
#include <librealsense2/rs.hpp>
using namespace std;
using namespace cv;

typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

// 在pangolin中画图，已写好，无需调整
void showPointCloud(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud);

int main(int argc, char **argv) {
    cout << "Line 6 - c 的值是 " << endl ;
    rs2::pipeline pipeline;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    pipeline.start(cfg);
    
    // 初始化窗口
    cv::namedWindow("RGB Image", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Depth Image", cv::WINDOW_AUTOSIZE);

    //相机内参
    double fx = 607.19;
    double fy = 607.19;
    double cx = 321.818;
    double cy = 239.120;
    double depthScale = 1000.0;
    vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud;
    pointcloud.reserve(1000000);

    vector<cv::Mat> colorImgs, depthImgs;    // 彩色图和深度图
    TrajectoryType poses;         // 相机位姿

    ifstream fin("../pose.txt");
    if (!fin) {
        cerr << "请在有pose.txt的目录下运行此程序" << endl;
        return 1;
    }
    double data[7] = {0};
    for (auto &d:data)
        fin >> d;
    Sophus::SE3d pose(Eigen::Quaterniond(data[6], data[3], data[4], data[5]),Eigen::Vector3d(data[0], data[1], data[2]));
    poses.push_back(pose);

    while (true) {
        // 等待获取帧数据
        rs2::frameset frames = pipeline.wait_for_frames();
    
        // 获取RGB图像
        rs2::video_frame colorinit = frames.get_color_frame();
        cv::Mat rgb_image(cv::Size(colorinit.get_width(), colorinit.get_height()), CV_8UC3, (void*)colorinit.get_data(), cv::Mat::AUTO_STEP);
    
        // 获取深度图像
        rs2::depth_frame depthinit = frames.get_depth_frame();
        cv::Mat depth_image(cv::Size(depthinit.get_width(), depthinit.get_height()), CV_16UC1, (void*)depthinit.get_data(), cv::Mat::AUTO_STEP);

        // 归一化深度图像以便显示
        double min, max;
        cv::minMaxIdx(depth_image, &min, &max);
        cv::Mat normalized_depth_image;
        cv::convertScaleAbs(depth_image, normalized_depth_image, 255.0 / max);
    
        // 将RGB图像和深度图像显示出来
        cv::imshow("RGB Image", rgb_image);
        cv::imshow("Depth Image", normalized_depth_image);
    
        cv::Mat color = rgb_image;
        cv::Mat depth = normalized_depth_image;

        Sophus::SE3d T = poses[1];
        for (int v = 0; v < color.rows; v++)
            for (int u = 0; u < color.cols; u++) {
                unsigned int d = depth.ptr<unsigned short>(v)[u]; // 深度值
                    if (d == 0) continue; // 为0表示没有测量到
                    Eigen::Vector3d point;
                    point[2] = double(d) / depthScale;
                    point[0] = (u - cx) * point[2] / fx;
                    point[1] = (v - cy) * point[2] / fy;
                    Eigen::Vector3d pointWorld = T * point;

                    Vector6d p;
                    p.head<3>() = pointWorld;
                    p[5] = color.data[v * color.step + u * color.channels()];   // blue
                    p[4] = color.data[v * color.step + u * color.channels() + 1]; // green
                    p[3] = color.data[v * color.step + u * color.channels() + 2]; // red
                    
                    pointcloud.push_back(p);
                    cout<<"S"<<endl;
                }
                 showPointCloud(pointcloud);
                 cout<<"J"<<endl;
        
                // 等待按键事件
                if (cv::waitKey(1) >= 0)
                    break;
            }
         return 0;
}



void showPointCloud(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud) {

    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3d(p[3] / 255.0, p[4] / 255.0, p[5] / 255.0);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        //usleep(5000);   // sleep 5 ms
    }
    return;
}

// #include <iostream>
// #include <librealsense2/rs.hpp>

// using namespace std;
// using namespace rs2;

// int main()
// {
// 	pipeline pipe;
// 	frameset frames;
// 	config cfg;

// 	//cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
// 	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	
// 	pipe.start(cfg);
// 	for (int i = 1; i < 20; i++)pipe.wait_for_frames();
// 	frames = pipe.wait_for_frames();

// 	auto color= frames.get_color_frame();
// 	//auto depth = frames.get_depth_frame();

// 	stream_profile color_profile = color.get_profile();
// 	//stream_profile depth_profile = depth.get_profile();
	
// 	auto rvsprofile = video_stream_profile(color_profile);
// 	//auto dvsprofile = video_stream_profile(depth_profile);
	
// 	rs2_intrinsics a = rvsprofile.get_intrinsics();
// 	//rs2_intrinsics a = dvsprofile.get_intrinsics();
	
// 	cout << "fx:\t" << a.fx << endl;
// 	cout << "fy:\t" << a.fy << endl;
// 	cout << "ppx:\t" << a.ppx << endl;
// 	cout << "ppy:\t" << a.ppy << endl;
// 	return 0;
// }
