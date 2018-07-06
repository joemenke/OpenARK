#include "ZR300Camera.h"
#include "OkvisSLAMSystem.h"
#include "glfwManager.h"
#include <iostream>

using namespace ark;

int main(int argc, char **argv)
{

    if (argc != 4 && argc != 3 && argc !=2 ) {
        std::cerr << "Usage: ./" << argv[0] << " configuration-yaml-file [vocabulary-file] [skip-first-seconds]";
        return -1;
    }

    google::InitGoogleLogging(argv[0]);

    okvis::Duration deltaT(0.0);
    if (argc == 4) {
        deltaT = okvis::Duration(atof(argv[3]));
    }

    // read configuration file
    std::string configFilename;
    if (argc > 1) configFilename = argv[1];
    else configFilename = "intr.yml";

    std::string vocabFilename;
    if (argc > 2) vocabFilename = argv[2];
    else vocabFilename = "";

    OkvisSLAMSystem slam(vocabFilename, configFilename);

    //setup display
    if (!MyGUI::Manager::init())
    {
       fprintf(stdout, "Failed to initialize GLFW\n");
       return -1;
     }

    printf("Camera initialization started...\n");
    ZR300Camera camera (true);
    camera.writeCalibration("cameras.yaml");

    printf("Camera initialization complete\n");
    fflush(stdout);

    //Window for displaying the path
    MyGUI::CameraWindow path_win("Path Viewer", 1024, 620);
    MyGUI::ImageWindow img_win("Frame Viewer", 640,480, GL_LUMINANCE, GL_UNSIGNED_BYTE);
    MyGUI::Path path1("path1", Eigen::Vector3d(1, 0, 0));
    MyGUI::Axis axis1("axis1", 1);
    MyGUI::Axis axis2("axis2", 1);
    MyGUI::Grid grid1("grid1", 10, 1);
    path_win.add_object(&path1);
    path_win.add_object(&axis1);
    path_win.add_object(&axis2);
    path_win.add_object(&grid1);

    FrameAvailableHandler handler([&path1, &axis2, &img_win](MultiCameraFrame frame) {
        Eigen::Matrix4d eigen_Tcw;
        cv::cv2eigen(frame.vecTcw[0],eigen_Tcw);
        Eigen::Affine3d transform(eigen_Tcw.inverse());
        path1.add_node(transform.translation());
        axis2.set_transform(transform);
        //std::cout << "new frame\n";
        img_win.set_image(frame.images[0]);
    });
    slam.AddFrameAvailableHandler(handler, "mapping");



    //okvis::Time start(0.0);
    //okvis::Time t_imu(0.0); 

    //run until display is closed
    while (MyGUI::Manager::running()){
      //Update the display
        MyGUI::Manager::update();

        camera.nextFrame();

        cv::Mat fisheyeMap = camera.getFishEyeMap();
        //get others, push to vector

        slam.PushIMU(camera.getImuData());
        slam.PushFrame(camera.getFishEyeMap(), camera.getTimeStamp());

        slam.display();
        int k = cv::waitKey(1);
        if (k == 'q' || k == 'Q' || k == 27) break; // 27 is ESC
    }
    printf("\nTerminate...\n");
    // Clean up
    MyGUI::Manager::terminate();
    slam.ShutDown();
    printf("\nExiting...\n");
    return 0;
}