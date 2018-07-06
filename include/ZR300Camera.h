#pragma once
// OpenCV Libraries
#include "stdafx.h"
#include "version.h"

// OpenARK Libraries
#include "DepthCamera.h"

// RealSense SDK
#include <librealsense/rs.hpp>

//Eigen
#include <Eigen/Core>

#include "SingleConsumerPriorityQueue.h"
#include "Types.h"

namespace ark {
    class ZR300Camera : public DepthCamera{
        public: 
            struct FrameObject {
                std::shared_ptr<rs::frame> frame;
                bool operator<(const FrameObject& right) const
                {
                    return frame->get_timestamp() > right.frame->get_timestamp();
                }

                double operator-(const FrameObject& right) const
                {
                    if(frame!=nullptr&&right.frame!=nullptr)
                        return ((frame->get_timestamp())-(right.frame->get_timestamp()));
                    if(frame==nullptr)
                        std::cerr <<"ERROR in FrameObject -, LEFT NULLPTR" <<std::endl;        
                    if(right.frame==nullptr)
                        std::cerr <<"ERROR in FrameObject -, RIGHT NULLPTR" <<std::endl;
                    return 0;
                }

            };


            struct CameraCalibration{
            private:
                std::vector<int> dimensions;
                std::vector<float> coeffs;
                std::vector<float> focal;
                std::vector<float> principal;
                Eigen::Matrix4f transform;
            public:
                CameraCalibration(Eigen::Matrix4f transform, rs::intrinsics intr):
                transform(transform)
                {
                    dimensions = {intr.width,intr.height};
                    coeffs = {intr.coeffs[0], intr.coeffs[1], intr.coeffs[2], intr.coeffs[3]};
                    focal = {intr.fx, intr.fy};
                    principal = { intr.ppx, intr.ppy};
                }

                CameraCalibration(){}

                void write(cv::FileStorage& fs) const                        //Write serialization for this class
                {
                    fs << "{";
                    //std::cout << transform << std::endl;
                    std::vector<float> t_sc(transform.data(), transform.data() + transform.rows() * transform.cols());
                    fs << "T_SC" << t_sc;
                    fs << "image_dimension" << dimensions;
                    fs << "distortion_type" << "radialtangential";
                    fs << "distortion_coefficients" << coeffs;
                    fs << "focal_length" << focal;
                    fs << "principal_point" << principal;
                    fs << "}";
                }

                void read(const cv::FileNode& node)                          //Read serialization for this class
                {
                }

            };

            /** @param require_motion if true, require motion module to be available */
            explicit ZR300Camera(bool require_motion = false);

            ~ZR300Camera()  override;

            /**
             * Returns the width of the frame in pixels. 
             */
            int getWidth() const override {
                return REAL_WID;
            };

            /**
             * Returns the height of the frame in pixels. 
             */
            int getHeight() const override {
                return REAL_HI;
            };

            /**
             * Returns true if an RGB image is available from this camera.
             * @return true if an RGB image is available from this camera.
             */
            bool hasRGBMap() const override;

            /** Get the current RGB image from this camera (frame index 1) */
            const cv::Mat getRGBMap() const override;

            /**
             * Returns true if a fisheye image is available from this camera.
             * @return true if a fisheye image is available from this camera.
             */
            bool hasFishEyeMap() const override;

            /** Get the current FishEye image from this camera (frame index 2) */
            const cv::Mat getFishEyeMap() const override;

            /**
             * Update the IMU measurment queue up until the given time
             */
            void updateImuToTime(double time);

            /**
             * Get the stored IMU measurments corresponding to the last camera frame
             * @return pointer to vector of ordered and syncronized imu measurements
             */
            std::vector<ImuPair> getImuData() const;

            const double getTimeStamp() const;

            /**
             * writes a camera calibration file to the specified filename should be .yaml
             */
            void writeCalibration(std::string filename);

        protected:
            void update(MultiCameraFrame & frame);

        private:

            cv::Point3f deproject(cv::Point2f pixelPoint, cv::Mat intrinsics, float depth);


            /**
             * Initializat the camera
             */
            void initCamera();
            static const int REAL_WID = 640, REAL_HI = 480;
            static const int IMU_SYNC_THRESHOLD =4;
            static const int FRAME_SYNC_THRESHOLD =15;
            rs::context mCtx;
            rs::device * mpDev;
            rs::intrinsics mDepth_intrin;
            rs::intrinsics mColor_intrin;
            cv::Mat depth_intrinsics;
            std::vector<ImuPair> imuData;
            float mDepthScale;
            double mTimeStamp;
            double mTimeImu;
            SingleConsumerPriorityQueue<ImuMeasurement> imu_queue;
            SingleConsumerPriorityQueue<FrameObject> depth_queue;
            SingleConsumerPriorityQueue<FrameObject> fisheye_queue;
            SingleConsumerPriorityQueue<FrameObject> rgb_queue;
            std::vector<CameraCalibration> slam_camera_calibs;
            std::vector<CameraCalibration> additional_camera_calibs;
            bool requireMotion;
    };
}
