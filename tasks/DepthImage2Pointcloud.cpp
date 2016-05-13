/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "DepthImage2Pointcloud.hpp"
#include <fstream>

using namespace image_preprocessing;

DepthImage2Pointcloud::DepthImage2Pointcloud(std::string const& name, TaskCore::TaskState initial_state)
    : DepthImage2PointcloudBase(name, initial_state)
{
}

DepthImage2Pointcloud::DepthImage2Pointcloud(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : DepthImage2PointcloudBase(name, engine, initial_state)
{
}

DepthImage2Pointcloud::~DepthImage2Pointcloud()
{
}

void DepthImage2Pointcloud::color_frameCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &color_frame_sample)
{
    color_frame = color_frame_sample;
}

void DepthImage2Pointcloud::frameCallback(const base::Time &ts, const ::base::samples::DistanceImage &frame_sample)
{
    base::samples::Pointcloud pc;
    pc.time = frame_sample.time;
    Eigen::Vector3d v;
    if(color_frame.get()){
        assert(frame_sample.width == color_frame->getWidth() && frame_sample.height == color_frame->getHeight());
        for(int x = 0; x<frame_sample.width;x++){
            for(int y = 0; y<frame_sample.height;y++){
                    if(!frame_sample.getScenePoint(x,y,v)) continue;
                    pc.points.push_back(v);
                    const uint8_t *pos = &color_frame->getImage()[(color_frame->getWidth()*3.0)*y+(x*3.0)]; 
                    pc.colors.push_back(Eigen::Vector4d((*pos)/255.0,(*(pos+1))/255.0,(*(pos+2))/255.0,1.0f));
            }
        }
    }else{
        for(int x = 0; x<frame_sample.width;x++){
            for(int y = 0; y<frame_sample.height;y++){
                    if(!frame_sample.getScenePoint(x,y,v)) continue;
                    pc.points.push_back(v);
            }
        }
    }
    std::cout << "PointCloud debug line" << std::endl;
    _pointcloud.write(pc);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See DepthImage2Pointcloud.hpp for more detailed
// documentation about them.

bool DepthImage2Pointcloud::configureHook()
{
    if (! DepthImage2PointcloudBase::configureHook())
        return false;
    index=1;
    index2=1;
    return true;
}
bool DepthImage2Pointcloud::startHook()
{
    if (! DepthImage2PointcloudBase::startHook())
        return false;
    return true;
}
void DepthImage2Pointcloud::updateHook()
{
    DepthImage2PointcloudBase::updateHook();
    if (_color_frame.read(color_frame) == RTT::NewData)
        color_frameCallback(color_frame->time, color_frame);
    if (_frame.read(frame) == RTT::NewData)
    {
        char filename[240];
        sprintf (filename, "/home/exoter/Desktop/Images/distance_frame_%s_%d.txt", _camera_id.get().c_str(), index++);
        //frameCallback(frame.time, frame);
        std::ofstream distance_frame;
        distance_frame.open(filename);
        distance_frame << "width:  " << frame.width << std::endl;
        distance_frame << "height:  " << frame.height << std::endl;
        distance_frame << "scale_x:  " << frame.scale_x << std::endl;
        distance_frame << "scale_y:  " << frame.scale_y << std::endl;
        distance_frame << "center_x:  " << frame.center_x << std::endl;
        distance_frame << "center_y:  " << frame.center_y << std::endl;

        for (int j=0;j<frame.height;j++)
        {
            for (int i=0;i<frame.width;i++)
            {
                distance_frame << frame.data[frame.width*j+i] << std::endl;
            }
        }
        distance_frame.close();
        std::cout << "Distance Frame stored" << std::endl;
    }
    if (_disparity_frame.read(disparity_frame) == RTT::NewData){
        char filename2[240];
        sprintf (filename2, "/home/exoter/Desktop/Images/disparity_frame_%s_%d.jpg", _camera_id.get().c_str(), index2++);
        disp_helper.saveFrame(filename2, disparity_frame);
    }


}
void DepthImage2Pointcloud::errorHook()
{
    DepthImage2PointcloudBase::errorHook();
}
void DepthImage2Pointcloud::stopHook()
{
    DepthImage2PointcloudBase::stopHook();
}
void DepthImage2Pointcloud::cleanupHook()
{
    DepthImage2PointcloudBase::cleanupHook();
}
