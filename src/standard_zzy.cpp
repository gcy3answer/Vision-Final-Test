#include<iostream>
#include<fmt/format.h>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>
#include "io/camera.hpp"
#include "io/dm_imu/dm_imu.hpp"
#include "io/gimbal/gimbal.hpp"
#include "io/command.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tools/plotter.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/test2.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

    tools::Exiter exiter;
    tools::Plotter plotter;
    
    io::Camera camera(config_path);
    io::Gimbal gimbal(config_path);
    
    auto_aim::YOLO detector(config_path,false);
    auto_aim::Solver solver(config_path);
    auto_aim::Tracker tracker(config_path, solver);
    auto_aim::Aimer aimer(config_path);
    auto_aim::Shooter shooter(config_path);
    
    cv::Mat img;
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point t;
    auto mode = io::GimbalMode::IDLE;
    auto last_mode = io::GimbalMode::IDLE;

    while(!exiter.exit())
    {
        camera.read(img,t);
        q=gimbal.q(t);
        mode=gimbal.mode();
        if(mode!=last_mode) last_mode=mode;
        solver.set_R_gimbal2world(q);
        Eigen::Vector3d finalxyzw=tools::eulers(solver.R_gimbal2world(),2,1,0);
        auto armors=detector.detect(img);
        auto targets=tracker.track(armors,t);
        auto cmd=aimer.aim(targets,t,15);
        cmd.shoot=false;
        gimbal.send(cmd);

    }
    return 0;
}
