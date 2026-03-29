#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "fmt/format.h"

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/gimbal/gimbal.hpp"

#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"

#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

const std::string keys = 
    "{help h usage ? | }"
    "{@config-path   | configs/test2.yaml}";

int main(int argc, char * argv[]) {
    fmt::print("Hello from standard_hjy inside Dev Container!\n");
    
    cv::CommandLineParser cli(argc, argv, keys);
    auto config_path = cli.get<std::string>(0);
    if (cli.has("help") || config_path.empty()) {
        cli.printMessage();
        return 0;
    }
    
    tools::Exiter exiter;
    tools::Plotter plotter;
    tools::Recorder recorder;
    
    io::CBoard cboard(config_path);
    io::Camera camera(config_path);
    io::Gimbal gimbal(config_path);
    
    auto_aim::YOLO detector(config_path, false);
    auto_aim::Solver solver(config_path);
    auto_aim::Tracker tracker(config_path, solver);
    auto_aim::Aimer aimer(config_path);
    auto_aim::Shooter shooter(config_path);
    
    cv::Mat img;
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point t;
    
    auto mode = io::Mode::idle;
    auto last_mode = io::Mode::idle;
    
    while(!exiter.exit()) {
        camera.read(img, t);
        q = cboard.imu_at(t);
        mode = cboard.mode;
        if (mode != last_mode) last_mode = mode;
        
        solver.set_R_gimbal2world(q);
        
        Eigen::Vector3d finalxyzw = tools::eulers(solver.R_gimbal2world(), 0, 1, 2);
        
        auto armors = detector.detect(img);
        auto targets = tracker.track(armors, t);
        auto cmd = aimer.aim(targets, t, cboard.bullet_speed);
        gimbal.send(cmd);
    }    
    return 0;
}
