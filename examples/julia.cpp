#include <array>
#include <cmath>

#include <functional>
#include <future>
#include <iostream>
#include <string>
#include <sstream>
#include <thread>
#include <regex>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "examples_common.h"
#include "myLib.h"
#include "myJson.h"

#include <sys/types.h> // For pipe creation
#include <sys/stat.h>  // For pipe creation
#include <fcntl.h>     // For open
#include <unistd.h>    // For unlink
#include <fstream>

void pipe_state(std::ofstream &f, franka::RobotState robot_state)
{
    auto q = robot_state.q.data();
    auto dq = robot_state.dq.data();
    std::string msg("");
    for (int i = 0; i < 7; i++)
    {
        msg = msg + std::to_string(q[i]) + ",";
    }
    for (int i = 0; i < 6; i++)
    {
        msg = msg + std::to_string(dq[i]) + ",";
    }
    // No comma on final value
    msg = msg + std::to_string(dq[6]);
    // Terminate with endl
    f << msg << std::endl;
}

void parse_comma_separated_doubles(std::vector<double> &vect, std::stringstream& ss)
{
    while (ss.good())
    {
        std::string substr;
        getline(ss, substr, ',');
        vect.push_back(std::stod(substr));
    }
}

Eigen::VectorXd parse_torques(std::ifstream& infifo) {
        std::string line;
        std::getline(infifo, line);
        std::vector<double> torque_data;
        std::stringstream ss(line);
        
        parse_comma_separated_doubles(torque_data, ss);

        std::cout << torque_data.size() << std::endl;
        Eigen::VectorXd torques = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(torque_data.data(), torque_data.size());
        if (torques.size() != 7) {
            throw "Parsed torques vector not length 7!";
        }
        return torques;
}

int main(int argc, char **argv)
{
    // Check whether the required arguments were passed
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    }

    char *outfifoname = new char[15];
    strcpy(outfifoname, "./cxxjuliapipe");
    char *infifoname = new char[15];
    strcpy(infifoname, "./juliacxxpipe");

    /* create the FIFO (named pipe) */
    mkfifo(outfifoname, 0666);
    mkfifo(infifoname, 0666);
    printf("Opening pipes, waiting for other end to be opened in other program...\n");
    std::ofstream outfifo(outfifoname, std::ofstream::out);
    std::ifstream infifo(infifoname, std::ifstream::in);
    printf("FIFO pipes opened\n");

    try
    {
        // connect to robot
        franka::Robot robot(argv[1]);
        setDefaultBehavior(robot);
        // load the kinematics and dynamics model
        franka::Model model = robot.loadModel();
        franka::RobotState initial_state = robot.readOnce();

        pipe_state(outfifo, initial_state);
        auto torques = parse_torques(infifo);

        while (true) {
            std::cout << "Writing..." << std::endl;
            pipe_state(outfifo, initial_state);
            sleep(1);
        }

        // std::string line;
        // std::getline(infifo, line);
        // std::vector<double> torque_data;

        // printf("Line to parse\n");
        // std::cout << line << std::endl;
        // std::stringstream ss(line);
        // parse_comma_separated_doubles(torque_data, ss);

        // std::cout << torque_data.size() << std::endl;
        // Eigen::VectorXd torques = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(torque_data.data(), torque_data.size());

        //     // Register point
        //     Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        //     const Eigen::Vector3d ee_offset({0.377, 0.0, 0.042});
        //     const auto frame = franka::Frame::kEndEffector;
        //     Eigen::Vector3d starting_pos(initial_transform * ee_offset);

        //     WorldCoord ee;
        //     ee.offset = ee_offset;
        //     ee.ref = starting_pos;

        //     // define callback for the torque control loop
        //     std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
        //         impedance_control_callback = [&](const franka::RobotState &robot_state,
        //                                          franka::Duration duration) -> franka::Torques
        //     {
        //         ImpedanceCoordArgs iargs(robot_state, model, frame);
        //         auto ee_coord = computeWorldCoord(iargs, ee);
        //         if (ee_coord.z.norm() > 0.15)
        //         {
        //             throw std::runtime_error("Aborting; too far away from starting pose!");
        //         }

        //         // compute control
        //         Eigen::VectorXd tau_d(7);
        //         tau_d.fill(0);

        //         // convert to double array
        //         std::array<double, 7> tau_d_array{};$
        //         Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
        //         return tau_d_array;
        //     };

        //     // start real-time control loop
        //     std::cout << "WARNING: Collision thresholds are set to high values. "
        //               << "Make sure you have the user stop at hand!" << std::endl
        //               << "After starting try to push the robot and see how it reacts." << std::endl
        //               << "Press Enter to continue..." << std::endl;
        //     std::cin.ignore();
        //     robot.control(impedance_control_callback);
    }
    catch (const franka::Exception &ex)
    {
        // print exception
        std::cout << ex.what() << std::endl;
    }
    catch (const std::exception &ex)
    {
        std::cerr << ex.what() << std::endl;
    }

    outfifo.close();
    unlink(outfifoname);

    return 0;
}
