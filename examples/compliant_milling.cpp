#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "joyInput.h"
#include "myLib.h"

#include "examples_common.h"

class Cart {
 public:
  double mass;
  double damping;
  double q;
  double dq;
  Eigen::Vector3d pos0;
  Eigen::Vector3d dir;
  double F_input;
  Cart(double m, double c, Eigen::Vector3d line_pos, Eigen::Vector3d line_dir) {
    mass = m;
    damping = c;
    q = 0;
    dq = 0;
    pos0 = line_pos;
    dir = line_dir;
    F_input = 0.0;
  }
  void update(Eigen::Vector3d F, double dt) {
    double u_F = F.dot(dir);
    double u_damping = -dq * damping;
    double ddq = (F_input + u_F + u_damping) / mass;
    q += dq * dt;
    dq += ddq * dt;
  }
  Eigen::Vector3d pos() {
    return pos0 + q*dir;
  }
};

const double F_MAX = 2.0;  // In N

int main(int argc, char** argv) {
  std::cout << "Instrument impedance control demo" << std::endl;
  // Check whether the required arguments were passed
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  // Geometric parameters
  const Eigen::Vector3d ee_offset({0.377 - 0.193, 0.0, 0.042});
  const auto frame = franka::Frame::kEndEffector;

  // Compliance parameters
  const double stiffness{2000.0};
  const double damping{20.0};
  DiagonalSpringDamper<3,7> ee_impedance{Eigen::Array3d::Constant(stiffness),
                                         Eigen::Array3d::Constant(damping)};

  const double joint_stiffness{0.0};
  const double joint_damping{0.5};
  DiagonalSpringDamper<7,7> joint_impedance{Eigen::Array<double,7,1>::Constant(joint_stiffness),
                                            Eigen::Array<double,7,1>::Constant(joint_damping)};
  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    franka::RobotState initial_state = robot.readOnce();

    // equilibrium point is the initial position
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    
    // Cart
    double cart_mass = 2.0;
    double cart_damping = 10.0;
    Eigen::Vector3d cart_initial_pos = initial_transform * ee_offset;
    Eigen::Vector3d cart_dir({1.0, 0.0, 0.0});
    Cart cart(cart_mass, cart_damping, cart_initial_pos, cart_dir);
    int loopCounter = 0;

    // setup ee
    WorldCoord ee;
    ee.offset = ee_offset;
    ee.ref = cart.pos();

    // WorldCoord cart_err;
    // cart_err.offset = ee_offset;
    // cart_err.ref = cart.pos();


    const double max_f{15.0};
    const double max_t{10.0};
    // set collision behavior
    robot.setCollisionBehavior(
        {{max_t, max_t, max_t, max_t, max_t, max_t, max_t}}, {{max_t, max_t, max_t, max_t, max_t, max_t, max_t}},
        {{max_f, max_f, max_f, max_f, max_f, max_f}}, {{max_f, max_f, max_f, max_f, max_f, max_f}});

    with_controller([&](SDL_Joystick* controller) {
      // Define input functions
      TwoTriggerReader triggers(controller, 2, 5, AX_MAX, F_MAX);

      // define callback for the torque control loop
      std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
          impedance_control_callback = [&](const franka::RobotState& robot_state,
                                           franka::Duration duration) -> franka::Torques {
        // Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        ImpedanceCoordArgs iargs(robot_state, model, frame);

        // compute control coordinate error and jacobian
        ee.ref = cart.pos();
        // cart_err.ref = cart.pos();

        // std::cout << "q: " << cart.q << ", \tpos: " << cart.pos()[1] << " " << cart.pos()[2] << " " << cart.pos()[3] << std::endl;
        auto ee_coord = computeWorldCoord(iargs, ee);
        // auto cart_coord = computeWorldCoord(iargs, );
	      auto joint_coord = computeJointCoord(iargs);
        // Check error not too large
        if (ee_coord.z.norm() > 0.1) {
          throw std::runtime_error("Aborting; too far away from starting pose!");
        }

        // compute control
        Eigen::VectorXd tau_d(7);
        Eigen::Vector3d F_ee = ee_impedance.F(ee_coord);
        tau_d << ee_coord.J.transpose()*F_ee + joint_impedance.tau(joint_coord); 

        // Update cart 
        loopCounter = (loopCounter + 1) % 20;
        if (loopCounter == 0) {
          // Every 20 counts, read controller
          poll_SDL_events();
          cart.F_input = triggers.read();
        }
        cart.update(-F_ee, duration.toSec());

        // convert to double array
        std::array<double, 7> tau_d_array{};
        Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
        return tau_d_array;
      };

      // start real-time control loop
      std::cout << "WARNING: Collision thresholds are set to high values. "
                << "Make sure you have the user stop at hand!" << std::endl
                << "After starting try to push the robot and see how it reacts." << std::endl
                << "Press Enter to continue..." << std::endl;
      std::cin.ignore();
      robot.control(impedance_control_callback, true, 1000);
    });
  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  return 0;
}
