/*
 * Example of a controller for a Kuka arm made with the motion arm primitive for redundant arms
 *
 */

#include <iostream>
#include <string>
#include <thread>
#include <math.h>

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>

#include "Sai2Primitives.h"
#include <redis/RedisClient.h>
#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

#include <signal.h>
bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;
using namespace Eigen;

const string world_file = "resources/world.urdf";
const string robot_file = "resources/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA";

const string camera_name = "camera";

RedisClient redis_client;

// redis keys to change desired position and orientation
const string DESIRED_POSITION_KEY = "sai2::demo_kuka::desired_position";
const string KP_KEY = "sai2::demo_kuka::kp";
const string KV_KEY = "sai2::demo_kuka::kv";

// write eigen vector expanded into several key value pairs
// such that each redis value will contain a float
// e.g. if setting key: [val0, val1, val2], three key values are saved:
// <key>	<val>
// key..0	 val0
// key..1	 val1
// key..2	 val2
template<typename Derived>
void setEigenVectorDerivedExpanded(const std::string &cmd_mssg, const Eigen::MatrixBase<Derived> &set_mat) {
	if (set_mat.cols() != 1) {
		// only support vector
		throw std::runtime_error("RedisClient: Could not set expanded on matrix. must be vector.");
	}
	std::vector<std::pair<std::string, std::string>> keyvals;
	for (int i = 0; i < set_mat.size(); i++) {
		std::string key = cmd_mssg + ".." + std::to_string(i);
		std::stringstream ss;
		ss << set_mat(i);
		std::string val = ss.str();
		keyvals.push_back({key, val});
	}
	redis_client.mset(keyvals);
}

	// read eigen vector from several key value pairs
// such that returned value will assemble multiple floats into a vector
// e.g. if redis contains:
// <key>	<val>
// key..0	 val0
// key..1	 val1
// key..2	 val2
// the returned value is:
// [val0, val1, val2]
template<typename Derived>
void getEigenVectorDerivedExpanded(const std::string &cmd_mssg, Eigen::MatrixBase<Derived> &ret_mat) {
	if (ret_mat.cols() != 1) {
		// only support vector
		throw std::runtime_error("RedisClient: Could not get expanded on matrix. must be vector.");
	}
	std::vector<std::string> keys;
	for (int i = 0; i < ret_mat.size(); i++) {
		std::string key = cmd_mssg + ".." + std::to_string(i);
		keys.push_back(key);
	}
	std::vector<std::string> vals = redis_client.mget(keys);
	for (int i = 0; i < vals.size(); i++) {
		ret_mat(i) = std::stod(vals[i]);
	}
}

// simulation and control loop
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;

int main (int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, false);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);

	// load robots
	Eigen::Vector3d world_gravity = sim->_world->getGravity().eigen();
	auto robot = new Sai2Model::Sai2Model(robot_file, false, sim->getRobotBaseTransform(robot_name), world_gravity);

	sim->getJointPositions(robot_name, robot->_q);
	robot->updateModel();

	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

	double last_cursorx, last_cursory;

    // set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// start the simulation thread first
	fSimulationRunning = true;
	thread sim_thread(simulation, robot, sim);

	// next start the control thread
	thread ctrl_thread(control, robot, sim);
	
    // while window is open:
    while (!glfwWindowShouldClose(window)) {
		// update kinematic models
		// robot->updateModel();

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->render(camera_name, width, height);
		glfwSwapBuffers(window);
		glFinish();

	    // poll for events
	    glfwPollEvents();

		// move scene camera as required
    	// graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
    	Eigen::Vector3d cam_depth_axis;
    	cam_depth_axis = camera_lookat - camera_pos;
    	cam_depth_axis.normalize();
    	Eigen::Vector3d cam_up_axis;
    	// cam_up_axis = camera_vertical;
    	// cam_up_axis.normalize();
    	cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
	    Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
    	cam_roll_axis.normalize();
    	Eigen::Vector3d cam_lookat_axis = camera_lookat;
    	cam_lookat_axis.normalize();
    	if (fTransXp) {
	    	camera_pos = camera_pos + 0.05*cam_roll_axis;
	    	camera_lookat = camera_lookat + 0.05*cam_roll_axis;
	    }
	    if (fTransXn) {
	    	camera_pos = camera_pos - 0.05*cam_roll_axis;
	    	camera_lookat = camera_lookat - 0.05*cam_roll_axis;
	    }
	    if (fTransYp) {
	    	// camera_pos = camera_pos + 0.05*cam_lookat_axis;
	    	camera_pos = camera_pos + 0.05*cam_up_axis;
	    	camera_lookat = camera_lookat + 0.05*cam_up_axis;
	    }
	    if (fTransYn) {
	    	// camera_pos = camera_pos - 0.05*cam_lookat_axis;
	    	camera_pos = camera_pos - 0.05*cam_up_axis;
	    	camera_lookat = camera_lookat - 0.05*cam_up_axis;
	    }
	    if (fTransZp) {
	    	camera_pos = camera_pos + 0.1*cam_depth_axis;
	    	camera_lookat = camera_lookat + 0.1*cam_depth_axis;
	    }	    
	    if (fTransZn) {
	    	camera_pos = camera_pos - 0.1*cam_depth_axis;
	    	camera_lookat = camera_lookat - 0.1*cam_depth_axis;
	    }
	    if (fRotPanTilt) {
	    	// get current cursor position
	    	double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
	    }
	    graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
	    glfwGetCursorPos(window, &last_cursorx, &last_cursory);
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();
	ctrl_thread.join();

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {
	
	robot->updateModel();
	int dof = robot->dof();
	Eigen::VectorXd command_torques = Eigen::VectorXd::Zero(dof);
	Eigen::MatrixXd N_prec = Eigen::MatrixXd::Identity(dof,dof);

	string link_name = "link6";
	Eigen::Vector3d pos_in_link = Eigen::Vector3d(0.0,0.0,0.0);

	Vector3d kp = Vector3d(100,100,100);
	Vector3d kv = Vector3d(20,20,20);
	Matrix3d kp_mat = Matrix3d::Zero();
	Matrix3d kv_mat = Matrix3d::Zero();


	// Motion arm primitive
	auto pos_task = new Sai2Primitives::PositionTask(robot, link_name, pos_in_link);
	Eigen::VectorXd pos_task_torques = VectorXd::Zero(dof);
	Vector3d pos_task_force = Vector3d::Zero();

	setEigenVectorDerivedExpanded(KP_KEY, kp);
	setEigenVectorDerivedExpanded(KV_KEY, kv);
	setEigenVectorDerivedExpanded(DESIRED_POSITION_KEY, pos_task->_desired_position);

	Eigen::Matrix3d initial_orientation;
	Eigen::Vector3d initial_position;
	robot->rotation(initial_orientation, link_name);
	robot->position(initial_position, link_name, pos_in_link);

	auto ori_task = new Sai2Primitives::OrientationTask(robot, link_name, pos_in_link);
	Eigen::VectorXd ori_task_torques = VectorXd::Zero(dof);

	auto joint_task = new Sai2Primitives::JointTask(robot);
	VectorXd joint_task_torques = VectorXd::Zero(dof);

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	unsigned long long controller_counter = 0;

	while (fSimulationRunning) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// update time
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();

		// update tasks model
		N_prec = Eigen::MatrixXd::Identity(dof,dof);
		pos_task->updateTaskModel(N_prec);
		N_prec = pos_task->_N;
		ori_task->updateTaskModel(N_prec);
		N_prec = ori_task->_N;
		joint_task->updateTaskModel(N_prec);

		// -------------------------------------------
		////////////////////////////// Compute joint torques
		double time = controller_counter/control_freq;

		// torques
		// pos_task->computeTorques(pos_task_torques);
		// read from redis
		getEigenVectorDerivedExpanded(KP_KEY, kp);
		getEigenVectorDerivedExpanded(KV_KEY, kv);
		getEigenVectorDerivedExpanded(DESIRED_POSITION_KEY, pos_task->_desired_position);

		for(int i = 0; i<3 ; i++)
		{
			kp_mat(i,i) = kp(i);
			kv_mat(i,i) = kv(i);
		}

		pos_task->computeTorques(joint_task_torques);
		pos_task_force = pos_task->_Lambda * (-kp_mat * (pos_task->_current_position - pos_task->_desired_position) - kv_mat * pos_task->_current_velocity);
		pos_task_torques = pos_task->_projected_jacobian.transpose() * pos_task_force;

		
		// pos_task->computeTorques(joint_task_torques);
		ori_task->computeTorques(ori_task_torques);
		joint_task->computeTorques(joint_task_torques);

		//------ Final torques
		command_torques = pos_task_torques + ori_task_torques + joint_task_torques;
		// command_torques.setZero();

		// -------------------------------------------
		sim->setJointTorques(robot_name, command_torques);
		

		// -------------------------------------------
		if(controller_counter % 500 == 0)
		{
			// cout << time << endl;
			// cout << endl;
		}

		controller_counter++;

		// -------------------------------------------
		// update last time
		last_time = curr_time;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Control Loop run time  : " << end_time << " seconds\n";
    std::cout << "Control Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Control Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

}

//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {
	fSimulationRunning = true;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(2000); 
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// integrate forward
		sim->integrate(0.0005);

	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
    std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
}


//------------------------------------------------------------------------------
GLFWwindow* glfwInitialize() {
		/*------- Set up visualization -------*/
    // set up error callback
    glfwSetErrorCallback(glfwError);

    // initialize GLFW
    glfwInit();

    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);

    // information about computer screen and GLUT display window
	int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - CS327a HW2", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	return window;
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
    switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			glfwSetWindowShouldClose(window,GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			fTransXp = set;
			break;
		case GLFW_KEY_LEFT:
			fTransXn = set;
			break;
		case GLFW_KEY_UP:
			fTransYp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransYn = set;
			break;
		case GLFW_KEY_A:
			fTransZp = set;
			break;
		case GLFW_KEY_Z:
			fTransZn = set;
			break;
		default:
			break;
    }
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
		switch (button) {
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			// NOTE: the code below is recommended but doesn't work well
			// if (fRotPanTilt) {
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			// }
			break;
		// if right click: don't handle. this is for menu selection
		case GLFW_MOUSE_BUTTON_RIGHT:
			//TODO: menu
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}