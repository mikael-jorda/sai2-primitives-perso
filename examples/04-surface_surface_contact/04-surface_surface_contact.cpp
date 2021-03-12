/*
 * Example of a controller for a Panda arm (7DoF robot) 
 * performing a surface-surface alignment (zero moment control) 
 * after reaching contact
 */

// Initialization is the same as previous examples
#include <iostream>
#include <string>
#include <thread>
#include <math.h>
#include <mutex>

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include "tasks/PosOriTask.h"
#include "tasks/JointTask.h"
#include "timer/LoopTimer.h"

#include "force_sensor/ForceSensorSim.h"      // for simulated force sensor
#include "force_sensor/ForceSensorDisplay.h"  // for display of forces as lines


#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

#include <signal.h>
bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;
using namespace Eigen;

const string world_file = "resources/world.urdf";
const string robot_file = "resources/panda_arm.urdf";
const string robot_name = "PANDA";
// need a second robot model for the plate
const string plate_file = "resources/plate.urdf";
const string plate_name = "Plate";

const string camera_name = "camera";

mutex m;

// global variables for sensed force and moment
Vector3d sensed_force;
Vector3d sensed_moment;

// global variables for controller parametrization
const string link_name = "end-effector";
const Vector3d pos_in_link = Vector3d(0.0,0.0,0.04);
const Vector3d sensor_pos_in_link = Vector3d(0.0,0.0,0.0);

// state machine for control
#define GO_TO_CONTACT    0
#define CONTACT_CONTROL  1

// simulation and control loop
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);
void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* plate, Simulation::Sai2Simulation* sim, ForceSensorSim* fsensor);

/*--------------- Functions and variables for graphic display handling ----------------*/
GLFWwindow* glfwInitialize();
void glfwError(int error, const char* description);
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);
void mouseClick(GLFWwindow* window, int button, int action, int mods);

bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;

//------------ main function
int main (int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_file << endl;

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, false);
	Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setCoeffFrictionStatic(0);

	// load robot and plate
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	sim->getJointPositions(robot_name, robot->_q);
	robot->updateModel();

	// load plate
	auto plate = new Sai2Model::Sai2Model(plate_file, false);

	// create simulated force sensor
	Affine3d T_sensor = Affine3d::Identity();
	T_sensor.translation() = sensor_pos_in_link;
	auto fsensor = new ForceSensorSim(robot_name, link_name, T_sensor, robot);
	auto fsensor_display = new ForceSensorDisplay(fsensor, graphics);
	fsensor->enableFilter(0.005);

	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();
	double last_cursorx, last_cursory;

    // set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// start the simulation thread first
	fSimulationRunning = true;
	thread sim_thread(simulation, robot, plate, sim, fsensor);

	// next start the control thread
	thread ctrl_thread(control, robot, sim);
	
    // while window is open:
    while (!glfwWindowShouldClose(window)) {
    	fsensor_display->update();
		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->updateGraphics(plate_name, plate);
		graphics->render(camera_name, width, height);
		glfwSwapBuffers(window);
		glFinish();

	    // poll for events
	    glfwPollEvents();

		// move scene camera as required
    	// graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
    	Vector3d cam_depth_axis;
    	cam_depth_axis = camera_lookat - camera_pos;
    	cam_depth_axis.normalize();
    	Vector3d cam_up_axis;
    	// cam_up_axis = camera_vertical;
    	// cam_up_axis.normalize();
    	cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
	    Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
    	cam_roll_axis.normalize();
    	Vector3d cam_lookat_axis = camera_lookat;
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
			Matrix3d m_tilt; m_tilt = AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Matrix3d m_pan; m_pan = AngleAxisd(compass, -cam_up_axis);
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

	// prepare state machine
	int state = GO_TO_CONTACT;
	
	// update robot model and initialize control vectors
	robot->updateModel();
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof,dof);

	// Position plus orientation task

	Sai2Primitives::PosOriTask* posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	VectorXd posori_task_torques = VectorXd::Zero(dof);
	// set the force snesor location for the contact part of the task
	posori_task->setForceSensorFrame(link_name, Affine3d::Identity());

#ifdef USING_OTG
	// disable the interpolation
	posori_task->_use_interpolation_flag = false;
#endif
	// no gains setting here, using the default task values
	Matrix3d initial_orientation;
	Vector3d initial_position;
	robot->rotation(initial_orientation, posori_task->_link_name);
	robot->position(initial_position, posori_task->_link_name, posori_task->_control_frame.translation());

	// joint task to control the redundancy
	Sai2Primitives::JointTask* joint_task = new Sai2Primitives::JointTask(robot);
	VectorXd joint_task_torques = VectorXd::Zero(dof);

	VectorXd initial_q = robot->_q;

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
		{
			lock_guard<mutex> lock(m);
			sim->getJointPositions(robot_name, robot->_q);
			sim->getJointVelocities(robot_name, robot->_dq);
		}
		robot->updateModel();

		// update force sensor values (needs to be the force applied by the robot to the environment, in sensor frame)
		posori_task->updateSensedForceAndMoment(-sensed_force, -sensed_moment);

		// update tasks model. Order is important to define the hierarchy
		N_prec = MatrixXd::Identity(dof,dof);

		posori_task->updateTaskModel(N_prec);
		N_prec = posori_task->_N;    
		// after each task, need to update the nullspace 
		// of the previous tasks in order to garantee 
		// the dyamic consistency

		joint_task->updateTaskModel(N_prec);

		// -------- set task goals in the state machine and compute control torques
		if(state == GO_TO_CONTACT) {
			posori_task->_desired_position(2) -= 0.00003;  // go down at 30 cm/s until contact is detected
		
			if(posori_task->_sensed_force(2) <= -1.0) {
				// switch the local z axis to be force controlled and the local x and y axis to be moment controlled
				Vector3d local_z = posori_task->_current_orientation.col(2);
				posori_task->setForceAxis(local_z);
				posori_task->setAngularMotionAxis(local_z);

				// posori_task->setClosedLoopForceControl();
				posori_task->setClosedLoopMomentControl();

				// set the force and moment control set points and gains gains
				posori_task->_desired_force = 5.0*local_z;
				posori_task->_desired_moment = Vector3d::Zero();

				posori_task->_kp_force = 0.7;
				posori_task->_ki_force = 1.3;
				posori_task->_kv_force = 3.0;

				posori_task->_kp_moment = 0.5;
				posori_task->_ki_moment = 1.5;
				posori_task->_kv_moment = 1.0;

				// change the state of the state machine
				state = CONTACT_CONTROL;
			}	
		}
		else if(state == CONTACT_CONTROL) {
				Vector3d local_z = posori_task->_current_orientation.col(2);
				posori_task->_desired_force = 10.0*local_z;
				posori_task->updateForceAxis(local_z);
				posori_task->updateAngularMotionAxis(local_z);			
		}

		// compute torques for the different tasks
		posori_task->computeTorques(posori_task_torques);
		joint_task->computeTorques(joint_task_torques);

		//------ compute the final torques
		command_torques = posori_task_torques + joint_task_torques;

		// send to simulation
		{
			lock_guard<mutex> lock(m);
			sim->setJointTorques(robot_name, command_torques);
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
void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* plate, Simulation::Sai2Simulation* sim, ForceSensorSim* fsensor)
{
	fSimulationRunning = true;

	// plate controller
	Vector2d plate_qd = Vector2d::Zero();
	Vector2d plate_torques = Vector2d::Zero();

	// create a timer
	double sim_freq = 8000;
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(sim_freq); 
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();
		double time = timer.elapsedTime();

		// force sensor update
		fsensor->update(sim);
		fsensor->getForceLocalFrame(sensed_force);
		fsensor->getMomentLocalFrame(sensed_moment);

		// plate controller
		sim->getJointPositions(plate_name, plate->_q);
		sim->getJointVelocities(plate_name, plate->_dq);
		plate->updateKinematics();

		plate_qd(0) = 5.0/180.0*M_PI*sin(2*M_PI*0.12*time);
		plate_qd(1) = 7.0/180.0*M_PI*sin(2*M_PI*0.08*time);

		plate_torques = -1000.0*(plate->_q - plate_qd) - 75.0*plate->_dq;

		sim->setJointTorques(plate_name, plate_torques);

		// integrate forward
		{
			lock_guard<mutex> lock(m);
			sim->integrate(1.0/sim_freq);
		}

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