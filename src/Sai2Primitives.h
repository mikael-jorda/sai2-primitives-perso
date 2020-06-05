#include "tasks/JointTask.h"
#include "tasks/PositionTask.h"
#include "tasks/OrientationTask.h"
#include "tasks/PosOriTask.h"
//#include "tasks/TwoHandTwoRobotsTask.h"
#include "primitives/RedundantArmMotion.h"
#include "primitives/SurfaceSurfaceAlignment.h"

#include "haptic_tasks/OpenLoopTeleop.h"
#include "haptic_tasks/HapticController.h"
#include "haptic_tasks/BilateralPassivityController.h"
#include "haptic_tasks/ImpedanceControl.h"

#ifdef USING_OTG
#include "trajectory_generation/OTG.h"
#include "trajectory_generation/OTG_ori.h"
#include "trajectory_generation/OTG_posori.h"
#endif
