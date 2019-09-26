#include "tasks/JointTask.h"
#include "tasks/PositionTask.h"
#include "tasks/OrientationTask.h"
#include "tasks/PosOriTask.h"
#include "tasks/TwoHandTwoRobotsTask.h"

#ifdef USING_OTG
#include "trajectory_generation/OTG.h"
#include "trajectory_generation/OTG_ori.h"
#include "trajectory_generation/OTG_posori.h"
#endif
