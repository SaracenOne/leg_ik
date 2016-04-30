#ifndef LEG_IK_CONTROLLER_H
#define LEG_IK_CONTROLLER_H

#include "scene/3d/spatial.h"

class LegIKController : public Node {

	OBJ_TYPE( LegIKController, Node );
	OBJ_SAVE_TYPE( LegIKController );
public:

	LegIKController();
	~LegIKController();
};


#endif // LEG_IK_CONTROLLER_H
