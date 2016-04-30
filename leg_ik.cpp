#include "leg_ik.h"

_LegIK *_LegIK::singleton = NULL;

void _LegIK::_bind_methods() {
	ObjectTypeDB::bind_method(_MD("solve_leg_ik", "skeleton", "start", "mid", "end", "target"), &_LegIK::solve_leg_ik);
}

_LegIK *_LegIK::get_singleton() {
	return singleton;
}

void _LegIK::solve_leg_ik(Object *p_skeleton, int p_start, int p_mid, int p_end, Vector3 p_target) {
	if (p_skeleton) {
		Skeleton *skeleton = p_skeleton->cast_to<Skeleton>();
		if (skeleton)
			LegIK::solve_leg_ik(skeleton, p_start, p_mid, p_end, p_target);
	}
};

_LegIK::_LegIK() {
	singleton = this;
}