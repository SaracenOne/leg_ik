#include "register_types.h"

#include "globals.h"
#include "leg_ik.h"
#include "leg_ik_controller.h"

void register_leg_ik_types() {
	_leg_ik = memnew(_LegIK);
	Globals::get_singleton()->add_singleton(Globals::Singleton("LegIK", _LegIK::get_singleton()));

	ObjectTypeDB::register_type<LegIKController>();
}
void unregister_leg_ik_types() {
	memdelete(_leg_ik);
}
