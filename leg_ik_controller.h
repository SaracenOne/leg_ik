#ifndef LEG_IK_CONTROLLER_H
#define LEG_IK_CONTROLLER_H

#include "scene/3d/spatial.h"
#include "scene/3d/skeleton.h"
#include "servers/physics_server.h"

class LegIKController : public Node {

	OBJ_TYPE( LegIKController, Node );
	OBJ_SAVE_TYPE( LegIKController );
protected:
	NodePath skeleton_path;

	bool ik_on = false;
	bool ik_valid = false;

	Transform root_global_transform;
	Set<RID> ray_exclusion_array;

	String root_bone_name;

	String pelvis_name;

	String left_leg_name;
	String left_knee_name;
	String left_ankle_name;
	String left_toe_name;

	String right_leg_name;
	String right_knee_name;
	String right_ankle_name;
	String right_toe_name;

	int root_bone = -1;
	int pelvis = -1;

	int left_leg = -1;
	int left_knee = -1;
	int left_ankle = -1;
	int left_toe = -1;

	int right_leg = -1;
	int right_knee = -1;
	int right_ankle = -1;
	int right_toe = -1;

	float left_thigh_length;
	float right_thigh_length;
	float left_calf_length;
	float right_calf_length;
	float left_leg_full_length;
	float right_leg_full_length;

	float animation_offset;
	float ankle_height_L;
	float ankle_height_R;
	float root_offset;

	float target_height;
	float prev_target_height;
	float target_height_velocity;

	bool left_foot_down;
	bool right_foot_down;

	Spatial *root;
	Skeleton *skeleton;
	PhysicsDirectSpaceState *dss;

	Vector3 targeted_left_ground;
	Vector3 targeted_right_ground;

	float ik_feet_reposition_rate;
	float ik_body_reposition_rate;

	Vector3 prev_left_ground;
	Vector3 prev_right_ground;

	static void _bind_methods();
public:

	LegIKController();
	~LegIKController();

	void global_leg_ik_enabled_changed();
	void global_leg_ik_feet_reposition_rate_changed();
	void global_leg_ik_body_reposition_rate_changed();
	bool is_ik_active();

	void _ready();
	void _notification(int p_what);

	void solve_leg_ik(const float p_delta);

	_FORCE_INLINE_ void set_skeleton_path(NodePath &p_skeleton_path) {
		skeleton_path = p_skeleton_path;
		if (has_node(skeleton_path)) {
			skeleton = static_cast<Skeleton *>(get_node(skeleton_path));
		} else {
			skeleton = NULL;
		}
	}
	
	_FORCE_INLINE_ NodePath &get_skeleton_path() {
		return skeleton_path;
	}

	_FORCE_INLINE_ void set_root_bone_name(String &p_root_bone_name) {
		root_bone_name = p_root_bone_name;
	}

	_FORCE_INLINE_ String &get_root_bone_name() {
		return root_bone_name;
	}

	_FORCE_INLINE_ void set_pelvis_name(String &p_pelvis_name) {
		pelvis_name = p_pelvis_name;
	}

	_FORCE_INLINE_ String &get_pelvis_name() {
		return pelvis_name;
	}

	// Left
	_FORCE_INLINE_ void set_left_leg_name(String &p_left_leg_name) {
		left_leg_name = p_left_leg_name;
	}

	_FORCE_INLINE_ String &get_left_leg_name() {
		return left_leg_name;
	}

	_FORCE_INLINE_ void set_left_knee_name(String &p_left_knee_name) {
		left_knee_name = p_left_knee_name;
	}

	_FORCE_INLINE_ String &get_left_knee_name() {
		return left_knee_name;
	}

	_FORCE_INLINE_ void set_left_ankle_name(String &p_left_ankle_name) {
		left_ankle_name = p_left_ankle_name;
	}

	_FORCE_INLINE_ String &get_left_ankle_name() {
		return left_ankle_name;
	}

	_FORCE_INLINE_ void set_left_toe_name(String &p_left_toe_name) {
		left_toe_name = p_left_toe_name;
	}

	_FORCE_INLINE_ String &get_left_toe_name() {
		return left_toe_name;
	}

	// Right
	_FORCE_INLINE_ void set_right_leg_name(String &p_right_leg_name) {
		right_leg_name = p_right_leg_name;
	}

	_FORCE_INLINE_ String &get_right_leg_name() {
		return right_leg_name;
	}

	_FORCE_INLINE_ void set_right_knee_name(String &p_right_knee_name) {
		right_knee_name = p_right_knee_name;
	}

	_FORCE_INLINE_ String &get_right_knee_name() {
		return right_knee_name;
	}

	_FORCE_INLINE_ void set_right_ankle_name(String &p_right_ankle_name) {
		right_ankle_name = p_right_ankle_name;
	}

	_FORCE_INLINE_ String &get_right_ankle_name() {
		return right_ankle_name;
	}

	_FORCE_INLINE_ void set_right_toe_name(String &p_right_toe_name) {
		right_toe_name = p_right_toe_name;
	}

	_FORCE_INLINE_ String &get_right_toe_name() {
		return right_toe_name;
	}
};


#endif // LEG_IK_CONTROLLER_H
