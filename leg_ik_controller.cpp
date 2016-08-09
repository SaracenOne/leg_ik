#include "leg_ik_controller.h"
#include "leg_ik.h"
#include "message_queue.h"
#include "io/marshalls.h"
#include "scene/scene_string_names.h"
#include "scene/3d/collision_object.h"
#include "core/globals.h"

LegIKController::LegIKController() {
	skeleton_path = NodePath();

	bool ik_on = false;
	bool ik_valid = false;

	root_global_transform = Transform();
	ray_exclusion_array = Set<RID>();

	root_bone_name = "root";
	pelvis_name = "lower_body";

	left_leg_name = "leg.L";
	left_knee_name = "knee.L";
	left_ankle_name = "ankle.L";
	left_toe_name = "toe.L";

	right_leg_name = "leg.R";
	right_knee_name = "knee.R";
	right_ankle_name = "ankle.R";
	right_toe_name = "toe.R";

	root_bone = -1;
	pelvis = -1;

	left_leg = -1;
	left_knee = -1;
	left_ankle = -1;
	left_toe = -1;

	right_leg = -1;
	right_knee = -1;
	right_ankle = -1;
	right_toe = -1;

	left_thigh_length = 0.0f;
	right_thigh_length = 0.0f;
	left_calf_length = 0.0f;
	right_calf_length = 0.0f;
	left_leg_full_length = 0.0f;
	right_leg_full_length = 0.0f;

	animation_offset = 0.01f;
	ankle_height_L = 0.0f;
	ankle_height_R = 0.0f;
	root_offset = 0.0f;

	target_height = 0.0f;
	prev_target_height = 0.0f;
	target_height_velocity = 0.0f;
		
	left_foot_down = false;
	right_foot_down = false;

	root = NULL;
	skeleton = NULL;
	dss = NULL;

	targeted_left_ground = Vector3();
	targeted_right_ground = Vector3();

	ik_feet_reposition_rate = 15.0f;
	ik_body_reposition_rate = 1.0f;

	prev_left_ground = Vector3();
	prev_right_ground = Vector3();
}


LegIKController::~LegIKController() {

}

void LegIKController::global_leg_ik_enabled_changed() {
	if (Globals::get_singleton()->has("gameplay/leg_ik_enabled")) {
		ik_on = Globals::get_singleton()->get("gameplay/leg_ik_enabled");
	} else {
		ik_on = false;
		Globals::get_singleton()->set("gameplay/leg_ik_enabled", ik_on);
	}
}

void LegIKController::global_leg_ik_feet_reposition_rate_changed() {
	if (Globals::get_singleton()->has("gameplay/leg_ik_feet_reposition_rate")) {
		ik_feet_reposition_rate = Globals::get_singleton()->get("gameplay/leg_ik_feet_reposition_rate");
	} else {
		Globals::get_singleton()->set("gameplay/leg_ik_feet_reposition_rate", ik_feet_reposition_rate);
	}
}

void LegIKController::global_leg_ik_body_reposition_rate_changed() {
	if (Globals::get_singleton()->has("gameplay/leg_ik_body_reposition_rate")) {
		ik_body_reposition_rate = Globals::get_singleton()->get("gameplay/leg_ik_body_reposition_rate");
	} else {
		Globals::get_singleton()->set("gameplay/leg_ik_body_reposition_rate", ik_body_reposition_rate);
	}
}

bool LegIKController::is_ik_active() {
	if (ik_on == true && ik_valid == true) {
		return true;
	} else {
		return false;
	}
}

void LegIKController::_ready() {
	global_leg_ik_enabled_changed();
	global_leg_ik_feet_reposition_rate_changed();
	global_leg_ik_body_reposition_rate_changed();

	//Ignore player collision hull
	ray_exclusion_array = Set<RID>();

	CollisionObject *collision_object = static_cast<CollisionObject *>(get_node(NodePath("..")));
	if (collision_object)
		ray_exclusion_array.insert(collision_object->get_rid());

	skeleton = static_cast<Skeleton *>(get_node(skeleton_path));

	if (skeleton != NULL) {
		root_bone = skeleton->find_bone(root_bone_name);
		pelvis = skeleton->find_bone(pelvis_name);

		left_leg = skeleton->find_bone(left_leg_name);
		left_knee = skeleton->find_bone(left_knee_name);
		left_ankle = skeleton->find_bone(left_ankle_name);
		left_toe = skeleton->find_bone(left_toe_name);

		right_leg = skeleton->find_bone(right_leg_name);
		right_knee = skeleton->find_bone(right_knee_name);
		right_ankle = skeleton->find_bone(right_ankle_name);
		right_toe = skeleton->find_bone(right_toe_name);

		if (root_bone != -1 && pelvis != -1 &&
			left_leg != -1 && left_knee != -1 && left_ankle != -1 &&
			right_leg != -1 && right_knee != -1 && right_ankle != -1 &&
			left_toe != -1 && right_toe != -1) {

			left_thigh_length = skeleton->get_bone_global_pose(left_leg).origin.distance_to(skeleton->get_bone_global_pose(left_knee).origin);
			left_calf_length = skeleton->get_bone_global_pose(left_knee).origin.distance_to(skeleton->get_bone_global_pose(left_ankle).origin);
			left_leg_full_length = left_thigh_length + left_calf_length;

			right_thigh_length = skeleton->get_bone_global_pose(right_leg).origin.distance_to(skeleton->get_bone_global_pose(right_knee).origin);
			right_calf_length = skeleton->get_bone_global_pose(right_knee).origin.distance_to(skeleton->get_bone_global_pose(right_ankle).origin);
			right_leg_full_length = right_thigh_length + right_calf_length;

			prev_target_height = skeleton->get_bone_global_pose(root_bone).origin.y;

			Transform left_foot_local_transform = skeleton->get_bone_global_pose(left_ankle);
			Transform right_foot_local_transform = skeleton->get_bone_global_pose(right_ankle);

			Transform left_foot_transform = skeleton->get_global_transform() * left_foot_local_transform;
			Transform right_foot_transform = skeleton->get_global_transform() * right_foot_local_transform;

			targeted_left_ground = left_foot_transform.origin;
			targeted_right_ground = right_foot_transform.origin;

			prev_left_ground = left_foot_local_transform.origin;
			prev_right_ground = right_foot_local_transform.origin;

			ik_valid = true;
		}
		else {
			ik_valid = false;
		}
	}
}

void LegIKController::_notification(int p_what) {

	switch(p_what) {
		case NOTIFICATION_READY: {
			_ready();
		} break;
		case NOTIFICATION_ENTER_TREE: {
			add_to_group("ik_group");
		} break;
		case NOTIFICATION_EXIT_TREE: {
			remove_from_group("ik_group");
		} break;
	}
}

void LegIKController::_bind_methods() {

	ObjectTypeDB::bind_method(_MD("global_leg_ik_enabled_changed"), &LegIKController::global_leg_ik_enabled_changed);
	ObjectTypeDB::bind_method(_MD("global_leg_ik_feet_reposition_rate_changed"), &LegIKController::global_leg_ik_feet_reposition_rate_changed);
	ObjectTypeDB::bind_method(_MD("global_leg_ik_body_reposition_rate_changed"), &LegIKController::global_leg_ik_body_reposition_rate_changed);
	ObjectTypeDB::bind_method(_MD("is_ik_active"), &LegIKController::is_ik_active);

	ObjectTypeDB::bind_method(_MD("solve_leg_ik", "delta"), &LegIKController::solve_leg_ik);

	ObjectTypeDB::bind_method(_MD("set_skeleton_path", "skeleton_path"), &LegIKController::set_skeleton_path);
	ObjectTypeDB::bind_method(_MD("get_skeleton_path"), &LegIKController::get_skeleton_path);

	ObjectTypeDB::bind_method(_MD("set_root_bone_name", "name"), &LegIKController::set_root_bone_name);
	ObjectTypeDB::bind_method(_MD("get_root_bone_name"), &LegIKController::get_root_bone_name);
	ObjectTypeDB::bind_method(_MD("set_pelvis_name", "name"), &LegIKController::set_pelvis_name);
	ObjectTypeDB::bind_method(_MD("get_pelvis_name"), &LegIKController::get_pelvis_name);

	ObjectTypeDB::bind_method(_MD("set_left_leg_name", "name"), &LegIKController::set_left_leg_name);
	ObjectTypeDB::bind_method(_MD("get_left_leg_name"), &LegIKController::get_left_leg_name);
	ObjectTypeDB::bind_method(_MD("set_left_knee_name", "name"), &LegIKController::set_left_knee_name);
	ObjectTypeDB::bind_method(_MD("get_left_knee_name"), &LegIKController::get_left_knee_name);
	ObjectTypeDB::bind_method(_MD("set_left_ankle_name", "name"), &LegIKController::set_left_ankle_name);
	ObjectTypeDB::bind_method(_MD("get_left_ankle_name"), &LegIKController::get_left_ankle_name);
	ObjectTypeDB::bind_method(_MD("set_left_toe_name", "name"), &LegIKController::set_left_toe_name);
	ObjectTypeDB::bind_method(_MD("get_left_toe_name"), &LegIKController::get_left_toe_name);

	ObjectTypeDB::bind_method(_MD("set_right_leg_name", "name"), &LegIKController::set_right_leg_name);
	ObjectTypeDB::bind_method(_MD("get_right_leg_name"), &LegIKController::get_right_leg_name);
	ObjectTypeDB::bind_method(_MD("set_right_knee_name", "name"), &LegIKController::set_right_knee_name);
	ObjectTypeDB::bind_method(_MD("get_right_knee_name"), &LegIKController::get_right_knee_name);
	ObjectTypeDB::bind_method(_MD("set_right_ankle_name", "name"), &LegIKController::set_right_ankle_name);
	ObjectTypeDB::bind_method(_MD("get_right_ankle_name"), &LegIKController::get_right_ankle_name);
	ObjectTypeDB::bind_method(_MD("set_right_toe_name", "name"), &LegIKController::set_right_toe_name);
	ObjectTypeDB::bind_method(_MD("get_right_toe_name"), &LegIKController::get_right_toe_name);

	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "skeleton_path", PROPERTY_HINT_RESOURCE_TYPE, "path"), _SCS("set_skeleton_path"), _SCS("get_skeleton_path"));

	//
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "root_bone_name", PROPERTY_HINT_NONE, "name"), _SCS("set_root_bone_name"), _SCS("get_root_bone_name"));
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "pelvis_name", PROPERTY_HINT_NONE, "name"), _SCS("set_pelvis_name"), _SCS("get_pelvis_name"));

	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "left_leg_name", PROPERTY_HINT_NONE, "name"), _SCS("set_left_leg_name"), _SCS("get_left_leg_name"));
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "left_knee_name", PROPERTY_HINT_NONE, "name"), _SCS("set_left_knee_name"), _SCS("get_left_knee_name"));
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "left_ankle_name", PROPERTY_HINT_NONE, "name"), _SCS("set_left_ankle_name"), _SCS("get_left_ankle_name"));
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "left_toe_name", PROPERTY_HINT_NONE, "name"), _SCS("set_left_toe_name"), _SCS("get_left_toe_name"));

	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "right_leg_name", PROPERTY_HINT_NONE, "name"), _SCS("set_right_leg_name"), _SCS("get_right_leg_name"));
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "right_knee_name", PROPERTY_HINT_NONE, "name"), _SCS("set_right_knee_name"), _SCS("get_right_knee_name"));
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "right_ankle_name", PROPERTY_HINT_NONE, "name"), _SCS("set_right_ankle_name"), _SCS("get_right_ankle_name"));
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "right_toe_name", PROPERTY_HINT_NONE, "name"), _SCS("set_right_toe_name"), _SCS("get_right_toe_name"));
}

void LegIKController::solve_leg_ik(const float p_delta) {
	
	if(skeleton == NULL)
		return;
	
	Transform root_bone_transform = skeleton->get_global_transform() * skeleton->get_bone_global_pose(root_bone);
	
	if (ik_on && ik_valid) {
		if(dss == NULL) {
			if (root == NULL) {
				root = static_cast<Spatial *>(get_node(NodePath("..")));
				if (root == NULL)
					return;
			}

			dss = PhysicsServer::get_singleton()->space_get_direct_state(root->get_world()->get_space());
			if(dss == NULL);
				return;
		};
				
		root_global_transform = root->get_global_transform();
				
		Transform skeleton_affine_inverse = skeleton->get_global_transform().affine_inverse();

		Transform left_foot_local_transform = skeleton->get_bone_global_pose(left_ankle);
		Transform right_foot_local_transform = skeleton->get_bone_global_pose(right_ankle);
		
		Transform left_foot_transform = skeleton->get_global_transform() * left_foot_local_transform;
		Transform right_foot_transform = skeleton->get_global_transform() * right_foot_local_transform;
		
		Transform left_calf_transform = skeleton->get_global_transform() * skeleton->get_bone_global_pose(left_knee);
		Transform right_calf_transform = skeleton->get_global_transform() * skeleton->get_bone_global_pose(right_knee);
		
		Transform left_thigh_transform = skeleton->get_global_transform() * skeleton->get_bone_global_pose(left_leg);
		Transform right_thigh_transform = skeleton->get_global_transform() * skeleton->get_bone_global_pose(right_leg);
		
		root_offset = root_bone_transform.origin.y - root_global_transform.origin.y;
		ankle_height_L = left_foot_transform.origin.y - root_global_transform.origin.y;
		ankle_height_R = right_foot_transform.origin.y - root_global_transform.origin.y;
		
		targeted_left_ground.y += (left_foot_local_transform.origin.y - prev_left_ground.y);
		targeted_right_ground.y += (right_foot_local_transform.origin.y - prev_right_ground.y);
					
		prev_left_ground = left_foot_local_transform.origin;
		prev_right_ground = right_foot_local_transform.origin;
		
		// Default feet targets.
		Vector3 left_ground = left_foot_transform.origin;
		Vector3 right_ground = right_foot_transform.origin;
		Vector3 original_left_ground = left_foot_transform.origin;
		Vector3 original_right_ground = right_foot_transform.origin;
		
		float animation_delta = left_ground.y - right_ground.y;
		if (animation_delta < -animation_offset) {
			left_foot_down = true;
			right_foot_down = false;
		} else if (animation_delta > animation_offset) {
			left_foot_down = false;
			right_foot_down = true;
		} else {
			left_foot_down = true;
			right_foot_down = true;
		}
		
		float ray_distance = 0.0;
	 
		if ((!left_foot_down) && (right_foot_down)) {
			ray_distance = left_calf_length + ankle_height_L;
		} else {
			ray_distance = left_calf_length + left_leg_full_length;
		}

		// Left
		if (ray_distance > 0) {
			Vector3 from = left_foot_transform.origin + Vector3(0, left_calf_length, 0);
			Vector3 to = from + (-Vector3(0, 1, 0) * ray_distance);
			
			PhysicsDirectSpaceState::RayResult result_foot;
			if (dss->intersect_ray(from, to, result_foot, ray_exclusion_array, 1, 15)) {
				Transform left_toe_transform = skeleton->get_global_transform() * skeleton->get_bone_global_pose(left_toe);
				
				float original_y = from.y;
				from = left_toe_transform.origin + Vector3(0, left_calf_length, 0);
				from.y = original_y;
				to = from + (-Vector3(0, 1, 0) * ray_distance);
				
				PhysicsDirectSpaceState::RayResult result_toe;
				if (dss->intersect_ray(from, to, result_toe, ray_exclusion_array, 1, 15)) {
					if (result_toe.position.y - result_foot.position.y > 0.001) {
						left_ground = result_toe.position + Vector3(0, ankle_height_L, 0);
					} else {
						left_ground = result_foot.position + Vector3(0, ankle_height_L, 0);
					}
				} else {
					left_ground = result_foot.position + Vector3(0, ankle_height_L, 0);
				}
			}
		}

		if ((left_foot_down) && (!right_foot_down)) {
			ray_distance = right_calf_length + ankle_height_R;
		} else {
			ray_distance = right_calf_length + right_leg_full_length;
		}
		
		// Right
		if (ray_distance > 0) {
			Vector3 from = right_foot_transform.origin + Vector3(0, right_calf_length, 0);
			Vector3 to = from + (-Vector3(0, 1, 0) * ray_distance);
			
			PhysicsDirectSpaceState::RayResult result_foot;
			if (dss->intersect_ray(from, to, result_foot, ray_exclusion_array, 1, 15)) {
				Transform right_toe_transform = skeleton->get_global_transform() * skeleton->get_bone_global_pose(right_toe);

				float original_y = from.y;
				from = right_toe_transform.origin + Vector3(0, right_calf_length, 0);
				from.y = original_y;
				to = from + (-Vector3(0, 1, 0) * ray_distance);

				PhysicsDirectSpaceState::RayResult result_toe;
				if (dss->intersect_ray(from, to, result_toe, ray_exclusion_array, 1, 15)) {
					if (result_toe.position.y - result_foot.position.y > 0.001) {
						right_ground = result_toe.position + Vector3(0, ankle_height_R, 0);
					} else {
						right_ground = result_foot.position + Vector3(0, ankle_height_R, 0);
					}
				} else {
					right_ground = result_foot.position + Vector3(0, ankle_height_R, 0);
				}
			}
		}

		if (left_ground.y < right_ground.y) {
			target_height = left_ground.y - ankle_height_L + root_offset;
		} else {
			target_height = right_ground.y - ankle_height_R + root_offset;
		}
		
		target_height = Math::lerp(prev_target_height, target_height, p_delta * ik_body_reposition_rate);
		
		float height_diff = target_height - root_bone_transform.origin.y;
		
		root_bone_transform.origin.y = target_height;
		skeleton->set_bone_global_pose(root_bone, skeleton_affine_inverse * root_bone_transform);
		
		prev_target_height = root_bone_transform.origin.y;

		if((!left_foot_down) && (right_foot_down) && (left_ground == original_left_ground))
			left_ground.y += height_diff;
		
		if((left_foot_down) && (!right_foot_down) && (right_ground == original_right_ground))
			right_ground.y += height_diff;
		
		targeted_left_ground.x = left_ground.x;
		targeted_left_ground.z = left_ground.z;
		
		targeted_right_ground.x = right_ground.x;
		targeted_right_ground.z = right_ground.z;
		
		targeted_left_ground.y = Math::lerp(targeted_left_ground.y, left_ground.y, p_delta * ik_feet_reposition_rate);
		targeted_right_ground.y = Math::lerp(targeted_right_ground.y, right_ground.y, p_delta * ik_feet_reposition_rate);

		LegIK::solve_leg_ik(skeleton, left_leg, left_knee, left_ankle, targeted_left_ground);
		LegIK::solve_leg_ik(skeleton, right_leg, right_knee, right_ankle, targeted_right_ground);
	} else {
		prev_target_height = root_bone_transform.origin.y;
	}
}