#include "scene/3d/skeleton.h"

#include "modules/godot_math_extension/godot_math_extension.h"

class LegIK : public Object {

	OBJ_TYPE(LegIK, Object);

	static LegIK *singleton;
public:
	static void solve_leg_ik(Skeleton *p_skeleton, int p_start, int p_mid, int p_end, Vector3 p_target) {

		Transform skeleton_affine_inverse = p_skeleton->get_global_transform().affine_inverse();

		Transform start = p_skeleton->get_global_transform() * p_skeleton->get_bone_global_pose(p_start);
		Transform mid = p_skeleton->get_global_transform() * p_skeleton->get_bone_global_pose(p_mid);
		Transform end = p_skeleton->get_global_transform() * p_skeleton->get_bone_global_pose(p_end);

		Transform end_X = p_skeleton->get_bone_global_pose(p_end);

		Vector3 start_end_axis = Vector3(end.origin - start.origin).normalized();
		real_t start_end_dist = start.origin.distance_to(end.origin);

		Vector3 start_target_axis = Vector3(p_target - start.origin).normalized();
		real_t start_target_dist = start.origin.distance_to(p_target);

		real_t start_mid_dist = start.origin.distance_to(mid.origin);
		real_t mid_end_dist = mid.origin.distance_to(end.origin);

		real_t mid_angle = Math::rad2deg(acos((start_mid_dist * start_mid_dist + mid_end_dist * mid_end_dist - start_end_dist * start_end_dist) / (2 * start_mid_dist * mid_end_dist)));

		if (start_target_dist > (start_mid_dist + mid_end_dist))
			start_target_dist = start_mid_dist + mid_end_dist - 0.01;

		real_t new_mid_angle = Math::rad2deg(acos((start_mid_dist * start_mid_dist + mid_end_dist * mid_end_dist - start_target_dist * start_target_dist) / (2 * start_mid_dist * mid_end_dist)));

		real_t rotate_angle = (new_mid_angle - mid_angle) * 0.5;

		if (!Math::is_nan(rotate_angle)) {
			const static Vector3 start_foward = Vector3(1, 0, 0);
			const static Vector3 mid_forward = Vector3(1, 0, 0);

			Transform bone_transform = p_skeleton->get_bone_pose(p_start);
			bone_transform = GodotMathExtension::rotate_around(bone_transform, bone_transform.origin, start_foward, -1 * rotate_angle);
			p_skeleton->set_bone_pose(p_start, bone_transform);

			bone_transform = p_skeleton->get_bone_pose(p_mid);
			bone_transform = GodotMathExtension::rotate_around(bone_transform, bone_transform.origin, mid_forward, rotate_angle * 2);
			p_skeleton->set_bone_pose(p_mid, bone_transform);

			bone_transform = p_skeleton->get_bone_global_pose(p_end);
			bone_transform.basis = end_X.basis;
			p_skeleton->set_bone_global_pose(p_end, bone_transform);

		}
	}
};

class _LegIK : public Object {

	OBJ_TYPE(_LegIK, Object);

	static _LegIK *singleton;
protected:
	static void _bind_methods();

public:
	static _LegIK *get_singleton();
	void solve_leg_ik(Object *p_skeleton, int p_start, int p_mid, int p_end, Vector3 p_target);
	_LegIK();
};

static _LegIK *_leg_ik = NULL;