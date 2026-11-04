#include "direct_static_body.h"
#include "../profiling.h"
#include "classes/array_mesh.h"
#include "classes/physics_server_3d.h"
#include "classes/world_3d.h"

namespace zylann::godot {

DirectStaticBody::DirectStaticBody() {
	// Nothing here. It is a thin RID wrapper,
	// no calls to PhysicsServer3D are made until we called one of the functions.
}

DirectStaticBody::~DirectStaticBody() {
	destroy();
}

void DirectStaticBody::create() {
	ERR_FAIL_COND(_body.is_valid());
	PhysicsServer3D &ps = *PhysicsServer3D::get_singleton();
	_body = ps.body_create();
	ps.body_set_ray_pickable(_body, false);
	ps.body_set_mode(_body, PhysicsServer3DEnums::BODY_MODE_STATIC);
	if (_twin_space.is_valid()) {
		make_twin();
	}
}

void DirectStaticBody::make_twin() {
	PhysicsServer3D &ps = *PhysicsServer3D::get_singleton();
	_twin = ps.body_create();
	ps.body_set_ray_pickable(_twin, false);
	ps.body_set_mode(_twin, PhysicsServer3DEnums::BODY_MODE_STATIC);
	ps.body_attach_object_instance_id(_twin, ps.body_get_object_instance_id(_body));
	ps.body_set_collision_layer(_twin, ps.body_get_collision_layer(_body));
	ps.body_set_collision_mask(_twin, ps.body_get_collision_mask(_body));
	ps.body_set_state(
			_twin,
			PhysicsServer3DEnums::BODY_STATE_TRANSFORM,
			ps.body_get_state(_body, PhysicsServer3DEnums::BODY_STATE_TRANSFORM)
	);
	for (int i = 0; i < ps.body_get_shape_count(_body); ++i) {
		ps.body_add_shape(
				_twin, ps.body_get_shape(_body, i), ps.body_get_shape_transform(_body, i), _shape_disabled
		);
	}
	ps.body_set_space(_twin, ps.body_get_space(_body).is_valid() ? _twin_space : RID());
}

void DirectStaticBody::set_twin_space(RID space) {
	if (_twin_space == space) {
		return;
	}
	_twin_space = space;
	if (!_body.is_valid()) {
		return;
	}
	if (_twin.is_valid()) {
		free_physics_server_rid(*PhysicsServer3D::get_singleton(), _twin);
		_twin = RID();
	}
	if (space.is_valid()) {
		make_twin();
	}
}

void DirectStaticBody::destroy() {
	if (_body.is_valid()) {
		PhysicsServer3D &ps = *PhysicsServer3D::get_singleton();
		if (_twin.is_valid()) {
			free_physics_server_rid(ps, _twin);
			_twin = RID();
		}
		free_physics_server_rid(ps, _body);
		_body = RID();
		// The shape need to be destroyed after the body
		_shape.unref();
		_shape_disabled = false;
	}
	if (_debug_mesh_instance.is_valid()) {
		_debug_mesh_instance.destroy();
	}
}

bool DirectStaticBody::is_valid() const {
	return _body.is_valid();
}

void DirectStaticBody::set_transform(Transform3D transform) {
	ZN_PROFILE_SCOPE();
	ERR_FAIL_COND(!_body.is_valid());
	PhysicsServer3D::get_singleton()->body_set_state(_body, PhysicsServer3DEnums::BODY_STATE_TRANSFORM, transform);
	if (_twin.is_valid()) {
		PhysicsServer3D::get_singleton()->body_set_state(_twin, PhysicsServer3DEnums::BODY_STATE_TRANSFORM, transform);
	}

	if (_debug_mesh_instance.is_valid()) {
		_debug_mesh_instance.set_transform(transform);
	}
}

void DirectStaticBody::add_shape(Ref<Shape3D> shape) {
	ZN_PROFILE_SCOPE();
	ERR_FAIL_COND(!_body.is_valid());
	PhysicsServer3D::get_singleton()->body_add_shape(_body, shape->get_rid(), Transform3D(), false);
	if (_twin.is_valid()) {
		PhysicsServer3D::get_singleton()->body_add_shape(_twin, shape->get_rid(), Transform3D(), false);
	}
	// No use case for multishape yet
	_shape = shape;

	if (_debug_mesh_instance.is_valid()) {
		Ref<Mesh> mesh = _shape->get_debug_mesh();
		_debug_mesh_instance.set_mesh(mesh);
	}
}

void DirectStaticBody::remove_shape(int shape_index) {
	ERR_FAIL_COND(!_body.is_valid());
	PhysicsServer3D::get_singleton()->body_remove_shape(_body, shape_index);
	if (_twin.is_valid()) {
		PhysicsServer3D::get_singleton()->body_remove_shape(_twin, shape_index);
	}
	_shape.unref();
	_shape_disabled = false;

	if (_debug_mesh_instance.is_valid()) {
		_debug_mesh_instance.set_mesh(Ref<Mesh>());
	}
}

Ref<Shape3D> DirectStaticBody::get_shape(int shape_index) const {
	ERR_FAIL_COND_V(shape_index < 0 || shape_index > 1, Ref<Shape3D>());
	return _shape;
}

void DirectStaticBody::set_world(World3D *world) {
	ERR_FAIL_COND(!_body.is_valid());
	PhysicsServer3D &ps = *PhysicsServer3D::get_singleton();
	ps.body_set_space(_body, world != nullptr ? world->get_space() : RID());
	if (_twin.is_valid()) {
		ps.body_set_space(_twin, world != nullptr ? _twin_space : RID());
	}

	if (_debug_mesh_instance.is_valid()) {
		_debug_mesh_instance.set_world(world);
	}
}

void DirectStaticBody::set_shape_enabled(int shape_index, bool enabled) {
	ERR_FAIL_COND(!_body.is_valid());
	PhysicsServer3D &ps = *PhysicsServer3D::get_singleton();
	ps.body_set_shape_disabled(_body, shape_index, !enabled);
	_shape_disabled = !enabled;
	if (_twin.is_valid()) {
		ps.body_set_shape_disabled(_twin, shape_index, !enabled);
	}

	if (_debug_mesh_instance.is_valid()) {
		_debug_mesh_instance.set_visible(enabled);
	}
}

void DirectStaticBody::set_attached_object(const Object *obj) {
	// Serves in high-level collision query results, `collider` will contain the attached object
	ERR_FAIL_COND(!_body.is_valid());
	PhysicsServer3D::get_singleton()->body_attach_object_instance_id(
			_body, obj != nullptr ? obj->get_instance_id() : ObjectID()
	);
	if (_twin.is_valid()) {
		PhysicsServer3D::get_singleton()->body_attach_object_instance_id(
				_twin, obj != nullptr ? obj->get_instance_id() : ObjectID()
		);
	}
}

void DirectStaticBody::set_collision_layer(int layer) {
	ERR_FAIL_COND(!_body.is_valid());
	PhysicsServer3D::get_singleton()->body_set_collision_layer(_body, layer);
	if (_twin.is_valid()) {
		PhysicsServer3D::get_singleton()->body_set_collision_layer(_twin, layer);
	}
}

void DirectStaticBody::set_collision_mask(int mask) {
	ERR_FAIL_COND(!_body.is_valid());
	PhysicsServer3D::get_singleton()->body_set_collision_mask(_body, mask);
	if (_twin.is_valid()) {
		PhysicsServer3D::get_singleton()->body_set_collision_mask(_twin, mask);
	}
}

void DirectStaticBody::set_debug(bool enabled, World3D *world) {
	ERR_FAIL_COND(world == nullptr);

	if (enabled && !_debug_mesh_instance.is_valid()) {
		_debug_mesh_instance.create();
		_debug_mesh_instance.set_interpolated(false);
		_debug_mesh_instance.set_world(world);

		const Transform3D transform =
				PhysicsServer3D::get_singleton()->body_get_state(_body, PhysicsServer3DEnums::BODY_STATE_TRANSFORM);
		_debug_mesh_instance.set_transform(transform);

		if (_shape.is_valid()) {
			Ref<Mesh> mesh = _shape->get_debug_mesh();
			_debug_mesh_instance.set_mesh(mesh);
		}

	} else if (!enabled && _debug_mesh_instance.is_valid()) {
		_debug_mesh_instance.destroy();
	}
}

} // namespace zylann::godot