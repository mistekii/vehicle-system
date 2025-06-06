extends RigidBody3D

@export var wheels: Array[Wheel3D]

func _physics_process(delta: float) -> void:
	for wheel in wheels:
		_do_single_wheel_suspension(wheel)

func _get_point_velocity(point: Vector3) -> Vector3:
	return linear_velocity + angular_velocity.cross(point - global_position)

# calculate & apply suspension forces
func _do_single_wheel_suspension(wheel: Wheel3D) -> void:
	if wheel.is_colliding():
		# ensure that the raycast will only collide when 
		# when the wheel is *actually* touching the ground
		wheel.target_position.y = -(wheel.rest_dist + wheel.wheel_radius + wheel.over_extend)

		var contact := wheel.get_collision_point()
		var spring_up_dir := wheel.global_transform.basis.y
		var spring_len := wheel.global_position.distance_to(contact) - wheel.wheel_radius
		var offset := wheel.rest_dist - spring_len

		wheel.mesh.position.y = -spring_len

		var spring_force := wheel.spring_strength * offset

		var world_vel := _get_point_velocity(contact)
		var relative_vel := spring_up_dir.dot(world_vel)
		var spring_damp_force := wheel.spring_damping * relative_vel

		var force_vector := (spring_force - spring_damp_force) * spring_up_dir

		var force_pos_offset := contact - global_position
		apply_force(force_vector, force_pos_offset)
		DebugDraw3D.draw_arrow_ray(contact, force_vector / mass, 1, Color(1, 0, 0), 0.1)
