extends RigidBody3D

@export var wheels: Array[Wheel3D]
@export var acceleration := 20.0
@export var deceleration := 2.0
@export var max_speed := 20
@export var accel_curve: Curve

var motor_input := 0.0

func _unhandled_input(event: InputEvent) -> void:
	motor_input = Input.get_axis("decelerate", "accelerate")

func _physics_process(delta: float) -> void:
	for wheel in wheels:
		wheel.force_raycast_update()
		_do_single_wheel_suspension(wheel)
		_do_single_wheel_acceleration(wheel)

func _get_point_velocity(point: Vector3) -> Vector3:
	return linear_velocity + angular_velocity.cross(point - global_position)

func _do_single_wheel_acceleration(wheel: Wheel3D) -> void:
	var forward_dir := -wheel.global_basis.z
	var vel := forward_dir.dot(linear_velocity)

	wheel.mesh.rotate_x(-vel * get_process_delta_time() * 2 * PI * wheel.wheel_radius)

	if wheel.is_colliding():
		var contact := wheel.mesh.global_position
		var force_pos := contact - global_position

		if wheel.is_motor and motor_input:
			var speed_ratio := vel / max_speed
			var ac := accel_curve.sample_baked(speed_ratio)

			var force_vector := forward_dir * acceleration * motor_input * ac
			apply_force(force_vector, force_pos)
			DebugDraw3D.draw_arrow_ray(contact, force_vector / mass, 1, Color.GREEN_YELLOW, 0.1)
		elif abs(vel) > 0.15 and not motor_input:
			var drag_force_vector = global_basis.z * deceleration * signf(vel)
			apply_force(drag_force_vector, force_pos)
			DebugDraw3D.draw_arrow_ray(contact, drag_force_vector / mass, 1, Color.PURPLE, 0.1)

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
