extends RigidBody3D

@export var wheels: Array[Wheel3D]
@export var acceleration := 20.0
@export var max_speed := 20
@export var accel_curve: Curve
@export var tire_turn_speed := 2.0
@export var tire_max_turn_degrees := 25

var motor_input := 0.0
var handbrake := false
var is_slipping := false

func _unhandled_input(event: InputEvent) -> void:
	handbrake = Input.is_action_pressed("handbrake")
	
	if Input.is_action_pressed("handbrake"):	
		is_slipping = true
	
	
	motor_input = Input.get_axis("decelerate", "accelerate")

func _physics_process(delta: float) -> void:
	_basic_steering_rotation(delta)
	
	for wheel in wheels:
		wheel.force_raycast_update()
		_do_single_wheel_suspension(wheel)
		_do_single_wheel_acceleration(wheel)
		_do_single_wheel_traction(wheel)

func _get_point_velocity(point: Vector3) -> Vector3:
	return linear_velocity + angular_velocity.cross(point - global_position)

func _basic_steering_rotation(delta: float) -> void:
	var turn_input := Input.get_axis("turn_right", "turn_left") * tire_turn_speed

	if turn_input:
		$wheels/fl.rotation.y = clampf($wheels/fl.rotation.y + turn_input * delta,
			deg_to_rad(-tire_max_turn_degrees), deg_to_rad(tire_max_turn_degrees))
		$wheels/fr.rotation.y = clampf($wheels/fl.rotation.y + turn_input * delta,
			deg_to_rad(-tire_max_turn_degrees), deg_to_rad(tire_max_turn_degrees))
	else:
		$wheels/fl.rotation.y = move_toward($wheels/fl.rotation.y, 0, tire_turn_speed * delta)
		$wheels/fr.rotation.y = move_toward($wheels/fr.rotation.y, 0, tire_turn_speed * delta)

func _do_single_wheel_acceleration(wheel: Wheel3D) -> void:
	var forward_dir := -wheel.global_basis.z
	var vel := forward_dir.dot(linear_velocity)

	wheel.mesh.rotate_x((-vel * get_process_delta_time()) / wheel.wheel_radius)

	if wheel.is_colliding():
		var contact := wheel.mesh.global_position
		var force_pos := contact - global_position

		if wheel.is_motor and motor_input:
			var speed_ratio := vel / max_speed
			var ac := accel_curve.sample_baked(speed_ratio)

			var force_vector := forward_dir * acceleration * motor_input * ac
			apply_force(force_vector, force_pos)
			DebugDraw3D.draw_arrow_ray(contact, force_vector / mass, 1, Color.HOT_PINK, 0.1)

func _do_single_wheel_traction(wheel: Wheel3D) -> void:
	if not wheel.is_colliding(): return

	var steer_side_dir := wheel.global_basis.x
	var tire_vel := _get_point_velocity(wheel.mesh.global_position)
	var steering_x_vel := steer_side_dir.dot(tire_vel)

	var grip_factor := absf(steering_x_vel / tire_vel.length())
	var x_traction := wheel.grip_curve.sample_baked(grip_factor)

	if not handbrake and grip_factor < 0.2:
		is_slipping = false

	if handbrake:
		x_traction = 0.1
	elif is_slipping:
		x_traction = 0.1

	var gravity: float = ProjectSettings.get_setting("physics/3d/default_gravity")
	var x_force := -steer_side_dir * steering_x_vel * x_traction * ((mass * gravity) / 4.0)

	var f_vel := -wheel.global_basis.z.dot(tire_vel)
	var z_traction := 0.05
	var z_force := wheel.global_basis.z * f_vel * z_traction * ((mass * gravity) / 4.0)

	var force_pos := wheel.mesh.global_position - global_position
	apply_force(x_force, force_pos)
	apply_force(z_force, force_pos)
	DebugDraw3D.draw_arrow_ray(wheel.mesh.global_position, x_force / mass, 1, Color.ORANGE, 0.1)
	DebugDraw3D.draw_arrow_ray(wheel.mesh.global_position, z_force / mass, 1, Color.BLUE, 0.1)

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

		var force_vector := (spring_force - spring_damp_force) * wheel.get_collision_normal()

		contact = wheel.mesh.global_position
		var force_pos_offset := contact - global_position
		apply_force(force_vector, force_pos_offset)
		DebugDraw3D.draw_arrow_ray(contact, force_vector / mass, 1, Color.RED, 0.1)
