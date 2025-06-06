extends RayCast3D
class_name Wheel3D

@export var spring_strength := 150.0
@export var spring_damping := 4.0
@export var rest_dist := 0.5
@export var over_extend := 0.0 # used to control how much the wheel "sticks" to the ground
@export var wheel_radius := 0.4
@export var is_motor := false
@export var grip_curve : Curve

@onready var mesh: Node3D = get_child(0)
