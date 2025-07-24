extends Node3D

# --- Parameters ---
const GROUND_SIZE = Vector3(20, 1, 20)
const BIKE_FRAME_SIZE = Vector3(1.2, 0.2, 0.4)
const WHEEL_RADIUS = 0.35
const WHEEL_WIDTH = 0.08
const WHEELBASE = 1.0
const SUSPENSION_REST_LENGTH = 0.25
const SUSPENSION_STIFFNESS = 1200.0
const SUSPENSION_DAMPING = 120.0
const BIKE_MASS = 15.0
const WHEEL_MASS = 1.0
const PEDAL_FORCE = 60.0
const BRAKE_FORCE = 80.0
const STEER_ANGLE_MAX = 0.5 # radians
const STEER_SPEED = 2.5 # radians/sec

# --- Nodes ---
var bike_frame: RigidBody3D
var front_wheel: CharacterBody3D
var rear_wheel: CharacterBody3D
var steer_angle := 0.0
var steer_input := 0.0
var pedal_input := 0.0
var brake_input := 0.0

# --- Suspension State ---
var front_susp_pos := 0.0
var front_susp_vel := 0.0
var rear_susp_pos := 0.0
var rear_susp_vel := 0.0

func _ready():
	# Set up ground
	var ground = StaticBody3D.new()
	var ground_mesh = MeshInstance3D.new()
	var ground_box = BoxMesh.new()
	ground_box.size = GROUND_SIZE
	ground_mesh.mesh = ground_box
	ground_mesh.position = Vector3(0, -GROUND_SIZE.y/2, 0)
	ground.add_child(ground_mesh)
	var ground_shape = CollisionShape3D.new()
	var ground_box_shape = BoxShape3D.new()
	ground_box_shape.size = GROUND_SIZE
	ground_shape.shape = ground_box_shape
	ground_shape.position = Vector3(0, -GROUND_SIZE.y/2, 0)
	ground.add_child(ground_shape)
	add_child(ground)

	# Set up bike frame
	bike_frame = RigidBody3D.new()
	bike_frame.mass = BIKE_MASS
	bike_frame.position = Vector3(0, 1.0, 0)
	var frame_mesh = MeshInstance3D.new()
	var frame_box = BoxMesh.new()
	frame_box.size = BIKE_FRAME_SIZE
	frame_mesh.mesh = frame_box
	bike_frame.add_child(frame_mesh)
	var frame_shape = CollisionShape3D.new()
	var frame_box_shape = BoxShape3D.new()
	frame_box_shape.size = BIKE_FRAME_SIZE
	frame_shape.shape = frame_box_shape
	bike_frame.add_child(frame_shape)
	add_child(bike_frame)

	# Set up wheels
	front_wheel = _create_wheel(Vector3(WHEELBASE/2, 1.0 - SUSPENSION_REST_LENGTH, 0))
	rear_wheel = _create_wheel(Vector3(-WHEELBASE/2, 1.0 - SUSPENSION_REST_LENGTH, 0))
	add_child(front_wheel)
	add_child(rear_wheel)

	# Camera
	var cam = Camera3D.new()
	cam.position = Vector3(0, 3, 6)
	cam.look_at(Vector3(0, 1, 0))
	add_child(cam)

	set_process(true)
	set_physics_process(true)

func _create_wheel(pos: Vector3) -> CharacterBody3D:
	var wheel = CharacterBody3D.new()
	wheel.position = pos
	var mesh = MeshInstance3D.new()
	var cyl = CylinderMesh.new()
	cyl.top_radius = WHEEL_RADIUS
	cyl.bottom_radius = WHEEL_RADIUS
	cyl.height = WHEEL_WIDTH
	mesh.mesh = cyl
	mesh.rotation = Vector3(deg_to_rad(90), 0, 0)
	wheel.add_child(mesh)
	var shape = CollisionShape3D.new()
	var cyl_shape = CylinderShape3D.new()
	cyl_shape.radius = WHEEL_RADIUS
	cyl_shape.height = WHEEL_WIDTH
	shape.shape = cyl_shape
	shape.rotation = Vector3(deg_to_rad(90), 0, 0)
	wheel.add_child(shape)
	return wheel

func _unhandled_input(event):
	steer_input = 0.0
	pedal_input = 0.0
	brake_input = 0.0
	if Input.is_action_pressed("ui_left"):
		steer_input -= 1.0
	if Input.is_action_pressed("ui_right"):
		steer_input += 1.0
	if Input.is_action_pressed("ui_up"):
		pedal_input += 1.0
	if Input.is_action_pressed("ui_down"):
		brake_input += 1.0

func _physics_process(delta):
	# --- Steering ---
	steer_angle = clamp(steer_angle + steer_input * STEER_SPEED * delta, -STEER_ANGLE_MAX, STEER_ANGLE_MAX)
	# Move front wheel left/right
	var fw_pos = bike_frame.global_transform.origin + bike_frame.global_transform.basis.x * (WHEELBASE/2)
	var rw_pos = bike_frame.global_transform.origin - bike_frame.global_transform.basis.x * (WHEELBASE/2)
	front_wheel.global_transform.origin = fw_pos - Vector3(0, SUSPENSION_REST_LENGTH + front_susp_pos, 0)
	rear_wheel.global_transform.origin = rw_pos - Vector3(0, SUSPENSION_REST_LENGTH + rear_susp_pos, 0)
	front_wheel.rotation.y = bike_frame.rotation.y + steer_angle
	rear_wheel.rotation.y = bike_frame.rotation.y

	# --- Suspension (vertical spring-damper, simplified) ---
	var front_target = fw_pos.y - WHEEL_RADIUS
	var rear_target = rw_pos.y - WHEEL_RADIUS
	var ground_y = 0.0
	var front_compression = clamp(front_target - ground_y, 0, SUSPENSION_REST_LENGTH)
	var rear_compression = clamp(rear_target - ground_y, 0, SUSPENSION_REST_LENGTH)
	var front_spring_force = SUSPENSION_STIFFNESS * (SUSPENSION_REST_LENGTH - front_compression) - SUSPENSION_DAMPING * front_susp_vel
	var rear_spring_force = SUSPENSION_STIFFNESS * (SUSPENSION_REST_LENGTH - rear_compression) - SUSPENSION_DAMPING * rear_susp_vel
	front_susp_vel += (front_spring_force / (BIKE_MASS/2)) * delta
	rear_susp_vel += (rear_spring_force / (BIKE_MASS/2)) * delta
	front_susp_pos += front_susp_vel * delta
	rear_susp_pos += rear_susp_vel * delta
	front_susp_pos = clamp(front_susp_pos, 0, SUSPENSION_REST_LENGTH)
	rear_susp_pos = clamp(rear_susp_pos, 0, SUSPENSION_REST_LENGTH)

	# --- Drive/Brake Forces (applied at rear wheel) ---
	var forward_dir = bike_frame.global_transform.basis.x
	var force = Vector3.ZERO
	if pedal_input > 0.0:
		force += forward_dir * PEDAL_FORCE * pedal_input
	if brake_input > 0.0:
		force -= forward_dir * BRAKE_FORCE * brake_input
	bike_frame.apply_force(force)

	# --- Simple steering torque (applied to frame) ---
	var steer_torque = steer_angle * 20.0
	bike_frame.apply_torque(Vector3(0, steer_torque, 0))

	# --- Print debug info ---
	var vel = bike_frame.linear_velocity.length()
	var lean = rad_to_deg(bike_frame.rotation.z)
	print("Speed: %.2f m/s | Lean: %.2f deg | Steer: %.2f deg" % [vel, lean, rad_to_deg(steer_angle)])
