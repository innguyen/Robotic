extends Node3D
## S0: ideal kinematic omni body, NOT wheel/traction dynamics.
## ENU world / FLU base -> Godot (X,Y,Z) = (east,up,-north).
const DT := 1.0 / 60.0
const RAYS := 192
const MAX_RANGE := 6.0
const SCENARIOS := ["clear", "occluded", "obstacle", "absent", "calibration"]
var server := TCPServer.new()
var peer: StreamPeerTCP
var rx := ""
var pending: Dictionary = {}
var busy := false
var remaining := 0
var command := Vector3.ZERO # vx, vy, wz, in FLU
var tick := 0
var epoch := 0
var scenario := "clear"
var seed_value := 7
var robot: CharacterBody3D
var person: StaticBody3D
var barrier: StaticBody3D
var viewport: SubViewport
var camera: Camera3D
var label: Label
var render_rgb := true
var collisions := 0

func material(color: Color) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.albedo_color = color
	m.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	return m

func box(parent: Node3D, size: Vector3, pos: Vector3, color: Color, solid := true) -> StaticBody3D:
	var body := StaticBody3D.new()
	parent.add_child(body)
	body.position = pos
	body.collision_layer = 1 if solid else 0
	var mesh := MeshInstance3D.new()
	var shape := BoxMesh.new()
	shape.size = size
	mesh.mesh = shape
	mesh.material_override = material(color)
	body.add_child(mesh)
	var collider := CollisionShape3D.new()
	var bounds := BoxShape3D.new()
	bounds.size = size
	collider.shape = bounds
	body.add_child(collider)
	return body

func _ready() -> void:
	Engine.max_fps = 60
	render_rgb = DisplayServer.get_name() != "headless"
	box(self, Vector3(10, 0.1, 10), Vector3(0, -0.05, 0), Color(0.27, 0.30, 0.32))
	for x in [-5.0, 5.0]:
		box(self, Vector3(0.1, 2.4, 10), Vector3(x, 1.2, 0), Color(0.5, 0.53, 0.56))
	for z in [-5.0, 5.0]:
		box(self, Vector3(10, 2.4, 0.1), Vector3(0, 1.2, z), Color(0.5, 0.53, 0.56))
	barrier = box(self, Vector3(0.12, 2.2, 1.4), Vector3(1.5, 1.1, 0), Color(0.7, 0.63, 0.33))
	# Capsule mannequin; its collider is also hit by horizontal range rays.
	person = StaticBody3D.new()
	add_child(person)
	var mannequin := MeshInstance3D.new()
	var capsule := CapsuleMesh.new()
	capsule.radius = 0.22
	capsule.height = 1.6
	mannequin.mesh = capsule
	mannequin.material_override = material(Color(0.9, 0.035, 0.035))
	person.add_child(mannequin)
	var human_collision := CollisionShape3D.new()
	var human_shape := CapsuleShape3D.new()
	human_shape.radius = 0.22
	human_shape.height = 1.6
	human_collision.shape = human_shape
	person.add_child(human_collision)
	robot = CharacterBody3D.new()
	add_child(robot)
	robot.motion_mode = CharacterBody3D.MOTION_MODE_FLOATING
	robot.collision_layer = 2
	robot.collision_mask = 1
	var chassis := MeshInstance3D.new()
	var chassis_mesh := BoxMesh.new()
	chassis_mesh.size = Vector3(0.4, 0.2, 0.4)
	chassis.mesh = chassis_mesh
	chassis.material_override = material(Color(0.08, 0.5, 0.9))
	robot.add_child(chassis)
	var robot_collision := CollisionShape3D.new()
	var robot_shape := BoxShape3D.new()
	robot_shape.size = chassis_mesh.size
	robot_collision.shape = robot_shape
	robot.add_child(robot_collision)
	viewport = SubViewport.new()
	viewport.size = Vector2i(160, 120)
	viewport.world_3d = get_world_3d()
	viewport.render_target_update_mode = SubViewport.UPDATE_DISABLED
	add_child(viewport)
	camera = Camera3D.new()
	camera.fov = rad_to_deg(1.012290966)
	camera.keep_aspect = Camera3D.KEEP_WIDTH
	camera.near = 0.03
	camera.far = 12.0
	viewport.add_child(camera)
	var overview := Camera3D.new()
	add_child(overview)
	overview.position = Vector3(6, 8, 7)
	overview.look_at(Vector3(0.8, 0, 0))
	overview.current = true
	var canvas := CanvasLayer.new()
	add_child(canvas)
	label = Label.new()
	label.position = Vector2(12, 12)
	canvas.add_child(label)
	var preview := TextureRect.new()
	preview.position = Vector2(12, 70)
	preview.size = Vector2(320, 240)
	preview.texture = viewport.get_texture()
	canvas.add_child(preview)
	reset_fixture("clear", 7)
	var port := 17650
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--port="):
			port = int(arg.get_slice("=", 1))
	var error := server.listen(port, "127.0.0.1")
	if error != OK:
		push_error("S0 listen failed: %s" % error)
		get_tree().quit(2)
		return
	print("S0_READY port=%d rgb=%s renderer=%s" % [port, render_rgb, RenderingServer.get_current_rendering_method()])

func reset_fixture(name_value: String, seed_number: int) -> void:
	scenario = name_value
	seed_value = seed_number
	tick = 0
	collisions = 0
	command = Vector3.ZERO
	robot.velocity = Vector3.ZERO
	robot.position = Vector3(0, 0.11, 0)
	robot.rotation = Vector3.ZERO
	# Fixed fixtures: seed is identity only; no randomized content in S0.
	if scenario == "clear":
		robot.rotation.y = -PI / 2.0 # person starts outside camera FOV; requires search spin
	person.position = Vector3(3, 0.8, 0) if scenario != "absent" and scenario != "calibration" else Vector3(30, 0.8, 0)
	barrier.position = Vector3(1.5, 1.1, 0) if scenario == "occluded" else Vector3(30, 1.1, 0)
	if scenario == "obstacle":
		barrier.position = Vector3(1.5, 1.1, 0)
	sync_camera()

func sync_camera() -> void:
	camera.global_position = robot.to_global(Vector3(0.12, 0.44, 0))
	camera.global_rotation = Vector3(0, robot.rotation.y - PI / 2.0, 0)

func send_error(message: String, request_id = null) -> void:
	if peer != null:
		peer.put_data((JSON.stringify({"ok": false, "id": request_id, "error": message, "epoch": epoch}) + "\n").to_utf8_buffer())

func _process(_delta: float) -> void:
	label.text = "Godot S0 | %s | epoch %d | sim %.3fs\nTCP 127.0.0.1 | external Python lockstep | RGB 160x120 / 192 rays" % [scenario, epoch, tick * DT]
	if peer == null:
		if server.is_connection_available() and not busy:
			peer = server.take_connection()
			peer.set_no_delay(true)
		return
	peer.poll()
	if peer.get_status() != StreamPeerTCP.STATUS_CONNECTED:
		peer = null
		rx = ""
		command = Vector3.ZERO
		remaining = 0
		robot.velocity = Vector3.ZERO
		if not pending.is_empty():
			pending = {}
			busy = false
		return
	var available := peer.get_available_bytes()
	if available > 0:
		var packet := peer.get_data(available)
		if packet[0] != OK:
			peer.disconnect_from_host()
			return
		rx += packet[1].get_string_from_utf8()
	if rx.length() > 65536:
		send_error("request_too_large")
		peer.disconnect_from_host()
		return
	if busy or not rx.contains("\n"):
		return
	var newline := rx.find("\n")
	var raw := rx.substr(0, newline)
	rx = rx.substr(newline + 1)
	var request = JSON.parse_string(raw)
	if not request is Dictionary:
		send_error("invalid_json_object")
		return
	var op = request.get("op", "")
	var request_id = request.get("id")
	if op not in ["observe", "reset", "step", "quit"]:
		send_error("unknown_op", request_id)
		return
	if op == "reset":
		if request.get("scenario", "clear") not in SCENARIOS or not finite_number(request.get("seed", 7)):
			send_error("invalid_fixture", request_id)
			return
	if op == "step":
		if request.get("epoch", -1) != epoch:
			send_error("stale_epoch", request_id)
			return
		var values = request.get("velocity")
		var ticks = request.get("ticks", 1)
		if not values is Array or values.size() != 3 or not finite_number(ticks):
			send_error("invalid_step", request_id)
			return
		if ticks < 1 or ticks > 600 or ticks != int(ticks):
			send_error("invalid_ticks", request_id)
			return
		for i in range(3):
			if not finite_number(values[i]) or abs(values[i]) > [0.6, 0.4, 1.0][i]:
				send_error("invalid_velocity", request_id)
				return
	pending = request
	busy = true

func finite_number(value) -> bool:
	return (value is float or value is int) and is_finite(float(value))

func _physics_process(_delta: float) -> void:
	if not busy or pending.is_empty():
		return
	var op: String = pending.op
	if op == "reset":
		reset_fixture(pending.get("scenario", "clear"), int(pending.get("seed", 7)))
		epoch += 1
		pending.op = "observe" # wait one physics tick for collision transform synchronization
		return
	if op == "step":
		if remaining == 0:
			remaining = int(pending.get("ticks", 1))
			var v: Array = pending.velocity
			command = Vector3(v[0], v[1], v[2])
		robot.rotation.y += command.z * DT
		robot.velocity = robot.basis * Vector3(command.x, 0, -command.y)
		# move_and_collide uses explicit displacement; rotation collision sweep is not modeled.
		var hit := robot.move_and_collide(robot.velocity * DT)
		if hit != null:
			collisions += 1
		tick += 1
		remaining -= 1
		if remaining > 0:
			return
	command = Vector3.ZERO # bounded command duration; never persists past requested ticks
	robot.velocity = Vector3.ZERO
	sync_camera()
	var request := pending
	pending = {}
	var observation := sample_sensors()
	finish_response(request, observation)

func sample_sensors() -> Dictionary:
	var ranges: Array = []
	var origin := robot.to_global(Vector3(0.12, 0.14, 0)) # world z=.25m
	var space := get_world_3d().direct_space_state
	for i in range(RAYS):
		var angle := -PI + (i + 0.5) * TAU / RAYS
		var direction := robot.basis * Vector3(cos(angle), 0, -sin(angle))
		var query := PhysicsRayQueryParameters3D.create(origin, origin + MAX_RANGE * direction, 1, [robot.get_rid()])
		var hit := space.intersect_ray(query)
		ranges.append(null if hit.is_empty() else origin.distance_to(hit.position))
	var stamp := tick * DT
	return {
		"protocol": 1, "epoch": epoch, "tick": tick, "simulation_timestamp": stamp,
		"scenario": scenario, "seed": seed_value, "dt": DT,
		"pose": {"frame": "world_enu", "source": "engine_ideal_self_pose", "stamp": stamp,
			"x": robot.position.x, "y": -robot.position.z, "z": robot.position.y, "yaw": robot.rotation.y},
		"scan": {"frame": "base_flu", "stamp": stamp, "origin": [0.12, 0, 0.14],
			"angle_min": -PI + PI / RAYS, "angle_increment": TAU / RAYS,
			"range_min": 0.0, "range_max": MAX_RANGE, "no_return": "null", "ranges": ranges},
		"camera": {"frame": "base_flu", "stamp": stamp, "width": 160, "height": 120, "horizontal_fov": 1.012290966,
			"origin": [0.12, 0, 0.44], "format": "rgb8_base64", "valid": false, "data": null},
		"diagnostics": {"collision_count": collisions, "applied_velocity": [0, 0, 0],
			"engine_physics_frames": Engine.get_physics_frames(),
			"renderer": RenderingServer.get_current_rendering_method(), "display": DisplayServer.get_name(),
			"fps": Engine.get_frames_per_second(), "static_memory_bytes": OS.get_static_memory_usage()}
	}

func finish_response(request: Dictionary, observation: Dictionary) -> void:
	if render_rgb:
		viewport.render_target_update_mode = SubViewport.UPDATE_ONCE
		await RenderingServer.frame_post_draw
		var image := viewport.get_texture().get_image()
		if image != null and not image.is_empty():
			image.convert(Image.FORMAT_RGB8)
			observation.camera.valid = true
			observation.camera.data = Marshalls.raw_to_base64(image.get_data())
	if peer != null:
		peer.put_data((JSON.stringify({"ok": true, "id": request.get("id"), "observation": observation}) + "\n").to_utf8_buffer())
	busy = false
	if request.op == "quit":
		get_tree().quit()
