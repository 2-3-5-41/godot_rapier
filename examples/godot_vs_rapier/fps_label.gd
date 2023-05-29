extends Label

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	var fps := Engine.get_frames_per_second()
	self.text = String("FPS...").num_scientific(fps)
