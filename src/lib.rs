mod godot_node_ref;
mod rapier_world_3d;

use godot::prelude::*;

struct RapierPhysics;

#[gdextension]
unsafe impl ExtensionLibrary for RapierPhysics {}
