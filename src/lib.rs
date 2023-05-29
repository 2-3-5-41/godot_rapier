mod rapier_collider_descriptor;
mod rapier_physics_world;
mod rapier_world_3d;

use godot::prelude::*;

struct RapierPhysics;

#[gdextension]
unsafe impl ExtensionLibrary for RapierPhysics {}
