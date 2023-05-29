use godot::prelude::{godot_api, real, Base, GodotClass, Resource, ResourceVirtual};

#[derive(GodotClass)]
#[class(base=Resource)]
pub struct RapierColliderDescriptor {
    #[export]
    pub restitution: real,

    #[export]
    pub density: real,

    #[export]
    pub friction: real,

    #[export]
    pub sensor: bool,

    #[base]
    base: Base<Resource>,
}

#[godot_api]
impl ResourceVirtual for RapierColliderDescriptor {
    fn init(base: Base<Self::Base>) -> Self {
        Self {
            restitution: 0.2,
            density: 1.0,
            friction: 0.5,
            sensor: false,
            base,
        }
    }
}

#[godot_api]
impl RapierColliderDescriptor {}
