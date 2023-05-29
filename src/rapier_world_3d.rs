use crate::rapier_collider_descriptor::RapierColliderDescriptor;
use crate::rapier_physics_world::{RapierPhysicsWorld, RapierSingletonAccess};
use godot::engine::Engine;
use godot::prelude::*;
use rapier3d::na;
use rapier3d::prelude::{
    ColliderBuilder, ColliderHandle, Isometry, RigidBodyBuilder, RigidBodyHandle,
};
use std::cell::RefCell;

#[derive(GodotClass)]
#[class(base=Node3D)]
pub struct RapierWorld3D {
    #[export]
    gravity: Vector3,
    pub rapier: RefCell<RapierPhysicsWorld>,

    #[base]
    base: Base<Node3D>,
}

#[godot_api]
impl NodeVirtual for RapierWorld3D {
    fn init(base: Base<Node3D>) -> Self {
        Self {
            gravity: Vector3::new(0.0, -9.81, 0.0),
            rapier: RefCell::new(RapierPhysicsWorld::new()),
            base,
        }
    }

    fn enter_tree(&mut self) {
        // Check for and remove pre-existing singleton registration.
        if let Some(_rapier_singleton) =
            Engine::singleton().get_singleton(StringName::from("RapierWorld3D"))
        {
            Engine::singleton().unregister_singleton(StringName::from("RapierWorld3D"));
        }

        // Register self as new rapier singleton.
        Engine::singleton().register_singleton(
            StringName::from("RapierWorld3D"),
            Gd::from_instance_id(self.base.instance_id()),
        );
    }

    fn physics_process(&mut self, _delta: f64) {
        if Engine::singleton().is_editor_hint() {
            return;
        }

        // Update rapier every frame
        self.rapier.borrow_mut().step(self.gravity);
    }
}

#[godot_api]
impl RapierWorld3D {}

#[derive(GodotClass)]
#[class(base=Node3D)]
pub struct RapierRigidBody3D {
    rapier_instance: Option<Gd<RapierWorld3D>>,
    rigid_body_handle: Option<RigidBodyHandle>,

    /// Rigid body type; select from four rapier physics bodies.
    ///
    /// 0 -> fixed
    ///
    /// 1 -> dynamic
    ///
    /// 2 -> kinematic_velocity_based
    ///
    /// 3 -> kinematic_position_based
    #[export]
    rigid_body_type: i8,

    #[export]
    gravity_scale: real,

    #[export]
    linear_dampening: real,

    #[export]
    angular_dampening: real,

    #[export]
    can_sleep: bool,

    #[export]
    ccd: bool,

    #[base]
    base: Base<Node3D>,
}

#[godot_api]
impl Node3DVirtual for RapierRigidBody3D {
    fn init(base: Base<Node3D>) -> Self {
        Self {
            rapier_instance: None,
            rigid_body_handle: None,
            rigid_body_type: 1,
            gravity_scale: 1.0,
            linear_dampening: 0.0,
            angular_dampening: 0.0,
            can_sleep: true,
            ccd: false,
            base,
        }
    }

    fn enter_tree(&mut self) {
        if Engine::singleton().is_editor_hint() {
            return;
        }

        self.rapier_instance = self.get_rapier3d_instance();

        let mut world: Option<GdRef<RapierWorld3D>> = None;
        if let Some(rapier_world) = &self.rapier_instance {
            world = Some(rapier_world.bind());
        }

        let builder: Option<RigidBodyBuilder>;

        if self.rigid_body_type == 1 {
            builder = Some(RigidBodyBuilder::dynamic())
        } else if self.rigid_body_type == 2 {
            builder = Some(RigidBodyBuilder::kinematic_velocity_based())
        } else if self.rigid_body_type == 3 {
            builder = Some(RigidBodyBuilder::kinematic_position_based())
        } else {
            builder = Some(RigidBodyBuilder::fixed())
        }

        let global_position = self.get_global_position();
        let global_rotation = self.get_global_rotation();

        let start_position = Isometry::new(
            na::Vector3::new(global_position.x, global_position.y, global_position.z),
            na::Vector3::new(global_rotation.x, global_rotation.y, global_rotation.z),
        );

        if let Some(body) = builder {
            let build = body
                .position(start_position)
                .gravity_scale(self.gravity_scale)
                .linear_damping(self.linear_dampening)
                .angular_damping(self.angular_dampening)
                .can_sleep(self.can_sleep)
                .ccd_enabled(self.ccd)
                .build();

            if let Some(world) = world {
                let handle = world.rapier.borrow_mut().rigid_body_set.insert(build);
                self.rigid_body_handle = Some(handle);
            }
        }
    }

    fn process(&mut self, _delta: f64) {
        if Engine::singleton().is_editor_hint() {
            return;
        }

        let mut world: Option<GdRef<RapierWorld3D>> = None;
        if let Some(rapier_world) = &self.rapier_instance {
            world = Some(rapier_world.bind());
        }

        if let Some(world) = world {
            let bodies = &world.rapier.borrow().rigid_body_set;
            let rapier_body = bodies.get(self.rigid_body_handle.unwrap()).unwrap();
            let body_pos = &rapier_body.position();

            self.base.set_global_position(Vector3::new(
                body_pos.translation.x,
                body_pos.translation.y,
                body_pos.translation.z,
            ));

            let body_orientation = rapier_body.rotation().quaternion().as_vector();
            self.base.set_basis(Basis::from_quat(Quaternion::new(
                body_orientation.x,
                body_orientation.y,
                body_orientation.z,
                body_orientation.w,
            )))
        }
    }
}

#[godot_api]
impl RapierRigidBody3D {}

impl RapierSingletonAccess for RapierRigidBody3D {}

#[derive(GodotClass)]
#[class(base=Node3D)]
pub struct RapierCuboidCollider3D {
    collider: Option<ColliderHandle>,

    #[export]
    descriptor: Option<Gd<RapierColliderDescriptor>>,

    #[export]
    rigid_body_parent: Option<Gd<RapierRigidBody3D>>,

    #[export]
    dimensions: Vector3,

    #[base]
    base: Base<Node3D>,
}

#[godot_api]
impl Node3DVirtual for RapierCuboidCollider3D {
    fn init(base: Base<Node3D>) -> Self {
        Self {
            collider: None,
            descriptor: None,
            rigid_body_parent: None,
            dimensions: Vector3::new(0.5, 0.5, 0.5),
            base,
        }
    }

    fn ready(&mut self) {
        if Engine::singleton().is_editor_hint() {
            return;
        }

        let world = self.get_rapier3d_instance().unwrap();
        let world = world.bind();

        let mut rapier = world.rapier.borrow_mut();

        let mut collider: Option<ColliderBuilder> = None;

        if let Some(descriptor) = &self.descriptor {
            let description = descriptor.bind();
            collider = Some(
                ColliderBuilder::cuboid(self.dimensions.x, self.dimensions.y, self.dimensions.z)
                    .restitution(description.restitution)
                    .density(description.density)
                    .friction(description.friction)
                    .sensor(description.sensor),
            );
        }

        match &self.rigid_body_parent {
            Some(parent) => {
                let parent = parent.bind();

                self.collider = Some(rapier.insert_rigid_body_collider(
                    collider.expect("No collider to insert!").build(),
                    parent.rigid_body_handle.unwrap(),
                ));
            }
            None => {
                let start_pos = self.base.get_global_position();
                let start_rot = self.base.get_global_rotation();

                let isolated_collider = collider.expect("No collider!").position(Isometry::new(
                    na::Vector3::new(start_pos.x, start_pos.y, start_pos.z),
                    na::Vector3::new(start_rot.x, start_rot.y, start_rot.z),
                ));

                self.collider = Some(rapier.collider_set.insert(isolated_collider.build()));
            }
        }
    }
}

#[godot_api]
impl RapierCuboidCollider3D {}

impl RapierSingletonAccess for RapierCuboidCollider3D {}

#[derive(GodotClass)]
#[class(base=Node3D)]
pub struct RapierBallCollider3D {
    collider: Option<ColliderHandle>,

    #[export]
    descriptor: Option<Gd<RapierColliderDescriptor>>,

    #[export]
    rigid_body_parent: Option<Gd<RapierRigidBody3D>>,

    #[export]
    radius: real,

    #[base]
    base: Base<Node3D>,
}

#[godot_api]
impl Node3DVirtual for RapierBallCollider3D {
    fn init(base: Base<Node3D>) -> Self {
        Self {
            collider: None,
            descriptor: None,
            rigid_body_parent: None,
            radius: 0.5,
            base,
        }
    }

    fn ready(&mut self) {
        if Engine::singleton().is_editor_hint() {
            return;
        }

        let world = self.get_rapier3d_instance().unwrap();
        let world = world.bind();

        let mut rapier = world.rapier.borrow_mut();

        let mut collider: Option<ColliderBuilder> = None;

        if let Some(descriptor) = &self.descriptor {
            let description = descriptor.bind();
            collider = Some(
                ColliderBuilder::ball(self.radius)
                    .restitution(description.restitution)
                    .density(description.density)
                    .friction(description.friction)
                    .sensor(description.sensor),
            );
        }

        match &self.rigid_body_parent {
            Some(parent) => {
                let parent = parent.bind();

                self.collider = Some(rapier.insert_rigid_body_collider(
                    collider.expect("No collider to insert!").build(),
                    parent.rigid_body_handle.unwrap(),
                ));
            }
            None => {
                let start_pos = self.base.get_global_position();
                let start_rot = self.base.get_global_rotation();

                let isolated_collider = collider.expect("No collider!").position(Isometry::new(
                    na::Vector3::new(start_pos.x, start_pos.y, start_pos.z),
                    na::Vector3::new(start_rot.x, start_rot.y, start_rot.z),
                ));

                self.collider = Some(rapier.collider_set.insert(isolated_collider.build()));
            }
        }
    }
}

#[godot_api]
impl RapierBallCollider3D {}

impl RapierSingletonAccess for RapierBallCollider3D {}

#[derive(GodotClass)]
#[class(base=Node3D)]
pub struct RapierCapsuleCollider3D {
    collider: Option<ColliderHandle>,

    #[export]
    descriptor: Option<Gd<RapierColliderDescriptor>>,

    #[export]
    rigid_body_parent: Option<Gd<RapierRigidBody3D>>,

    /// The axis in which the capsule collider will align itself.
    ///
    /// 0 -> x
    ///
    /// 1 -> y
    ///
    /// 2 -> z
    #[export]
    axis: i8,

    #[export]
    radius: real,

    #[export]
    height: real,

    #[base]
    base: Base<Node3D>,
}

#[godot_api]
impl Node3DVirtual for RapierCapsuleCollider3D {
    fn init(base: Base<Node3D>) -> Self {
        Self {
            collider: None,
            descriptor: None,
            rigid_body_parent: None,
            axis: 1,
            radius: 0.5,
            height: 0.5,
            base,
        }
    }

    fn ready(&mut self) {
        if Engine::singleton().is_editor_hint() {
            return;
        }

        let world = self.get_rapier3d_instance().unwrap();
        let world = world.bind();

        let mut rapier = world.rapier.borrow_mut();

        let mut collider: Option<ColliderBuilder> = None;

        if let Some(descriptor) = &self.descriptor {
            let description = descriptor.bind();

            let builder: Option<ColliderBuilder>;

            if self.axis == 0 {
                builder = Some(ColliderBuilder::capsule_x(self.height, self.radius));
            } else if self.axis == 1 {
                builder = Some(ColliderBuilder::capsule_y(self.height, self.radius));
            } else if self.axis == 2 {
                builder = Some(ColliderBuilder::capsule_z(self.height, self.radius));
            } else {
                builder = None;
            }

            if let Some(new_collider) = builder {
                collider = Some(
                    new_collider
                        .restitution(description.restitution)
                        .density(description.density)
                        .friction(description.friction)
                        .sensor(description.sensor),
                )
            } else {
                collider = None;
            }
        }

        match &self.rigid_body_parent {
            Some(parent) => {
                let parent = parent.bind();

                self.collider = Some(rapier.insert_rigid_body_collider(
                    collider.expect("No collider to insert!").build(),
                    parent.rigid_body_handle.unwrap(),
                ));
            }
            None => {
                let start_pos = self.base.get_global_position();
                let start_rot = self.base.get_global_rotation();

                let isolated_collider = collider.expect("No collider!").position(Isometry::new(
                    na::Vector3::new(start_pos.x, start_pos.y, start_pos.z),
                    na::Vector3::new(start_rot.x, start_rot.y, start_rot.z),
                ));

                self.collider = Some(rapier.collider_set.insert(isolated_collider.build()));
            }
        }
    }
}

#[godot_api]
impl RapierCapsuleCollider3D {}

impl RapierSingletonAccess for RapierCapsuleCollider3D {}
