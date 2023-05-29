use godot::engine::Engine;
use godot::prelude::*;
use rapier3d::dynamics::{
    CCDSolver, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet,
    RigidBodySet,
};
use rapier3d::geometry::{BroadPhase, ColliderSet, NarrowPhase};
use rapier3d::na;
use rapier3d::pipeline::PhysicsPipeline;
use rapier3d::prelude::{Collider, ColliderBuilder, Isometry, RigidBodyBuilder, RigidBodyHandle};
use std::cell::RefCell;

pub struct RapierPhysicsWorld {
    physics_pipeline: PhysicsPipeline,
    integration_params: IntegrationParameters,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
}

impl RapierPhysicsWorld {
    pub fn new() -> Self {
        Self {
            physics_pipeline: PhysicsPipeline::new(),
            integration_params: IntegrationParameters::default(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
        }
    }

    pub fn step(&mut self, gravity: Vector3) {
        let gravity = na::Vector3::new(gravity.x, gravity.y, gravity.z);
        let physics_hook = ();
        let event_handler = ();

        self.physics_pipeline.step(
            &gravity,
            &self.integration_params,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            None,
            &physics_hook,
            &event_handler,
        )
    }

    pub fn insert_rigid_body_collider(&mut self, collider: Collider, body: RigidBodyHandle) {
        self.collider_set
            .insert_with_parent(collider, body, &mut self.rigid_body_set);
    }
}

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

    fn process(&mut self, _delta: f64) {
        if Engine::singleton().is_editor_hint() {
            return;
        }

        // Update rapier every frame
        self.rapier.borrow_mut().step(self.gravity);
    }
}

#[godot_api]
impl RapierWorld3D {}

pub trait RapierSingletonAccess {
    fn get_rapier_instance(&self) -> Option<Gd<RapierWorld3D>> {
        let rapier_singleton = Engine::singleton().get_singleton(StringName::from("RapierWorld3D"));

        let rapier_instance: Option<Gd<RapierWorld3D>>;

        match rapier_singleton {
            Some(instance) => match instance.try_cast::<RapierWorld3D>() {
                Some(rapier) => rapier_instance = Some(rapier),
                None => rapier_instance = None,
            },

            None => rapier_instance = None,
        };

        return rapier_instance;
    }
}

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

        self.rapier_instance = self.get_rapier_instance();

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

                // Test collider, please remove later.
                // world.rapier.borrow_mut().insert_rigid_body_collider(
                //     ColliderBuilder::cuboid(0.5, 0.5, 0.5)
                //         .restitution(0.3)
                //         .build(),
                //     handle,
                // );
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
            let body_pos = &mut rapier_body.position();

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
    collider: Option<Collider>,

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

        let world = self.get_rapier_instance().unwrap();
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

                rapier.insert_rigid_body_collider(
                    collider.expect("No collider to insert!").build(),
                    parent.rigid_body_handle.unwrap(),
                );
            }
            None => {
                let start_pos = self.base.get_global_position();
                let start_rot = self.base.get_global_rotation();

                let isolated_collider = collider.expect("No collider!").position(
                    Isometry::new(
                        na::Vector3::new(start_pos.x, start_pos.y, start_pos.z),
                        na::Vector3::new(start_rot.x, start_rot.y, start_rot.z)
                    )
                );

                rapier
                    .collider_set
                    .insert(isolated_collider.build());
            }
        }
    }
}

impl RapierSingletonAccess for RapierCuboidCollider3D {}

#[godot_api]
impl RapierCuboidCollider3D {}

#[derive(GodotClass)]
#[class(base=Resource)]
pub struct RapierColliderDescriptor {
    #[export]
    restitution: real,

    #[export]
    density: real,

    #[export]
    friction: real,

    #[export]
    sensor: bool,

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