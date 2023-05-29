use crate::rapier_world_3d::RapierWorld3D;
use godot::engine::Engine;
use godot::prelude::{Gd, StringName, Vector3};
use rapier3d::na;
use rapier3d::prelude::{
    BroadPhase, CCDSolver, Collider, ColliderHandle, ColliderSet, ImpulseJointSet,
    IntegrationParameters, IslandManager, MultibodyJointSet, NarrowPhase, PhysicsPipeline,
    RigidBodyHandle, RigidBodySet,
};

pub struct RapierPhysicsWorld {
    physics_pipeline: PhysicsPipeline,
    integration_params: IntegrationParameters,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    pub rigid_body_set: RigidBodySet,
    pub collider_set: ColliderSet,
    pub impulse_joint_set: ImpulseJointSet,
    pub multibody_joint_set: MultibodyJointSet,
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

    pub fn insert_rigid_body_collider(
        &mut self,
        collider: Collider,
        body: RigidBodyHandle,
    ) -> ColliderHandle {
        self.collider_set
            .insert_with_parent(collider, body, &mut self.rigid_body_set)
    }
}

pub trait RapierSingletonAccess {
    fn get_rapier3d_instance(&self) -> Option<Gd<RapierWorld3D>> {
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
