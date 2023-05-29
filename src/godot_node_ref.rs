use godot::obj::mem::ManualMemory;
use godot::prelude::*;

#[derive(Clone, Debug, Copy)]
pub struct GodotNodeRef(InstanceId);

impl GodotNodeRef {
    pub fn new<T: GodotClass<Mem = ManualMemory>>(node: Gd<T>) -> Self {
        Self(node.instance_id())
    }

    pub fn from_id(id: InstanceId) -> Self {
        Self(id)
    }

    pub fn get<T: GodotClass<Mem = ManualMemory>>(&self) -> Gd<T> {
        Gd::from_instance_id(self.0)
    }
}
