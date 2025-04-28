pub mod car;
pub mod wheel;

pub mod prelude {
    use crate::car;
    use crate::wheel;

    pub use car::*;
    pub use wheel::*;

    use avian3d::prelude::*;
    use bevy::prelude::*;

    #[derive(Default)]
    pub struct BevyVehiclesPlugin {}

    impl Plugin for BevyVehiclesPlugin {
        fn build(&self, app: &mut App) {
            app.add_plugins(DefaultPlugins);
            app.add_plugins(PhysicsPlugins::default());
            app.add_plugins(PhysicsDebugPlugin::default());
        }
    }
}
