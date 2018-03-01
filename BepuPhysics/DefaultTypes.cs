using BepuPhysics.Constraints;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuPhysics.Constraints.Contact;

namespace BepuPhysics
{
    /// <summary>
    /// Helper class to register the default types within a simulation instance.
    /// </summary>
    public static class DefaultTypes
    {
        /// <summary>
        /// Registers the set of constraints that are packaged in the engine.
        /// </summary>
        public static void RegisterDefaults(Solver solver, NarrowPhase narrowPhase)
        {
            solver.Register<BallSocket>();
            solver.Register<AngularHinge>();
            solver.Register<AngularSwivelHinge>();
            solver.Register<SwingLimit>();
            solver.Register<Contact1OneBody>();
            solver.Register<Contact2OneBody>();
            solver.Register<Contact3OneBody>();
            solver.Register<Contact4OneBody>();
            solver.Register<Contact1>();
            solver.Register<Contact2>();
            solver.Register<Contact3>();
            solver.Register<Contact4>();

            narrowPhase.RegisterContactConstraintAccessor(new Contact4Accessor());
            narrowPhase.RegisterContactConstraintAccessor(new Contact3Accessor());
            narrowPhase.RegisterContactConstraintAccessor(new Contact2Accessor());
            narrowPhase.RegisterContactConstraintAccessor(new Contact1Accessor());
            narrowPhase.RegisterContactConstraintAccessor(new Contact4OneBodyAccessor());
            narrowPhase.RegisterContactConstraintAccessor(new Contact3OneBodyAccessor());
            narrowPhase.RegisterContactConstraintAccessor(new Contact2OneBodyAccessor());
            narrowPhase.RegisterContactConstraintAccessor(new Contact1OneBodyAccessor());
            
        }

        /// <summary>
        /// Creates a task registry containing the default collision pair types.
        /// </summary>
        public static CollisionTaskRegistry CreateDefaultCollisionTaskRegistry()
        {
            var defaultTaskRegistry = new CollisionTaskRegistry();
            defaultTaskRegistry.Register(new SpherePairCollisionTask());
            defaultTaskRegistry.Register(new SphereCapsuleCollisionTask());
            defaultTaskRegistry.Register(new SphereBoxCollisionTask());
            defaultTaskRegistry.Register(new CapsulePairCollisionTask());
            defaultTaskRegistry.Register(new CapsuleBoxCollisionTask());
            defaultTaskRegistry.Register(new BoxPairCollisionTask());
            return defaultTaskRegistry;
        }
    }
}
