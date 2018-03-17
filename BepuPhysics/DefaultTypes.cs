using BepuPhysics.Constraints;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuPhysics.Constraints.Contact;
using BepuPhysics.Collidables;

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

            narrowPhase.RegisterContactConstraintAccessor(new ConvexTwoBodyAccessor<Contact4, Contact4AccumulatedImpulses, ContactImpulses4, ConstraintCache4>());
            narrowPhase.RegisterContactConstraintAccessor(new ConvexTwoBodyAccessor<Contact3, Contact3AccumulatedImpulses, ContactImpulses3, ConstraintCache3>());
            narrowPhase.RegisterContactConstraintAccessor(new ConvexTwoBodyAccessor<Contact2, Contact2AccumulatedImpulses, ContactImpulses2, ConstraintCache2>());
            narrowPhase.RegisterContactConstraintAccessor(new ConvexTwoBodyAccessor<Contact1, Contact1AccumulatedImpulses, ContactImpulses1, ConstraintCache1>());
            narrowPhase.RegisterContactConstraintAccessor(new ConvexOneBodyAccessor<Contact4OneBody, Contact4AccumulatedImpulses, ContactImpulses4, ConstraintCache4>());
            narrowPhase.RegisterContactConstraintAccessor(new ConvexOneBodyAccessor<Contact3OneBody, Contact3AccumulatedImpulses, ContactImpulses3, ConstraintCache3>());
            narrowPhase.RegisterContactConstraintAccessor(new ConvexOneBodyAccessor<Contact2OneBody, Contact2AccumulatedImpulses, ContactImpulses2, ConstraintCache2>());
            narrowPhase.RegisterContactConstraintAccessor(new ConvexOneBodyAccessor<Contact1OneBody, Contact1AccumulatedImpulses, ContactImpulses1, ConstraintCache1>());

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
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Sphere>());
            defaultTaskRegistry.Register(new CapsulePairCollisionTask());
            defaultTaskRegistry.Register(new CapsuleBoxCollisionTask());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Capsule>());
            defaultTaskRegistry.Register(new BoxPairCollisionTask());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Box>());
            return defaultTaskRegistry;
        }
    }
}
