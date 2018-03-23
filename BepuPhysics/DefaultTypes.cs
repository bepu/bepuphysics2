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
            solver.Register<Contact2NonconvexOneBody>();
            solver.Register<Contact3NonconvexOneBody>();
            solver.Register<Contact4NonconvexOneBody>();
            solver.Register<Contact5NonconvexOneBody>();
            solver.Register<Contact6NonconvexOneBody>();
            solver.Register<Contact7NonconvexOneBody>();
            solver.Register<Contact8NonconvexOneBody>();
            solver.Register<Contact2Nonconvex>();
            solver.Register<Contact3Nonconvex>();
            solver.Register<Contact4Nonconvex>();
            solver.Register<Contact5Nonconvex>();
            solver.Register<Contact6Nonconvex>();
            solver.Register<Contact7Nonconvex>();
            solver.Register<Contact8Nonconvex>();

            narrowPhase.RegisterContactConstraintAccessor(new NonconvexTwoBodyAccessor<Contact8Nonconvex, Contact8NonconvexAccumulatedImpulses, ContactImpulses8, ConstraintCache8>());
            narrowPhase.RegisterContactConstraintAccessor(new NonconvexTwoBodyAccessor<Contact7Nonconvex, Contact7NonconvexAccumulatedImpulses, ContactImpulses7, ConstraintCache7>());
            narrowPhase.RegisterContactConstraintAccessor(new NonconvexTwoBodyAccessor<Contact6Nonconvex, Contact6NonconvexAccumulatedImpulses, ContactImpulses6, ConstraintCache6>());
            narrowPhase.RegisterContactConstraintAccessor(new NonconvexTwoBodyAccessor<Contact5Nonconvex, Contact5NonconvexAccumulatedImpulses, ContactImpulses5, ConstraintCache5>());
            narrowPhase.RegisterContactConstraintAccessor(new NonconvexTwoBodyAccessor<Contact4Nonconvex, Contact4NonconvexAccumulatedImpulses, ContactImpulses4, ConstraintCache4>());
            narrowPhase.RegisterContactConstraintAccessor(new NonconvexTwoBodyAccessor<Contact3Nonconvex, Contact3NonconvexAccumulatedImpulses, ContactImpulses3, ConstraintCache3>());
            narrowPhase.RegisterContactConstraintAccessor(new NonconvexTwoBodyAccessor<Contact2Nonconvex, Contact2NonconvexAccumulatedImpulses, ContactImpulses2, ConstraintCache2>());
            
            narrowPhase.RegisterContactConstraintAccessor(new NonconvexOneBodyAccessor<Contact8NonconvexOneBody, Contact8NonconvexAccumulatedImpulses, ContactImpulses8, ConstraintCache8>());
            narrowPhase.RegisterContactConstraintAccessor(new NonconvexOneBodyAccessor<Contact7NonconvexOneBody, Contact7NonconvexAccumulatedImpulses, ContactImpulses7, ConstraintCache7>());
            narrowPhase.RegisterContactConstraintAccessor(new NonconvexOneBodyAccessor<Contact6NonconvexOneBody, Contact6NonconvexAccumulatedImpulses, ContactImpulses6, ConstraintCache6>());
            narrowPhase.RegisterContactConstraintAccessor(new NonconvexOneBodyAccessor<Contact5NonconvexOneBody, Contact5NonconvexAccumulatedImpulses, ContactImpulses5, ConstraintCache5>());
            narrowPhase.RegisterContactConstraintAccessor(new NonconvexOneBodyAccessor<Contact4NonconvexOneBody, Contact4NonconvexAccumulatedImpulses, ContactImpulses4, ConstraintCache4>());
            narrowPhase.RegisterContactConstraintAccessor(new NonconvexOneBodyAccessor<Contact3NonconvexOneBody, Contact3NonconvexAccumulatedImpulses, ContactImpulses3, ConstraintCache3>());
            narrowPhase.RegisterContactConstraintAccessor(new NonconvexOneBodyAccessor<Contact2NonconvexOneBody, Contact2NonconvexAccumulatedImpulses, ContactImpulses2, ConstraintCache2>());

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
            defaultTaskRegistry.Register(new CompoundPairCollisionTask());
            return defaultTaskRegistry;
        }
    }
}
