using BepuPhysics.Constraints;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuPhysics.Constraints.Contact;

namespace BepuPhysics
{
    /// <summary>
    /// Helper class to register the default types of constraints and shapes within a simulation instance.
    /// </summary>
    public static class DefaultTypes
    {
        /// <summary>
        /// Registers the set of shapes constraints that are packaged in the engine.
        /// </summary>
        public static void Register(Solver solver, out CollisionTaskRegistry defaultTaskRegistry)
        {
            solver.Register<BallSocket>();
            solver.Register<Contact1OneBody>();
            solver.Register<Contact2OneBody>();
            solver.Register<Contact1>();
            solver.Register<Contact2>();
            solver.Register<Contact4>();

            defaultTaskRegistry = new CollisionTaskRegistry();
            defaultTaskRegistry.Register(new SpherePairCollisionTask());
            defaultTaskRegistry.Register(new SphereCapsuleCollisionTask());
            defaultTaskRegistry.Register(new SphereBoxCollisionTask());
            defaultTaskRegistry.Register(new CapsulePairCollisionTask());
        }
    }
}
