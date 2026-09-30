using BepuPhysics.Constraints;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuPhysics.Constraints.Contact;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.SweepTasks;

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
            solver.Register<TwistServo>();
            solver.Register<TwistLimit>();
            solver.Register<TwistMotor>();
            solver.Register<AngularServo>();
            solver.Register<AngularMotor>();
            solver.Register<Weld>();
            solver.Register<VolumeConstraint>();
            solver.Register<DistanceServo>();
            solver.Register<DistanceLimit>();
            solver.Register<CenterDistanceConstraint>();
            solver.Register<AreaConstraint>();
            solver.Register<PointOnLineServo>();
            solver.Register<LinearAxisServo>();
            solver.Register<LinearAxisMotor>();
            solver.Register<LinearAxisLimit>();
            solver.Register<AngularAxisMotor>();
            solver.Register<OneBodyAngularServo>();
            solver.Register<OneBodyAngularMotor>();
            solver.Register<OneBodyLinearServo>();
            solver.Register<OneBodyLinearMotor>();
            solver.Register<SwivelHinge>();
            solver.Register<Hinge>();
            solver.Register<BallSocketMotor>();
            solver.Register<BallSocketServo>();
            solver.Register<AngularAxisGearMotor>();
            solver.Register<CenterDistanceLimit>();

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
            //We may later use more contacts in the nonconvex manifold; for now we clamped it to 4.
            solver.Register<Contact2Nonconvex>();
            solver.Register<Contact3Nonconvex>();
            solver.Register<Contact4Nonconvex>();

            narrowPhase.RegisterContactConstraintAccessor(new NonconvexTwoBodyAccessor<Contact4Nonconvex, Contact4NonconvexPrestepData, Contact4NonconvexAccumulatedImpulses, ContactImpulses4>());
            narrowPhase.RegisterContactConstraintAccessor(new NonconvexTwoBodyAccessor<Contact3Nonconvex, Contact3NonconvexPrestepData, Contact3NonconvexAccumulatedImpulses, ContactImpulses3>());
            narrowPhase.RegisterContactConstraintAccessor(new NonconvexTwoBodyAccessor<Contact2Nonconvex, Contact2NonconvexPrestepData, Contact2NonconvexAccumulatedImpulses, ContactImpulses2>());

            narrowPhase.RegisterContactConstraintAccessor(new NonconvexOneBodyAccessor<Contact4NonconvexOneBody, Contact4NonconvexOneBodyPrestepData, Contact4NonconvexAccumulatedImpulses, ContactImpulses4>());
            narrowPhase.RegisterContactConstraintAccessor(new NonconvexOneBodyAccessor<Contact3NonconvexOneBody, Contact3NonconvexOneBodyPrestepData, Contact3NonconvexAccumulatedImpulses, ContactImpulses3>());
            narrowPhase.RegisterContactConstraintAccessor(new NonconvexOneBodyAccessor<Contact2NonconvexOneBody, Contact2NonconvexOneBodyPrestepData, Contact2NonconvexAccumulatedImpulses, ContactImpulses2>());

            narrowPhase.RegisterContactConstraintAccessor(new ConvexTwoBodyAccessor<Contact4, Contact4PrestepData, Contact4AccumulatedImpulses, ContactImpulses4>());
            narrowPhase.RegisterContactConstraintAccessor(new ConvexTwoBodyAccessor<Contact3, Contact3PrestepData, Contact3AccumulatedImpulses, ContactImpulses3>());
            narrowPhase.RegisterContactConstraintAccessor(new ConvexTwoBodyAccessor<Contact2, Contact2PrestepData, Contact2AccumulatedImpulses, ContactImpulses2>());
            narrowPhase.RegisterContactConstraintAccessor(new ConvexTwoBodyAccessor<Contact1, Contact1PrestepData, Contact1AccumulatedImpulses, ContactImpulses1>());
            narrowPhase.RegisterContactConstraintAccessor(new ConvexOneBodyAccessor<Contact4OneBody, Contact4OneBodyPrestepData, Contact4AccumulatedImpulses, ContactImpulses4>());
            narrowPhase.RegisterContactConstraintAccessor(new ConvexOneBodyAccessor<Contact3OneBody, Contact3OneBodyPrestepData, Contact3AccumulatedImpulses, ContactImpulses3>());
            narrowPhase.RegisterContactConstraintAccessor(new ConvexOneBodyAccessor<Contact2OneBody, Contact2OneBodyPrestepData, Contact2AccumulatedImpulses, ContactImpulses2>());
            narrowPhase.RegisterContactConstraintAccessor(new ConvexOneBodyAccessor<Contact1OneBody, Contact1OneBodyPrestepData, Contact1AccumulatedImpulses, ContactImpulses1>());

        }

        /// <summary>
        /// Creates a task registry containing the default collision pair types.
        /// </summary>
        public static CollisionTaskRegistry CreateDefaultCollisionTaskRegistry()
        {
            var defaultTaskRegistry = new CollisionTaskRegistry();
            defaultTaskRegistry.Register(new ConvexCollisionTask<Sphere, SphereWide, Sphere, SphereWide, SpherePair, SpherePairWide, Convex1ContactManifoldWide, SpherePairTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Sphere, SphereWide, Capsule, CapsuleWide, SphereIncludingPair, SphereIncludingPairWide<Capsule, CapsuleWide>, Convex1ContactManifoldWide, SphereCapsuleTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Sphere, SphereWide, Box, BoxWide, SphereIncludingPair, SphereIncludingPairWide<Box, BoxWide>, Convex1ContactManifoldWide, SphereBoxTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Sphere, SphereWide, Triangle, TriangleWide, SphereIncludingPair, SphereIncludingPairWide<Triangle, TriangleWide>, Convex1ContactManifoldWide, SphereTriangleTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Sphere, SphereWide, Cylinder, CylinderWide, SphereIncludingPair, SphereIncludingPairWide<Cylinder, CylinderWide>, Convex1ContactManifoldWide, SphereCylinderTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Sphere, SphereWide, Cone, ConeWide, SphereIncludingPair, SphereIncludingPairWide<Cone, ConeWide>, Convex1ContactManifoldWide, SphereConeTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Sphere, SphereWide, Pyramid, PyramidWide, SphereIncludingPair, SphereIncludingPairWide<Pyramid, PyramidWide>, Convex1ContactManifoldWide, SpherePyramidTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Sphere, SphereWide, Wedge, WedgeWide, SphereIncludingPair, SphereIncludingPairWide<Wedge, WedgeWide>, Convex1ContactManifoldWide, SphereWedgeTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Sphere, SphereWide, Rectangle, RectangleWide, SphereIncludingPair, SphereIncludingPairWide<Rectangle, RectangleWide>, Convex1ContactManifoldWide, SphereRectangleTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Sphere, SphereWide, ConvexHull, ConvexHullWide, SphereIncludingPair, SphereIncludingPairWide<ConvexHull, ConvexHullWide>, Convex1ContactManifoldWide, SphereConvexHullTester>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Sphere, Compound, ConvexCompoundOverlapFinder<Sphere, SphereWide, Compound>, ConvexCompoundContinuations<Compound>, NonconvexReduction>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Sphere, BigCompound, ConvexCompoundOverlapFinder<Sphere, SphereWide, BigCompound>, ConvexCompoundContinuations<BigCompound>, NonconvexReduction>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Sphere, Mesh, ConvexCompoundOverlapFinder<Sphere, SphereWide, Mesh>, ConvexMeshContinuations<Mesh>, MeshReduction>());

            defaultTaskRegistry.Register(new ConvexCollisionTask<Capsule, CapsuleWide, Capsule, CapsuleWide, FliplessPair, FliplessPairWide<Capsule, CapsuleWide>, Convex2ContactManifoldWide, CapsulePairTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Capsule, CapsuleWide, Box, BoxWide, CollisionPair, ConvexPairWide<Capsule, CapsuleWide, Box, BoxWide>, Convex2ContactManifoldWide, CapsuleBoxTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Capsule, CapsuleWide, Triangle, TriangleWide, CollisionPair, ConvexPairWide<Capsule, CapsuleWide, Triangle, TriangleWide>, Convex2ContactManifoldWide, CapsuleTriangleTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Capsule, CapsuleWide, Cylinder, CylinderWide, CollisionPair, ConvexPairWide<Capsule, CapsuleWide, Cylinder, CylinderWide>, Convex2ContactManifoldWide, CapsuleCylinderTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Capsule, CapsuleWide, Cone, ConeWide, CollisionPair, ConvexPairWide<Capsule, CapsuleWide, Cone, ConeWide>, Convex2ContactManifoldWide, CapsuleConeTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Capsule, CapsuleWide, Pyramid, PyramidWide, CollisionPair, ConvexPairWide<Capsule, CapsuleWide, Pyramid, PyramidWide>, Convex2ContactManifoldWide, CapsulePyramidTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Capsule, CapsuleWide, Wedge, WedgeWide, CollisionPair, ConvexPairWide<Capsule, CapsuleWide, Wedge, WedgeWide>, Convex2ContactManifoldWide, CapsuleWedgeTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Capsule, CapsuleWide, Rectangle, RectangleWide, CollisionPair, ConvexPairWide<Capsule, CapsuleWide, Rectangle, RectangleWide>, Convex2ContactManifoldWide, CapsuleRectangleTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Capsule, CapsuleWide, ConvexHull, ConvexHullWide, CollisionPair, ConvexPairWide<Capsule, CapsuleWide, ConvexHull, ConvexHullWide>, Convex2ContactManifoldWide, CapsuleConvexHullTester>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Capsule, Compound, ConvexCompoundOverlapFinder<Capsule, CapsuleWide, Compound>, ConvexCompoundContinuations<Compound>, NonconvexReduction>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Capsule, BigCompound, ConvexCompoundOverlapFinder<Capsule, CapsuleWide, BigCompound>, ConvexCompoundContinuations<BigCompound>, NonconvexReduction>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Capsule, Mesh, ConvexCompoundOverlapFinder<Capsule, CapsuleWide, Mesh>, ConvexMeshContinuations<Mesh>, MeshReduction>());

            defaultTaskRegistry.Register(new ConvexCollisionTask<Box, BoxWide, Box, BoxWide, FliplessPair, FliplessPairWide<Box, BoxWide>, Convex4ContactManifoldWide, BoxPairTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Box, BoxWide, Triangle, TriangleWide, CollisionPair, ConvexPairWide<Box, BoxWide, Triangle, TriangleWide>, Convex4ContactManifoldWide, BoxTriangleTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Box, BoxWide, Cylinder, CylinderWide, CollisionPair, ConvexPairWide<Box, BoxWide, Cylinder, CylinderWide>, Convex4ContactManifoldWide, BoxCylinderTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Box, BoxWide, Cone, ConeWide, CollisionPair, ConvexPairWide<Box, BoxWide, Cone, ConeWide>, Convex4ContactManifoldWide, BoxConeTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Box, BoxWide, Pyramid, PyramidWide, CollisionPair, ConvexPairWide<Box, BoxWide, Pyramid, PyramidWide>, Convex4ContactManifoldWide, BoxPyramidTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Box, BoxWide, Wedge, WedgeWide, CollisionPair, ConvexPairWide<Box, BoxWide, Wedge, WedgeWide>, Convex4ContactManifoldWide, BoxWedgeTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Box, BoxWide, Rectangle, RectangleWide, CollisionPair, ConvexPairWide<Box, BoxWide, Rectangle, RectangleWide>, Convex4ContactManifoldWide, BoxRectangleTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Box, BoxWide, ConvexHull, ConvexHullWide, CollisionPair, ConvexPairWide<Box, BoxWide, ConvexHull, ConvexHullWide>, Convex4ContactManifoldWide, BoxConvexHullTester>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Box, Compound, ConvexCompoundOverlapFinder<Box, BoxWide, Compound>, ConvexCompoundContinuations<Compound>, NonconvexReduction>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Box, BigCompound, ConvexCompoundOverlapFinder<Box, BoxWide, BigCompound>, ConvexCompoundContinuations<BigCompound>, NonconvexReduction>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Box, Mesh, ConvexCompoundOverlapFinder<Box, BoxWide, Mesh>, ConvexMeshContinuations<Mesh>, MeshReduction>());

            defaultTaskRegistry.Register(new ConvexCollisionTask<Triangle, TriangleWide, Triangle, TriangleWide, FliplessPair, FliplessPairWide<Triangle, TriangleWide>, Convex4ContactManifoldWide, TrianglePairTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Triangle, TriangleWide, Cylinder, CylinderWide, CollisionPair, ConvexPairWide<Triangle, TriangleWide, Cylinder, CylinderWide>, Convex4ContactManifoldWide, TriangleCylinderTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Triangle, TriangleWide, Cone, ConeWide, CollisionPair, ConvexPairWide<Triangle, TriangleWide, Cone, ConeWide>, Convex4ContactManifoldWide, TriangleConeTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Triangle, TriangleWide, Pyramid, PyramidWide, CollisionPair, ConvexPairWide<Triangle, TriangleWide, Pyramid, PyramidWide>, Convex4ContactManifoldWide, TrianglePyramidTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Triangle, TriangleWide, Wedge, WedgeWide, CollisionPair, ConvexPairWide<Triangle, TriangleWide, Wedge, WedgeWide>, Convex4ContactManifoldWide, TriangleWedgeTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Triangle, TriangleWide, Rectangle, RectangleWide, CollisionPair, ConvexPairWide<Triangle, TriangleWide, Rectangle, RectangleWide>, Convex4ContactManifoldWide, TriangleRectangleTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Triangle, TriangleWide, ConvexHull, ConvexHullWide, CollisionPair, ConvexPairWide<Triangle, TriangleWide, ConvexHull, ConvexHullWide>, Convex4ContactManifoldWide, TriangleConvexHullTester>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Triangle, Compound, ConvexCompoundOverlapFinder<Triangle, TriangleWide, Compound>, ConvexCompoundContinuations<Compound>, NonconvexReduction>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Triangle, BigCompound, ConvexCompoundOverlapFinder<Triangle, TriangleWide, BigCompound>, ConvexCompoundContinuations<BigCompound>, NonconvexReduction>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Triangle, Mesh, ConvexCompoundOverlapFinder<Triangle, TriangleWide, Mesh>, ConvexMeshContinuations<Mesh>, MeshReduction>());

            defaultTaskRegistry.Register(new ConvexCollisionTask<Cylinder, CylinderWide, Cylinder, CylinderWide, FliplessPair, FliplessPairWide<Cylinder, CylinderWide>, Convex4ContactManifoldWide, CylinderPairTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Cylinder, CylinderWide, Cone, ConeWide, CollisionPair, ConvexPairWide<Cylinder, CylinderWide, Cone, ConeWide>, Convex4ContactManifoldWide, CylinderConeTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Cylinder, CylinderWide, Pyramid, PyramidWide, CollisionPair, ConvexPairWide<Cylinder, CylinderWide, Pyramid, PyramidWide>, Convex4ContactManifoldWide, CylinderPyramidTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Cylinder, CylinderWide, Wedge, WedgeWide, CollisionPair, ConvexPairWide<Cylinder, CylinderWide, Wedge, WedgeWide>, Convex4ContactManifoldWide, CylinderWedgeTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Cylinder, CylinderWide, Rectangle, RectangleWide, CollisionPair, ConvexPairWide<Cylinder, CylinderWide, Rectangle, RectangleWide>, Convex4ContactManifoldWide, CylinderRectangleTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Cylinder, CylinderWide, ConvexHull, ConvexHullWide, CollisionPair, ConvexPairWide<Cylinder, CylinderWide, ConvexHull, ConvexHullWide>, Convex4ContactManifoldWide, CylinderConvexHullTester>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Cylinder, Compound, ConvexCompoundOverlapFinder<Cylinder, CylinderWide, Compound>, ConvexCompoundContinuations<Compound>, NonconvexReduction>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Cylinder, BigCompound, ConvexCompoundOverlapFinder<Cylinder, CylinderWide, BigCompound>, ConvexCompoundContinuations<BigCompound>, NonconvexReduction>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Cylinder, Mesh, ConvexCompoundOverlapFinder<Cylinder, CylinderWide, Mesh>, ConvexMeshContinuations<Mesh>, MeshReduction>());

            defaultTaskRegistry.Register(new ConvexCollisionTask<Cone, ConeWide, Cone, ConeWide, FliplessPair, FliplessPairWide<Cone, ConeWide>, Convex4ContactManifoldWide, ConePairTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Cone, ConeWide, Pyramid, PyramidWide, CollisionPair, ConvexPairWide<Cone, ConeWide, Pyramid, PyramidWide>, Convex4ContactManifoldWide, ConePyramidTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Cone, ConeWide, Wedge, WedgeWide, CollisionPair, ConvexPairWide<Cone, ConeWide, Wedge, WedgeWide>, Convex4ContactManifoldWide, ConeWedgeTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Cone, ConeWide, Rectangle, RectangleWide, CollisionPair, ConvexPairWide<Cone, ConeWide, Rectangle, RectangleWide>, Convex4ContactManifoldWide, ConeRectangleTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Cone, ConeWide, ConvexHull, ConvexHullWide, CollisionPair, ConvexPairWide<Cone, ConeWide, ConvexHull, ConvexHullWide>, Convex4ContactManifoldWide, ConeConvexHullTester>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Cone, Compound, ConvexCompoundOverlapFinder<Cone, ConeWide, Compound>, ConvexCompoundContinuations<Compound>, NonconvexReduction>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Cone, BigCompound, ConvexCompoundOverlapFinder<Cone, ConeWide, BigCompound>, ConvexCompoundContinuations<BigCompound>, NonconvexReduction>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Cone, Mesh, ConvexCompoundOverlapFinder<Cone, ConeWide, Mesh>, ConvexMeshContinuations<Mesh>, MeshReduction>());

            defaultTaskRegistry.Register(new ConvexCollisionTask<Pyramid, PyramidWide, Pyramid, PyramidWide, FliplessPair, FliplessPairWide<Pyramid, PyramidWide>, Convex4ContactManifoldWide, PyramidPairTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Pyramid, PyramidWide, Wedge, WedgeWide, CollisionPair, ConvexPairWide<Pyramid, PyramidWide, Wedge, WedgeWide>, Convex4ContactManifoldWide, PyramidWedgeTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Pyramid, PyramidWide, Rectangle, RectangleWide, CollisionPair, ConvexPairWide<Pyramid, PyramidWide, Rectangle, RectangleWide>, Convex4ContactManifoldWide, PyramidRectangleTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Pyramid, PyramidWide, ConvexHull, ConvexHullWide, CollisionPair, ConvexPairWide<Pyramid, PyramidWide, ConvexHull, ConvexHullWide>, Convex4ContactManifoldWide, PyramidConvexHullTester>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Pyramid, Compound, ConvexCompoundOverlapFinder<Pyramid, PyramidWide, Compound>, ConvexCompoundContinuations<Compound>, NonconvexReduction>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Pyramid, BigCompound, ConvexCompoundOverlapFinder<Pyramid, PyramidWide, BigCompound>, ConvexCompoundContinuations<BigCompound>, NonconvexReduction>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Pyramid, Mesh, ConvexCompoundOverlapFinder<Pyramid, PyramidWide, Mesh>, ConvexMeshContinuations<Mesh>, MeshReduction>());

            defaultTaskRegistry.Register(new ConvexCollisionTask<Wedge, WedgeWide, Wedge, WedgeWide, FliplessPair, FliplessPairWide<Wedge, WedgeWide>, Convex4ContactManifoldWide, WedgePairTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Wedge, WedgeWide, Rectangle, RectangleWide, CollisionPair, ConvexPairWide<Wedge, WedgeWide, Rectangle, RectangleWide>, Convex4ContactManifoldWide, WedgeRectangleTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Wedge, WedgeWide, ConvexHull, ConvexHullWide, CollisionPair, ConvexPairWide<Wedge, WedgeWide, ConvexHull, ConvexHullWide>, Convex4ContactManifoldWide, WedgeConvexHullTester>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Wedge, Compound, ConvexCompoundOverlapFinder<Wedge, WedgeWide, Compound>, ConvexCompoundContinuations<Compound>, NonconvexReduction>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Wedge, BigCompound, ConvexCompoundOverlapFinder<Wedge, WedgeWide, BigCompound>, ConvexCompoundContinuations<BigCompound>, NonconvexReduction>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Wedge, Mesh, ConvexCompoundOverlapFinder<Wedge, WedgeWide, Mesh>, ConvexMeshContinuations<Mesh>, MeshReduction>());

            defaultTaskRegistry.Register(new ConvexCollisionTask<Rectangle, RectangleWide, Rectangle, RectangleWide, FliplessPair, FliplessPairWide<Rectangle, RectangleWide>, Convex4ContactManifoldWide, RectanglePairTester>());
            defaultTaskRegistry.Register(new ConvexCollisionTask<Rectangle, RectangleWide, ConvexHull, ConvexHullWide, CollisionPair, ConvexPairWide<Rectangle, RectangleWide, ConvexHull, ConvexHullWide>, Convex4ContactManifoldWide, RectangleConvexHullTester>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Rectangle, Compound, ConvexCompoundOverlapFinder<Rectangle, RectangleWide, Compound>, ConvexCompoundContinuations<Compound>, NonconvexReduction>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Rectangle, BigCompound, ConvexCompoundOverlapFinder<Rectangle, RectangleWide, BigCompound>, ConvexCompoundContinuations<BigCompound>, NonconvexReduction>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<Rectangle, Mesh, ConvexCompoundOverlapFinder<Rectangle, RectangleWide, Mesh>, ConvexMeshContinuations<Mesh>, MeshReduction>());

            defaultTaskRegistry.Register(new ConvexCollisionTask<ConvexHull, ConvexHullWide, ConvexHull, ConvexHullWide, FliplessPair, FliplessPairWide<ConvexHull, ConvexHullWide>, Convex4ContactManifoldWide, ConvexHullPairTester>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<ConvexHull, Compound, ConvexCompoundOverlapFinder<ConvexHull, ConvexHullWide, Compound>, ConvexCompoundContinuations<Compound>, NonconvexReduction>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<ConvexHull, BigCompound, ConvexCompoundOverlapFinder<ConvexHull, ConvexHullWide, BigCompound>, ConvexCompoundContinuations<BigCompound>, NonconvexReduction>());
            defaultTaskRegistry.Register(new ConvexCompoundCollisionTask<ConvexHull, Mesh, ConvexCompoundOverlapFinder<ConvexHull, ConvexHullWide, Mesh>, ConvexMeshContinuations<Mesh>, MeshReduction>());

            defaultTaskRegistry.Register(new CompoundPairCollisionTask<Compound, Compound, CompoundPairOverlapFinder<Compound, Compound>, CompoundPairContinuations<Compound, Compound>, NonconvexReduction>());
            defaultTaskRegistry.Register(new CompoundPairCollisionTask<Compound, BigCompound, CompoundPairOverlapFinder<Compound, BigCompound>, CompoundPairContinuations<Compound, BigCompound>, NonconvexReduction>());
            defaultTaskRegistry.Register(new CompoundPairCollisionTask<Compound, Mesh, CompoundPairOverlapFinder<Compound, Mesh>, CompoundMeshContinuations<Compound, Mesh>, CompoundMeshReduction>());

            defaultTaskRegistry.Register(new CompoundPairCollisionTask<BigCompound, BigCompound, CompoundPairOverlapFinder<BigCompound, BigCompound>, CompoundPairContinuations<BigCompound, BigCompound>, NonconvexReduction>());
            defaultTaskRegistry.Register(new CompoundPairCollisionTask<BigCompound, Mesh, CompoundPairOverlapFinder<BigCompound, Mesh>, CompoundMeshContinuations<BigCompound, Mesh>, CompoundMeshReduction>());

            defaultTaskRegistry.Register(new CompoundPairCollisionTask<Mesh, Mesh, MeshPairOverlapFinder<Mesh, Mesh>, MeshPairContinuations<Mesh, Mesh>, CompoundMeshReduction>());

            return defaultTaskRegistry;
        }

        /// <summary>
        /// Creates a task registry containing the default sweep task types.
        /// </summary>
        public static SweepTaskRegistry CreateDefaultSweepTaskRegistry()
        {
            var defaultTaskRegistry = new SweepTaskRegistry();
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Sphere, SphereWide, Sphere, SphereWide, SpherePairDistanceTester>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Sphere, SphereWide, Capsule, CapsuleWide, SphereCapsuleDistanceTester>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Sphere, SphereWide, Box, BoxWide, SphereBoxDistanceTester>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Sphere, SphereWide, Triangle, TriangleWide, SphereTriangleDistanceTester>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Sphere, SphereWide, Cylinder, CylinderWide, SphereCylinderDistanceTester>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Sphere, SphereWide, Cone, ConeWide, GJKDistanceTester<Sphere, SphereWide, SphereSupportFinder, Cone, ConeWide, ConeSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Sphere, SphereWide, Pyramid, PyramidWide, GJKDistanceTester<Sphere, SphereWide, SphereSupportFinder, Pyramid, PyramidWide, PyramidSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Sphere, SphereWide, Wedge, WedgeWide, GJKDistanceTester<Sphere, SphereWide, SphereSupportFinder, Wedge, WedgeWide, WedgeSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Sphere, SphereWide, Rectangle, RectangleWide, GJKDistanceTester<Sphere, SphereWide, SphereSupportFinder, Rectangle, RectangleWide, RectangleSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Sphere, SphereWide, ConvexHull, ConvexHullWide, GJKDistanceTester<Sphere, SphereWide, SphereSupportFinder, ConvexHull, ConvexHullWide, ConvexHullSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexCompoundSweepTask<Sphere, SphereWide, Compound, ConvexCompoundSweepOverlapFinder<Sphere, Compound>>());
            defaultTaskRegistry.Register(new ConvexCompoundSweepTask<Sphere, SphereWide, BigCompound, ConvexCompoundSweepOverlapFinder<Sphere, BigCompound>>());
            defaultTaskRegistry.Register(new ConvexHomogeneousCompoundSweepTask<Sphere, SphereWide, Mesh, Triangle, TriangleWide, ConvexCompoundSweepOverlapFinder<Sphere, Mesh>>());

            defaultTaskRegistry.Register(new ConvexPairSweepTask<Capsule, CapsuleWide, Capsule, CapsuleWide, CapsulePairDistanceTester>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Capsule, CapsuleWide, Box, BoxWide, CapsuleBoxDistanceTester>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Capsule, CapsuleWide, Triangle, TriangleWide, GJKDistanceTester<Capsule, CapsuleWide, CapsuleSupportFinder, Triangle, TriangleWide, TriangleSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Capsule, CapsuleWide, Cylinder, CylinderWide, GJKDistanceTester<Capsule, CapsuleWide, CapsuleSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Capsule, CapsuleWide, Cone, ConeWide, GJKDistanceTester<Capsule, CapsuleWide, CapsuleSupportFinder, Cone, ConeWide, ConeSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Capsule, CapsuleWide, Pyramid, PyramidWide, GJKDistanceTester<Capsule, CapsuleWide, CapsuleSupportFinder, Pyramid, PyramidWide, PyramidSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Capsule, CapsuleWide, Wedge, WedgeWide, GJKDistanceTester<Capsule, CapsuleWide, CapsuleSupportFinder, Wedge, WedgeWide, WedgeSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Capsule, CapsuleWide, Rectangle, RectangleWide, GJKDistanceTester<Capsule, CapsuleWide, CapsuleSupportFinder, Rectangle, RectangleWide, RectangleSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Capsule, CapsuleWide, ConvexHull, ConvexHullWide, GJKDistanceTester<Capsule, CapsuleWide, CapsuleSupportFinder, ConvexHull, ConvexHullWide, ConvexHullSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexCompoundSweepTask<Capsule, CapsuleWide, Compound, ConvexCompoundSweepOverlapFinder<Capsule, Compound>>());
            defaultTaskRegistry.Register(new ConvexCompoundSweepTask<Capsule, CapsuleWide, BigCompound, ConvexCompoundSweepOverlapFinder<Capsule, BigCompound>>());
            defaultTaskRegistry.Register(new ConvexHomogeneousCompoundSweepTask<Capsule, CapsuleWide, Mesh, Triangle, TriangleWide, ConvexCompoundSweepOverlapFinder<Capsule, Mesh>>());

            defaultTaskRegistry.Register(new ConvexPairSweepTask<Box, BoxWide, Box, BoxWide, GJKDistanceTester<Box, BoxWide, BoxSupportFinder, Box, BoxWide, BoxSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Box, BoxWide, Triangle, TriangleWide, GJKDistanceTester<Box, BoxWide, BoxSupportFinder, Triangle, TriangleWide, TriangleSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Box, BoxWide, Cylinder, CylinderWide, GJKDistanceTester<Box, BoxWide, BoxSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Box, BoxWide, Cone, ConeWide, GJKDistanceTester<Box, BoxWide, BoxSupportFinder, Cone, ConeWide, ConeSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Box, BoxWide, Pyramid, PyramidWide, GJKDistanceTester<Box, BoxWide, BoxSupportFinder, Pyramid, PyramidWide, PyramidSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Box, BoxWide, Wedge, WedgeWide, GJKDistanceTester<Box, BoxWide, BoxSupportFinder, Wedge, WedgeWide, WedgeSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Box, BoxWide, Rectangle, RectangleWide, GJKDistanceTester<Box, BoxWide, BoxSupportFinder, Rectangle, RectangleWide, RectangleSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Box, BoxWide, ConvexHull, ConvexHullWide, GJKDistanceTester<Box, BoxWide, BoxSupportFinder, ConvexHull, ConvexHullWide, ConvexHullSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexCompoundSweepTask<Box, BoxWide, Compound, ConvexCompoundSweepOverlapFinder<Box, Compound>>());
            defaultTaskRegistry.Register(new ConvexCompoundSweepTask<Box, BoxWide, BigCompound, ConvexCompoundSweepOverlapFinder<Box, BigCompound>>());
            defaultTaskRegistry.Register(new ConvexHomogeneousCompoundSweepTask<Box, BoxWide, Mesh, Triangle, TriangleWide, ConvexCompoundSweepOverlapFinder<Box, Mesh>>());

            defaultTaskRegistry.Register(new ConvexPairSweepTask<Triangle, TriangleWide, Triangle, TriangleWide, GJKDistanceTester<Triangle, TriangleWide, TriangleSupportFinder, Triangle, TriangleWide, TriangleSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Triangle, TriangleWide, Cylinder, CylinderWide, GJKDistanceTester<Triangle, TriangleWide, TriangleSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Triangle, TriangleWide, Cone, ConeWide, GJKDistanceTester<Triangle, TriangleWide, TriangleSupportFinder, Cone, ConeWide, ConeSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Triangle, TriangleWide, Pyramid, PyramidWide, GJKDistanceTester<Triangle, TriangleWide, TriangleSupportFinder, Pyramid, PyramidWide, PyramidSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Triangle, TriangleWide, Wedge, WedgeWide, GJKDistanceTester<Triangle, TriangleWide, TriangleSupportFinder, Wedge, WedgeWide, WedgeSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Triangle, TriangleWide, Rectangle, RectangleWide, GJKDistanceTester<Triangle, TriangleWide, TriangleSupportFinder, Rectangle, RectangleWide, RectangleSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Triangle, TriangleWide, ConvexHull, ConvexHullWide, GJKDistanceTester<Triangle, TriangleWide, TriangleSupportFinder, ConvexHull, ConvexHullWide, ConvexHullSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexCompoundSweepTask<Triangle, TriangleWide, Compound, ConvexCompoundSweepOverlapFinder<Triangle, Compound>>());
            defaultTaskRegistry.Register(new ConvexCompoundSweepTask<Triangle, TriangleWide, BigCompound, ConvexCompoundSweepOverlapFinder<Triangle, BigCompound>>());
            defaultTaskRegistry.Register(new ConvexHomogeneousCompoundSweepTask<Triangle, TriangleWide, Mesh, Triangle, TriangleWide, ConvexCompoundSweepOverlapFinder<Triangle, Mesh>>());

            defaultTaskRegistry.Register(new ConvexPairSweepTask<Cylinder, CylinderWide, Cylinder, CylinderWide, GJKDistanceTester<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Cylinder, CylinderWide, Cone, ConeWide, GJKDistanceTester<Cylinder, CylinderWide, CylinderSupportFinder, Cone, ConeWide, ConeSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Cylinder, CylinderWide, Pyramid, PyramidWide, GJKDistanceTester<Cylinder, CylinderWide, CylinderSupportFinder, Pyramid, PyramidWide, PyramidSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Cylinder, CylinderWide, Wedge, WedgeWide, GJKDistanceTester<Cylinder, CylinderWide, CylinderSupportFinder, Wedge, WedgeWide, WedgeSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Cylinder, CylinderWide, Rectangle, RectangleWide, GJKDistanceTester<Cylinder, CylinderWide, CylinderSupportFinder, Rectangle, RectangleWide, RectangleSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Cylinder, CylinderWide, ConvexHull, ConvexHullWide, GJKDistanceTester<Cylinder, CylinderWide, CylinderSupportFinder, ConvexHull, ConvexHullWide, ConvexHullSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexCompoundSweepTask<Cylinder, CylinderWide, Compound, ConvexCompoundSweepOverlapFinder<Cylinder, Compound>>());
            defaultTaskRegistry.Register(new ConvexCompoundSweepTask<Cylinder, CylinderWide, BigCompound, ConvexCompoundSweepOverlapFinder<Cylinder, BigCompound>>());
            defaultTaskRegistry.Register(new ConvexHomogeneousCompoundSweepTask<Cylinder, CylinderWide, Mesh, Triangle, TriangleWide, ConvexCompoundSweepOverlapFinder<Cylinder, Mesh>>());

            defaultTaskRegistry.Register(new ConvexPairSweepTask<Cone, ConeWide, Cone, ConeWide, GJKDistanceTester<Cone, ConeWide, ConeSupportFinder, Cone, ConeWide, ConeSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Cone, ConeWide, Pyramid, PyramidWide, GJKDistanceTester<Cone, ConeWide, ConeSupportFinder, Pyramid, PyramidWide, PyramidSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Cone, ConeWide, Wedge, WedgeWide, GJKDistanceTester<Cone, ConeWide, ConeSupportFinder, Wedge, WedgeWide, WedgeSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Cone, ConeWide, Rectangle, RectangleWide, GJKDistanceTester<Cone, ConeWide, ConeSupportFinder, Rectangle, RectangleWide, RectangleSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Cone, ConeWide, ConvexHull, ConvexHullWide, GJKDistanceTester<Cone, ConeWide, ConeSupportFinder, ConvexHull, ConvexHullWide, ConvexHullSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexCompoundSweepTask<Cone, ConeWide, Compound, ConvexCompoundSweepOverlapFinder<Cone, Compound>>());
            defaultTaskRegistry.Register(new ConvexCompoundSweepTask<Cone, ConeWide, BigCompound, ConvexCompoundSweepOverlapFinder<Cone, BigCompound>>());
            defaultTaskRegistry.Register(new ConvexHomogeneousCompoundSweepTask<Cone, ConeWide, Mesh, Triangle, TriangleWide, ConvexCompoundSweepOverlapFinder<Cone, Mesh>>());

            defaultTaskRegistry.Register(new ConvexPairSweepTask<Pyramid, PyramidWide, Pyramid, PyramidWide, GJKDistanceTester<Pyramid, PyramidWide, PyramidSupportFinder, Pyramid, PyramidWide, PyramidSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Pyramid, PyramidWide, Wedge, WedgeWide, GJKDistanceTester<Pyramid, PyramidWide, PyramidSupportFinder, Wedge, WedgeWide, WedgeSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Pyramid, PyramidWide, Rectangle, RectangleWide, GJKDistanceTester<Pyramid, PyramidWide, PyramidSupportFinder, Rectangle, RectangleWide, RectangleSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Pyramid, PyramidWide, ConvexHull, ConvexHullWide, GJKDistanceTester<Pyramid, PyramidWide, PyramidSupportFinder, ConvexHull, ConvexHullWide, ConvexHullSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexCompoundSweepTask<Pyramid, PyramidWide, Compound, ConvexCompoundSweepOverlapFinder<Pyramid, Compound>>());
            defaultTaskRegistry.Register(new ConvexCompoundSweepTask<Pyramid, PyramidWide, BigCompound, ConvexCompoundSweepOverlapFinder<Pyramid, BigCompound>>());
            defaultTaskRegistry.Register(new ConvexHomogeneousCompoundSweepTask<Pyramid, PyramidWide, Mesh, Triangle, TriangleWide, ConvexCompoundSweepOverlapFinder<Pyramid, Mesh>>());

            defaultTaskRegistry.Register(new ConvexPairSweepTask<Wedge, WedgeWide, Wedge, WedgeWide, GJKDistanceTester<Wedge, WedgeWide, WedgeSupportFinder, Wedge, WedgeWide, WedgeSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Wedge, WedgeWide, Rectangle, RectangleWide, GJKDistanceTester<Wedge, WedgeWide, WedgeSupportFinder, Rectangle, RectangleWide, RectangleSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Wedge, WedgeWide, ConvexHull, ConvexHullWide, GJKDistanceTester<Wedge, WedgeWide, WedgeSupportFinder, ConvexHull, ConvexHullWide, ConvexHullSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexCompoundSweepTask<Wedge, WedgeWide, Compound, ConvexCompoundSweepOverlapFinder<Wedge, Compound>>());
            defaultTaskRegistry.Register(new ConvexCompoundSweepTask<Wedge, WedgeWide, BigCompound, ConvexCompoundSweepOverlapFinder<Wedge, BigCompound>>());
            defaultTaskRegistry.Register(new ConvexHomogeneousCompoundSweepTask<Wedge, WedgeWide, Mesh, Triangle, TriangleWide, ConvexCompoundSweepOverlapFinder<Wedge, Mesh>>());

            defaultTaskRegistry.Register(new ConvexPairSweepTask<Rectangle, RectangleWide, Rectangle, RectangleWide, GJKDistanceTester<Rectangle, RectangleWide, RectangleSupportFinder, Rectangle, RectangleWide, RectangleSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexPairSweepTask<Rectangle, RectangleWide, ConvexHull, ConvexHullWide, GJKDistanceTester<Rectangle, RectangleWide, RectangleSupportFinder, ConvexHull, ConvexHullWide, ConvexHullSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexCompoundSweepTask<Rectangle, RectangleWide, Compound, ConvexCompoundSweepOverlapFinder<Rectangle, Compound>>());
            defaultTaskRegistry.Register(new ConvexCompoundSweepTask<Rectangle, RectangleWide, BigCompound, ConvexCompoundSweepOverlapFinder<Rectangle, BigCompound>>());
            defaultTaskRegistry.Register(new ConvexHomogeneousCompoundSweepTask<Rectangle, RectangleWide, Mesh, Triangle, TriangleWide, ConvexCompoundSweepOverlapFinder<Rectangle, Mesh>>());

            defaultTaskRegistry.Register(new ConvexPairSweepTask<ConvexHull, ConvexHullWide, ConvexHull, ConvexHullWide, GJKDistanceTester<ConvexHull, ConvexHullWide, ConvexHullSupportFinder, ConvexHull, ConvexHullWide, ConvexHullSupportFinder>>());
            defaultTaskRegistry.Register(new ConvexCompoundSweepTask<ConvexHull, ConvexHullWide, Compound, ConvexCompoundSweepOverlapFinder<ConvexHull, Compound>>());
            defaultTaskRegistry.Register(new ConvexCompoundSweepTask<ConvexHull, ConvexHullWide, BigCompound, ConvexCompoundSweepOverlapFinder<ConvexHull, BigCompound>>());
            defaultTaskRegistry.Register(new ConvexHomogeneousCompoundSweepTask<ConvexHull, ConvexHullWide, Mesh, Triangle, TriangleWide, ConvexCompoundSweepOverlapFinder<ConvexHull, Mesh>>());

            defaultTaskRegistry.Register(new CompoundPairSweepTask<Compound, Compound, CompoundPairSweepOverlapFinder<Compound, Compound>>());
            defaultTaskRegistry.Register(new CompoundPairSweepTask<Compound, BigCompound, CompoundPairSweepOverlapFinder<Compound, BigCompound>>());
            defaultTaskRegistry.Register(new CompoundHomogeneousCompoundSweepTask<Compound, Mesh, Triangle, TriangleWide, CompoundPairSweepOverlapFinder<Compound, Mesh>>());

            defaultTaskRegistry.Register(new CompoundPairSweepTask<BigCompound, BigCompound, CompoundPairSweepOverlapFinder<BigCompound, BigCompound>>());
            defaultTaskRegistry.Register(new CompoundHomogeneousCompoundSweepTask<BigCompound, Mesh, Triangle, TriangleWide, CompoundPairSweepOverlapFinder<BigCompound, Mesh>>());

            //TODO: No mesh-mesh at the moment.
            return defaultTaskRegistry;
        }
    }
}
