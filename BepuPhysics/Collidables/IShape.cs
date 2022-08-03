using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuPhysics.Trees;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Numerics;

namespace BepuPhysics.Collidables
{
    /// <summary>
    /// Defines a type usable as a shape by collidables.
    /// </summary>
    public interface IShape
    {
        //TODO: Note that these should really be *static* as they do not need any information about an instance, but static abstract interface methods are not yet out of preview.
        /// <summary>
        /// Unique type id for this shape type.
        /// </summary>
        int TypeId { get; }
        /// <summary>
        /// Creates a shape batch for this type of shape.
        /// </summary>
        /// <param name="pool">Buffer pool used to create the batch.</param>
        /// <param name="initialCapacity">Initial capacity to allocate within the batch.</param>
        /// <param name="shapeBatches">The set of shapes to contain this batch.</param>
        /// <returns>Shape batch for the shape type.</returns>
        /// <remarks>This is typically used internally to initialize new shape collections in response to shapes being added. It is not likely to be useful outside of the engine.
        /// Ideally, this would be implemented as a static abstract, but those aren't available yet.</remarks>
        ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapeBatches);
    }

    //Note that the following bounds functions require only an orientation because the effect of the position on the bounding box is the same for all shapes.
    //By isolating the shape from the position, we can more easily swap out the position representation for higher precision modes while only modifying the stuff that actually
    //deals with positions directly.

    //Note that we also support one-off bounds calculations. They are used even in the engine sometimes. Adding individual bodies to the simulation, for example.
    //Note, however, that we do not bother supporting velocity expansion on the one-off variant. For the purposes of adding objects to the simulation, that is basically irrelevant.
    //I don't predict ever needing it, but such an implementation could be added...

    /// <summary>
    /// Defines functions available on all convex shapes. Convex shapes have no hollowed out regions; any line passing through a convex shape will never enter and exit more than once.
    /// </summary>
    public interface IConvexShape : IShape
    {
        /// <summary>
        /// Computes the bounding box of a shape given an orientation.
        /// </summary>
        /// <param name="orientation">Orientation of the shape to use when computing the bounding box.</param>
        /// <param name="min">Minimum corner of the bounding box.</param>
        /// <param name="max">Maximum corner of the bounding box.</param>
        void ComputeBounds(Quaternion orientation, out Vector3 min, out Vector3 max);

        /// <summary>
        /// Computes information about how the bounding box should be expanded in response to angular velocity.
        /// </summary>
        /// <param name="maximumRadius"></param>
        /// <param name="maximumAngularExpansion"></param>
        /// <remarks>This is typically used in the engine for predicting bounding boxes at the beginning of the frame.
        /// Velocities are used to expand the bounding box so that likely future collisions will be detected.
        /// Linear velocity expands the bounding box in a direct and simple way, but angular expansion requires more information about the shape. 
        /// Imagine a long and thin capsule versus a sphere: high angular velocity may require significant expansion on the capsule, but spheres are rotationally invariant.</remarks>
        void ComputeAngularExpansionData(out float maximumRadius, out float maximumAngularExpansion);

        /// <summary>
        /// Computes the inertia for a body given a mass.
        /// </summary>
        /// <param name="mass">Mass to use to compute the body's inertia.</param>
        /// <returns>Inertia for the body.</returns>
        /// <remarks>Note that the <see cref="BodyInertia"/> returned by this stores the inverse mass and inverse inertia tensor. 
        /// This is because the most high frequency use of body inertia most naturally uses the inverse.</remarks>
        BodyInertia ComputeInertia(float mass);

        /// <summary>
        /// Tests a ray against the shape.
        /// </summary>
        /// <param name="pose">Pose of the shape during the ray test.</param>
        /// <param name="origin">Origin of the ray to test against the shape relative to the shape.</param>
        /// <param name="direction">Direction of the ray to test against the shape.</param>
        /// <param name="t">Distance along the ray direction to the hit point, if any, in units of the ray direction's length. In other words, hitLocation = origin + direction * t.</param>
        /// <param name="normal">Normal of the impact surface, if any.</param>
        /// <returns>True if the ray intersected the shape, false otherwise.</returns>
        bool RayTest(in RigidPose pose, Vector3 origin, Vector3 direction, out float t, out Vector3 normal);
    }

    /// <summary>
    /// Defines a compound shape type that has children of potentially different types.
    /// </summary>
    public interface ICompoundShape : IShape, IBoundsQueryableCompound
    {
        //Note that compound shapes have no wide GetBounds function. Compounds, by virtue of containing shapes of different types, cannot be usefully vectorized over.
        //Instead, their children are added to other computation batches.
        /// <summary>
        /// Computes the bounding box of a compound shape.
        /// </summary>
        /// <param name="orientation">Orientation of the compound.</param>
        /// <param name="shapeBatches">Shape batches to look up child shape information in.</param>
        /// <param name="min">Minimum of the compound's bounding box.</param>
        /// <param name="max">Maximum of the compound's bounding box.</param>
        void ComputeBounds(Quaternion orientation, Shapes shapeBatches, out Vector3 min, out Vector3 max);
        /// <summary>
        /// Submits child shapes to a bounding box batcher for vectorized bounds calculation.
        /// </summary>
        /// <remarks>This is used internally for bounding box calculation, but it is unlikely to be useful externally.</remarks>
        /// <param name="batcher">Batcher to accumulate children in.</param>
        /// <param name="pose">Pose of the compound.</param>
        /// <param name="velocity">Velocity of the compound used to expand child bounds.</param>
        /// <param name="bodyIndex">Index of the body in the active body set; used to accumulate child bounds results.</param>
        void AddChildBoundsToBatcher(ref BoundingBoxBatcher batcher, in RigidPose pose, in BodyVelocity velocity, int bodyIndex);

        //Compound shapes may require indirections into other shape batches. This isn't wonderfully fast, but this scalar path is designed more for convenience than performance anyway.
        //For performance, a batched and vectorized codepath should be used.

        /// <summary>
        /// Tests a ray against the shape.
        /// </summary>
        /// <param name="pose">Pose of the shape during the ray test.</param>
        /// <param name="ray">Ray to test against the shape.</param>
        /// <param name="maximumT">Maximum distance along the ray, in units of the ray direction's length, that the ray will test.</param>
        /// <param name="shapeBatches">Shape batches to look up child shapes in if necessary.</param>
        /// <param name="hitHandler">Callbacks called when the ray interacts with a test candidate.</param>
        void RayTest<TRayHitHandler>(in RigidPose pose, in RayData ray, ref float maximumT, Shapes shapeBatches, ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler;

        /// <summary>
        /// Tests multiple rays against the shape.
        /// </summary>
        /// <param name="pose">Pose of the shape during the ray test.</param>
        /// <param name="rays">Rays to test against the shape.</param>
        /// <param name="shapeBatches">Shape batches to look up child shapes in if necessary.</param>
        /// <param name="hitHandler">Callbacks called when the ray interacts with a test candidate.</param>
        void RayTest<TRayHitHandler>(in RigidPose pose, ref RaySource rays, Shapes shapeBatches, ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler;
        /// <summary>
        /// Gets the number of children in the compound shape.
        /// </summary>
        int ChildCount { get; }
        /// <summary>
        /// Gets a child from the compound by index.
        /// </summary>
        /// <param name="compoundChildIndex">Index of the child to look up.</param>
        /// <returns>Reference to the requested compound child.</returns>
        ref CompoundChild GetChild(int compoundChildIndex);
        /// <summary>
        /// Returns all resources used by the shape instance to the given pool.
        /// </summary>
        /// <param name="pool">Pool to return shape resources to.</param>
        void Dispose(BufferPool pool);
    }

    /// <summary>
    /// Defines a compound shape type that has children of only one type.
    /// </summary>
    /// <typeparam name="TChildShape">Type of the child shapes.</typeparam>
    /// <typeparam name="TChildShapeWide">Type of the child shapes, formatted in AOSOA layout.</typeparam>
    public interface IHomogeneousCompoundShape<TChildShape, TChildShapeWide> : IShape, IBoundsQueryableCompound
        where TChildShape : unmanaged, IConvexShape
        where TChildShapeWide : unmanaged, IShapeWide<TChildShape>
    {
        /// <summary>
        /// Computes the bounding box of a compound shape.
        /// </summary>
        /// <param name="orientation">Orientation of the compound.</param>
        /// <param name="min">Minimum of the compound's bounding box.</param>
        /// <param name="max">Maximum of the compound's bounding box.</param>
        void ComputeBounds(Quaternion orientation, out Vector3 min, out Vector3 max);

        /// <summary>
        /// Tests a ray against the shape.
        /// </summary>
        /// <param name="pose">Pose of the shape during the ray test.</param>
        /// <param name="ray">Ray to test against the shape.</param>
        /// <param name="maximumT">Maximum distance along the ray, in units of the ray direction's length, that the ray will test.</param>
        /// <param name="hitHandler">Callbacks called when the ray interacts with a test candidate.</param>
        void RayTest<TRayHitHandler>(in RigidPose pose, in RayData ray, ref float maximumT, ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler;

        /// <summary>
        /// Tests multiple rays against the shape.
        /// </summary>
        /// <param name="pose">Pose of the shape during the ray test.</param>
        /// <param name="rays">Rays to test against the shape.</param>
        /// <param name="hitHandler">Callbacks called when the ray interacts with a test candidate.</param>
        void RayTest<TRayHitHandler>(in RigidPose pose, ref RaySource rays, ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler;
        /// <summary>
        /// Gets the number of children in the compound shape.
        /// </summary>
        int ChildCount { get; }
        /// <summary>
        /// Gets a child shape as it appears in the compound's local space.
        /// </summary>
        /// <param name="childIndex">Index of the child in the compound parent.</param>
        /// <param name="childData">Data representing the child.</param>
        void GetLocalChild(int childIndex, out TChildShape childData);
        /// <summary>
        /// Gets a child shape from the compound and compounds a pose for it in the local space of the compound parent.
        /// Useful for processes which require a child shape (like a triangle in a mesh) to have their center of mass at zero in the child's own local space.
        /// </summary>
        /// <param name="childIndex">Index of the child.</param>
        /// <param name="childData">Shape of the child.</param>
        /// <param name="childPose">Pose in the compound's local space that brings the child shape as described to the proper location in the parent compound's local space.</param>
        void GetPosedLocalChild(int childIndex, out TChildShape childData, out RigidPose childPose);
        /// <summary>
        /// Gets a child shape as it appears in the compound's local space.
        /// </summary>
        /// <param name="childIndex">Index of the child in the compound parent.</param>
        /// <param name="childData">Reference to an AOSOA slot.</param>
        void GetLocalChild(int childIndex, ref TChildShapeWide childData);
        /// <summary>
        /// Returns all resources used by the shape instance to the given pool.
        /// </summary>
        /// <param name="pool">Pool to return shape resources to.</param>
        void Dispose(BufferPool pool);
    }

    /// <summary>
    /// Defines a widely vectorized bundle representation of a shape.
    /// </summary>
    /// <typeparam name="TShape">Scalar type of the shape.</typeparam>
    public interface IShapeWide<TShape> where TShape : IShape
    {
        /// <summary>
        /// Gets whether this type supports accessing its memory by lane offsets. If false, WriteSlot must be used instead of WriteFirst.
        /// </summary>
        bool AllowOffsetMemoryAccess { get; }
        /// <summary>
        /// Gets the number of bytes required for allocations within the wide shape.
        /// </summary>
        int InternalAllocationSize { get; }
        /// <summary>
        /// For types with a nonzero internal allocation size, provides memory to the shape for internal allocations.
        /// Memory should be assumed to be stack allocated.
        /// </summary>
        /// <param name="memory">Memory to use for internal allocations in the wide shape.</param>
        void Initialize(in Buffer<byte> memory);

        /// <summary>
        /// Places the specified AOS-formatted shape into the first lane of the wide 'this' reference.
        /// </summary>
        /// <remarks>Note that we are effectively using the TShapeWide as a stride.
        /// The base address is offset by the user of this function, so the implementation only ever considers the first slot.</remarks>
        /// <param name="source">AOS-formatted shape to gather from.</param>
        void WriteFirst(in TShape source);
        /// <summary>
        /// Places the specified AOS-formatted shape into the selected slot of the wide 'this' reference.
        /// </summary>
        /// <param name="index">Index of the slot to put the data into.</param>
        /// <param name="source">Source of the data to insert.</param>
        void WriteSlot(int index, in TShape source);
        /// <summary>
        /// Broadcasts a scalar shape into a bundle containing the same shape in every lane.
        /// </summary>
        /// <param name="shape">Scalar shape to broadcast.</param>
        void Broadcast(in TShape shape);
        /// <summary>
        /// Computes the bounds of all shapes in the bundle.
        /// </summary>
        /// <param name="orientations">Orientations of the shapes in the bundle.</param>
        /// <param name="countInBundle">Number of lanes filled in the bundle.</param>
        /// <param name="maximumRadius">Computed maximum radius of the shapes in the bundle.</param>
        /// <param name="maximumAngularExpansion">Computed maximum bounds expansion that can be caused by angular motion.</param>
        /// <param name="min">Minimum bounds of the shapes.</param>
        /// <param name="max">Maximum bounds of the shapes.</param>
        void GetBounds(ref QuaternionWide orientations, int countInBundle, out Vector<float> maximumRadius, out Vector<float> maximumAngularExpansion, out Vector3Wide min, out Vector3Wide max);
        /// <summary>
        /// Gets the lower bound on the number of rays to execute in a wide fashion. Ray bundles with fewer rays will fall back to the single ray code path.
        /// </summary>
        int MinimumWideRayCount { get; }

        /// <summary>
        /// Tests a ray against the shape.
        /// </summary>
        /// <param name="poses">Poses of the shape bundle during the ray test.</param>
        /// <param name="rayWide">Ray to test against the shape bundle.</param>
        /// <param name="intersected">Mask representing hit state in each lane. -1 means the ray in that lane hit, 0 means a miss.</param>
        /// <param name="t">Distance along the ray direction to the hit point, if any, in units of the ray direction's length. In other words, hitLocation = origin + direction * t.</param>
        /// <param name="normal">Normal of the impact surface, if any.</param>
        void RayTest(ref RigidPoseWide poses, ref RayWide rayWide, out Vector<int> intersected, out Vector<float> t, out Vector3Wide normal);
    }

}
