using BepuPhysics.CollisionDetection;
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
        int TypeId { get; }

        ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapeBatches);
    }

    //Note that the following bounds functions require only an orientation because the effect of the position on the bounding box is the same for all shapes.
    //By isolating the shape from the position, we can more easily swap out the position representation for higher precision modes while only modifying the stuff that actually
    //deals with positions directly.

    //Note that we also support one-off bounds calculations. They are used even in the engine sometimes. Adding individual bodies to the simulation, for example.
    //Note, however, that we do not bother supporting velocity expansion on the one-off variant. For the purposes of adding objects to the simulation, that is basically irrelevant.
    //I don't predict ever needing it, but such an implementation could be added...

    public interface IShapeRayHitHandler
    {
        void OnRayHit(int rayIndex, float t, in Vector3 normal);
    }

    public interface IConvexShape : IShape
    {
        void ComputeBounds(in BepuUtilities.Quaternion orientation, out Vector3 min, out Vector3 max);
        void ComputeAngularExpansionData(out float maximumRadius, out float maximumAngularExpansion);

        void ComputeInertia(float mass, out BodyInertia inertia);

        bool RayTest(in RigidPose pose, in Vector3 origin, in Vector3 direction, out float t, out Vector3 normal);
    }

    public interface ICompoundShape : IShape
    {
        //Note that compound shapes have no wide GetBounds function. Compounds, by virtue of containing shapes of different types, cannot be usefully vectorized over.
        //Instead, their children are added to other computation batches.
        void ComputeBounds(in BepuUtilities.Quaternion orientation, Shapes shapeBatches, out Vector3 min, out Vector3 max);
        void AddChildBoundsToBatcher(ref BoundingBoxBatcher batcher, ref RigidPose pose, ref BodyVelocity velocity, int bodyIndex);

        //Compound shapes may require indirections into other shape batches. This isn't wonderfully fast, but this scalar path is designed more for convenience than performance anyway.
        //For performance, a batched and vectorized codepath should be used.
        bool RayTest(in RigidPose pose, in Vector3 origin, in Vector3 direction, float maximumT, Shapes shapeBatches, out float t, out Vector3 normal);
        void RayTest<TRayHitHandler>(in RigidPose pose, Shapes shapeBatches, ref RaySource rays, ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler;

    }

    public unsafe interface IMeshShape : IShape
    {
        //Meshes have homogenous child types, so internal vectorization is in principle possible. And it's hard to vectorize over multiple meshes.
        //And the speed of mesh bounds calculation is pretty irrelevant, since meshes should essentially always be static.
        void ComputeBounds(in BepuUtilities.Quaternion orientation, out Vector3 min, out Vector3 max);

        bool RayTest(in RigidPose pose, in Vector3 origin, in Vector3 direction, float maximumT, out float t, out Vector3 normal);
        void RayTest<TRayHitHandler>(in RigidPose pose, ref RaySource rays, ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler;

        void FindLocalOverlaps(in Vector3 min, in Vector3 max, BufferPool pool, ref QuickList<int, Buffer<int>> overlappedChildren);
        void FindLocalOverlaps(in Vector3 min, in Vector3 max, in Vector3 sweep, float maximumT, BufferPool pool, ref QuickList<int, Buffer<int>> overlappedChildren);
        void FindLocalOverlaps(ref Buffer<IntPtr> meshes, ref Vector3Wide min, ref Vector3Wide max, int count, BufferPool pool, ref Buffer<QuickList<int, Buffer<int>>> overlappedChildren);
        void GetTriangles(ref QuickList<int, Buffer<int>> childIndices, ref Buffer<Triangle> triangles);
    }

    public interface IShapeWide<TShape> where TShape : IShape
    {
        /// <summary>
        /// Places the specified AOS-formatted shape into the first lane of the wide 'this' reference.
        /// </summary>
        /// <remarks>Note that we are effectively using the TShapeWide as a stride.
        /// The base address is offset by the user of this function, so the implementation only ever considers the first slot.</remarks>
        /// <param name="source">AOS-formatted shape to gather from.</param>
        void WriteFirst(ref TShape source);
        void Broadcast(in TShape shape);

        void GetBounds(ref QuaternionWide orientations, out Vector<float> maximumRadius, out Vector<float> maximumAngularExpansion, out Vector3Wide min, out Vector3Wide max);
        /// <summary>
        /// Gets the lower bound on the number of rays to execute in a wide fashion. Ray bundles with fewer rays will fall back to the single ray code path.
        /// </summary>
        int MinimumWideRayCount { get; }
        void RayTest(ref RigidPoses poses, ref RayWide rayWide, out Vector<int> intersected, out Vector<float> t, out Vector3Wide normal);
    }

}
