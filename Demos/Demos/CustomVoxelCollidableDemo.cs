using BepuUtilities;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Collidables;
using System.Numerics;
using Quaternion = BepuUtilities.Quaternion;
using DemoContentLoader;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuPhysics.Trees;
using BepuUtilities.Memory;
using BepuUtilities.Collections;
using System.Runtime.CompilerServices;

namespace Demos.Demos
{
    //There are a variety of types related to collision pairs designed to work with particular interfaces.
    //We don't want to make a bunch of special cases, so we'll implement some of the interfaces.
    //Note that we don't just use ICompoundShape, even though that would lessen the amount of special case work needed-
    //ICompoundShape is designed to work in a more general case where every child may have a unique shape.
    //Meshes and voxel sets don't behave this way- all children are a single type, hence 'homogeneous'.
    struct Voxels : IHomogeneousCompoundShape<Box, BoxWide>
    {
        //Type ids should be unique across all shape types in a simulation.
        public int TypeId => 12;


        //Using an object space tree isn't necessarily ideal for a highly regular data like voxels.
        //We're using it here since it exists already and a voxel-specialized version doesn't.
        //If you wanted maximum efficiency for some specific use case- especially with support for faster modifications- 
        //then you may want to consider an alternative.
        public Tree Tree;

        /// <summary>
        /// List of the voxels in the voxel set by three dimensional index.
        /// </summary>
        public QuickList<Vector3> VoxelIndices;

        /// <summary>
        /// Size of each individual voxel. Updating this value requires updating the tree leaf bounds and refitting.
        /// </summary>
        public Vector3 VoxelSize;

        public int ChildCount => VoxelIndices.Count;

        public Voxels(QuickList<Vector3> voxelIndices, Vector3 voxelSize, BufferPool pool)
        {
            VoxelIndices = voxelIndices;
            VoxelSize = voxelSize;
            Tree = new Tree(pool, voxelIndices.Count);
            //Could stackalloc here, but the assumption is that there could be quite a few voxels.
            //Quite possible to overflow the stack, so we instead resort to heap allocation.
            pool.Take(voxelIndices.Count, out Buffer<BoundingBox> bounds);
            bounds = bounds.Slice(0, voxelIndices.Count);
            for (int i = 0; i < voxelIndices.Count; ++i)
            {
                ref var voxel = ref voxelIndices[i];
                ref var voxelBounds = ref bounds[i];
                //Note that the voxel scale is baked into the tree. That's different than the Mesh, which allows sharing the same tree across different scaled shapes.
                //You could do something similar with the voxel set if you wanted to; check ou tthe Mesh for an example.
                voxelBounds.Min = voxel * VoxelSize;
                voxelBounds.Max = voxelBounds.Min + VoxelSize;
            }
            Tree.SweepBuild(pool, bounds);
            pool.Return(ref bounds);
        }

        public ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapeBatches)
        {
            //Shapes types are responsible for informing the shape system how to create a batch for them.
            //Convex shapes will return a ConvexShapeBatch<TShape>, compound shapes a CompoundShapeBatch<TShape>,
            //and then we have the HomogeneousCompoundShapeBatch... I don't love this name, but it just means
            //that every child of the compound shape has the same type. So a mesh is a 'homogeneous compound'
            //because its children are all triangles. Likewise, a voxel set is too, because every child is just a box.
            return new HomogeneousCompoundShapeBatch<Voxels, Box, BoxWide>(pool, initialCapacity);
        }

        public void ComputeBounds(in Quaternion orientation, out Vector3 min, out Vector3 max)
        {
            Matrix3x3.CreateFromQuaternion(orientation, out var basis);
            min = new Vector3(float.MaxValue);
            max = new Vector3(float.MinValue);
            for (int i = 0; i < VoxelIndices.Count; ++i)
            {
                var localVoxelPosition = (VoxelIndices[i] + new Vector3(0.5f)) * VoxelSize;
                Matrix3x3.TransformTranspose(localVoxelPosition, basis, out var rotatedPosition);
                min = Vector3.Min(rotatedPosition, min);
                max = Vector3.Max(rotatedPosition, max);
            }
            //All children have the same shape and orientation, so we can simply expand the centroids bounding box.
            var box = new Box(VoxelSize.X, VoxelSize.Y, VoxelSize.Z);
            box.ComputeBounds(orientation, out var childLocalMin, out var childLocalMax);
            min += childLocalMin;
            max += childLocalMax;
        }

        public bool RayTest(in RigidPose pose, in Vector3 origin, in Vector3 direction, float maximumT, out float t, out Vector3 normal)
        {
            throw new System.NotImplementedException();
        }

        public void RayTest<TRayHitHandler>(in RigidPose pose, ref RaySource rays, ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayBatchHitHandler
        {
            throw new System.NotImplementedException();
        }

        public void GetLocalChild(int childIndex, out Box shape)
        {
            shape.HalfWidth = VoxelSize.X;
            shape.HalfHeight = VoxelSize.Y;
            shape.HalfLength = VoxelSize.Z;
        }

        public void GetLocalChild(int childIndex, ref BoxWide shapeWide)
        {
            //This function provides a reference to a lane in an AOSOA structure.
            //We are to fill in the first lane and ignore the others.
            GatherScatter.GetFirst(ref shapeWide.HalfWidth) = VoxelSize.X;
            GatherScatter.GetFirst(ref shapeWide.HalfHeight) = VoxelSize.Y;
            GatherScatter.GetFirst(ref shapeWide.HalfLength) = VoxelSize.Z;
        }

        public unsafe void FindLocalOverlaps<TOverlaps, TSubpairOverlaps>(PairsToTestForOverlap* pairs, int count, BufferPool pool, Shapes shapes, ref TOverlaps overlaps)
             where TOverlaps : struct, ICollisionTaskOverlaps<TSubpairOverlaps>
             where TSubpairOverlaps : struct, ICollisionTaskSubpairOverlaps
        {
            //Just traverse every child against the tree sequentially.
            //This sequentializes a whole lot of cache misses. You could probably get some benefit out of traversing all pairs 'simultaneously'- that is, 
            //using the fact that we have lots of independent queries to ensure the CPU always has something to do. But for the sake of this demo, we'll do it the simple way.

            //All this enumerator does is take an overlap reported by the GetOverlaps function and add it to the overlaps list.
            ShapeTreeOverlapEnumerator<TSubpairOverlaps> enumerator;
            enumerator.Pool = pool;
            for (int i = 0; i < count; ++i)
            {
                ref var pair = ref pairs[i];
                ref var voxelsSet = ref Unsafe.AsRef<Voxels>(pair.Container);
                enumerator.Overlaps = Unsafe.AsPointer(ref overlaps.GetOverlapsForPair(i));
                Tree.GetOverlaps(pair.Min, pair.Max, ref enumerator);
            }
        }

        public unsafe void FindLocalOverlaps<TOverlaps>(in Vector3 min, in Vector3 max, in Vector3 sweep, float maximumT, BufferPool pool, Shapes shapes, void* overlaps) where TOverlaps : ICollisionTaskSubpairOverlaps
        {
            //Similar to the non-swept FindLocalOverlaps function above, this just adds the overlaps to the provided collection.
            //Some unfortunate loss of type information due to some language limitations around generic pointers- pretend the overlaps pointer has type TOverlaps*.
            //It will when C# allows it.
            ShapeTreeSweepLeafTester<TOverlaps> enumerator;
            enumerator.Pool = pool;
            enumerator.Overlaps = overlaps;
            Tree.Sweep(min, max, sweep, maximumT, ref enumerator);
        }

        public void Dispose(BufferPool pool)
        {
            Tree.Dispose(pool);
            VoxelIndices.Dispose(pool);
        }
    }

    //"Continuations" tell the collision batcher what to do with the collision detection results after completing a batch.
    //For simple convex-convex pairs in the usual narrow phase pipeline, they just report the manifold for constraint generation.
    //For more complex types like compounds, there may be multiple subpairs, each with their own contact manifold.
    //Those manifolds are combined in a postprocessing step. For most compounds, this is a "NonconvexReduction".
    //There is also a "MeshReduction" which is a bit more involved- it tries to smooth out collisions at the boundaries of triangles to avoid bumps during sliding.
    //For our voxel set, we'll just use the NonconvexReduction despite the fact that it'll allow bumps at shape boundaries during sliding.
    //(I'll leave boundary smoothing for voxels as a not-easy exercise for the highly motivated reader.)
    public struct ConvexVoxelsContinuations : IConvexCompoundContinuationHandler<NonconvexReduction>
    {
        public CollisionContinuationType CollisionContinuationType => CollisionContinuationType.NonconvexReduction;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexReduction CreateContinuation<TCallbacks>(
            ref CollisionBatcher<TCallbacks> collisionBatcher, int childCount, in BoundsTestedPair pair, out int continuationIndex)
            where TCallbacks : struct, ICollisionCallbacks
        {
            return ref collisionBatcher.NonconvexReductions.CreateContinuation(childCount, collisionBatcher.Pool, out continuationIndex);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ConfigureContinuationChild<TCallbacks>(
            ref CollisionBatcher<TCallbacks> collisionBatcher, ref NonconvexReduction continuation, int continuationChildIndex, in BoundsTestedPair pair, int shapeTypeA, int childIndex,
            out RigidPose childPoseB, out int childTypeB, out void* childShapeDataB)
            where TCallbacks : struct, ICollisionCallbacks
        {
            ref var voxels = ref Unsafe.AsRef<Voxels>(pair.B);
            ref var voxelIndex = ref voxels.VoxelIndices[childIndex];
            var localPosition = (voxelIndex + new Vector3(0.5f)) * voxels.VoxelSize;
            Quaternion.TransformWithoutOverlap(localPosition, pair.OrientationB, out childPoseB.Position);
            childPoseB.Orientation = Quaternion.Identity;
            ref var continuationChild = ref continuation.Children[continuationChildIndex];
            childTypeB = Box.Id;
            //The collision batcher accumulates pairs to process by pointer, since in almost every other case the shape data is available by pointer already.
            //But in the voxel set case, we don't actually have an explicit shape (or a secondary storage location as we have in MeshReductions).
            //We can't just allocate a shape on the stack and return a pointer to it- the data needs to be valid until the collision batcher flushes that type batch.
            //Fortunately, the collision batcher exposes a handy per-pair-type heap allocated memory blob that we can use to store the shape data.
            //When the collision batcher flushes, it'll automatically get cleaned up.
            var halfSize = voxels.VoxelSize * 0.5f;
            //This reinterprets the vector3 as a Box to copy into the cache, which is a bit gross.
            //The shape cache doesn't actually have any type information- it's just strategically placed memory.
            //In other words, we're just giving a place for these 12 bytes to live until the flush.
            collisionBatcher.CacheShapeB(shapeTypeA, childTypeB, Unsafe.AsPointer(ref halfSize), 12, out childShapeDataB);
            //Collision processors expect data to be provided in a specific order. The flip mask is used to make sure we're giving the data in the proper order.
            //The collision batcher also takes into account the flip mask when reporting collision data through callbacks to preserve original user order.
            if (pair.FlipMask < 0)
            {
                continuationChild.ChildIndexA = childIndex;
                continuationChild.ChildIndexB = 0;
                continuationChild.OffsetA = childPoseB.Position;
                continuationChild.OffsetB = default;
            }
            else
            {
                continuationChild.ChildIndexA = 0;
                continuationChild.ChildIndexB = childIndex;
                continuationChild.OffsetA = default;
                continuationChild.OffsetB = childPoseB.Position;
            }
        }
    }


    public class CustomVoxelCollidableDemo : Demo
    {
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-20, 10, -20);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = MathHelper.Pi * 0.05f;
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));

            Simulation.NarrowPhase.CollisionTaskRegistry.Register(new ConvexCompoundCollisionTask<Sphere, Voxels, ConvexCompoundOverlapFinder<Sphere, SphereWide, Voxels>, ConvexVoxelsContinuations, NonconvexReduction>());
            Simulation.NarrowPhase.CollisionTaskRegistry.Register(new ConvexCompoundCollisionTask<Capsule, Voxels, ConvexCompoundOverlapFinder<Capsule, CapsuleWide, Voxels>, ConvexVoxelsContinuations, NonconvexReduction>());
            Simulation.NarrowPhase.CollisionTaskRegistry.Register(new ConvexCompoundCollisionTask<Box, Voxels, ConvexCompoundOverlapFinder<Box, BoxWide, Voxels>, ConvexVoxelsContinuations, NonconvexReduction>());
            Simulation.NarrowPhase.CollisionTaskRegistry.Register(new ConvexCompoundCollisionTask<Triangle, Voxels, ConvexCompoundOverlapFinder<Triangle, TriangleWide, Voxels>, ConvexVoxelsContinuations, NonconvexReduction>());

            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -0.5f, 0), new CollidableDescription(Simulation.Shapes.Add(new Box(300, 1, 300)), 0.1f)));
        }

    }
}


