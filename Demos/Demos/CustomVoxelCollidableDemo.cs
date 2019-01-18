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
        /// Size of each individual voxel. Updating this value requires refitting the Tree.
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

    //The Simulation.Shapes collection stores all shape data in a simulation, segregated by type.
    //Systems can call into the type-specific shape batches to execute shape defined logic.
    //(Most shape types just use one of the generic types like ConvexShapeBatch<T>, CompoundShapeBatch<T>, MeshShapeBatch<T>,
    //but voxels aren't a perfect fit for any of those. You could pretty easily generalize MeshShapeBatch<T> to cover any group type
    //that always outputs the same shape type, but rather than doing that, this demo will show how to create a unique shape batch type.)


    public class CustomVoxelCollidableDemo : Demo
    {
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-20, 10, -20);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = MathHelper.Pi * 0.05f;
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));

            //Simulation.NarrowPhase.CollisionTaskRegistry.Register(new ConvexCompoundCollisionTask<Sphere, Voxels, ConvexCompoundOverlapFinder<Sphere, SphereWide, Voxels>, ConvexMeshContinuations<Voxels>, NonconvexReduction>());

            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -0.5f, 0), new CollidableDescription(Simulation.Shapes.Add(new Box(300, 1, 300)), 0.1f)));
        }

    }
}


