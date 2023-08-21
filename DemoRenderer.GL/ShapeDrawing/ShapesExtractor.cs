using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.Collidables;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Diagnostics;

namespace DemoRenderer.ShapeDrawing
{
    public struct ShapeCache
    {
        internal QuickList<SphereInstance> Spheres;
        internal QuickList<CapsuleInstance> Capsules;
        internal QuickList<CylinderInstance> Cylinders;
        internal QuickList<BoxInstance> Boxes;
        internal QuickList<TriangleInstance> Triangles;
        internal QuickList<MeshInstance> Meshes;

        public ShapeCache(int initialCapacityPerShapeType, BufferPool pool)
        {
            Spheres = new QuickList<SphereInstance>(initialCapacityPerShapeType, pool);
            Capsules = new QuickList<CapsuleInstance>(initialCapacityPerShapeType, pool);
            Cylinders = new QuickList<CylinderInstance>(initialCapacityPerShapeType, pool);
            Boxes = new QuickList<BoxInstance>(initialCapacityPerShapeType, pool);
            Triangles = new QuickList<TriangleInstance>(initialCapacityPerShapeType, pool);
            Meshes = new QuickList<MeshInstance>(initialCapacityPerShapeType, pool);
        }
        public void Clear()
        {
            Spheres.Count = 0;
            Capsules.Count = 0;
            Cylinders.Count = 0;
            Boxes.Count = 0;
            Triangles.Count = 0;
            Meshes.Count = 0;
        }
        public void Dispose(BufferPool pool)
        {
            Spheres.Dispose(pool);
            Capsules.Dispose(pool);
            Cylinders.Dispose(pool);
            Boxes.Dispose(pool);
            Triangles.Dispose(pool);
            Meshes.Dispose(pool);
        }
    }
    public class ShapesExtractor : IDisposable
    {
        public ShapeCache ShapeCache;

        BufferPool pool;
        public MeshCache MeshCache;

        ParallelLooper looper;
        public ShapesExtractor(ParallelLooper looper, BufferPool pool, int initialCapacityPerShapeType = 1024)
        {
            ShapeCache = new ShapeCache(initialCapacityPerShapeType, pool);
            this.MeshCache = new MeshCache(pool);
            this.pool = pool;
            this.looper = looper;
        }

        public void ClearInstances()
        {
            ShapeCache.Clear();
        }

        private unsafe void AddCompoundChildren(ref Buffer<CompoundChild> children, Shapes shapes, RigidPose pose, Vector3 color, ref ShapeCache shapeCache, BufferPool pool)
        {
            for (int i = 0; i < children.Length; ++i)
            {
                ref var child = ref children[i];
                RigidPose childPose;
                Compound.GetRotatedChildPose(child.LocalPosition, child.LocalOrientation, pose.Orientation, out childPose.Position, out childPose.Orientation);
                childPose.Position += pose.Position;
                AddShape(shapes, child.ShapeIndex, childPose, color, ref shapeCache, pool);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void AddShape(void* shapeData, int shapeType, Shapes shapes, RigidPose pose, Vector3 color, ref ShapeCache shapeCache, BufferPool pool)
        {
            //TODO: This should likely be swapped over to a registration-based virtualized table approach to more easily support custom shape extractors-
            //generic terrain windows and examples like voxel grids would benefit.
            switch (shapeType)
            {
                case Sphere.Id:
                    {
                        SphereInstance instance;
                        instance.Position = pose.Position;
                        instance.Radius = Unsafe.AsRef<Sphere>(shapeData).Radius;
                        Helpers.PackOrientation(pose.Orientation, out instance.PackedOrientation);
                        instance.PackedColor = Helpers.PackColor(color);
                        shapeCache.Spheres.Add(instance, pool);
                    }
                    break;
                case Capsule.Id:
                    {
                        CapsuleInstance instance;
                        instance.Position = pose.Position;
                        ref var capsule = ref Unsafe.AsRef<Capsule>(shapeData);
                        instance.Radius = capsule.Radius;
                        instance.HalfLength = capsule.HalfLength;
                        instance.PackedOrientation = Helpers.PackOrientationU64(pose.Orientation);
                        instance.PackedColor = Helpers.PackColor(color);
                        shapeCache.Capsules.Add(instance, pool);
                    }
                    break;
                case Box.Id:
                    {
                        BoxInstance instance;
                        instance.Position = pose.Position;
                        ref var box = ref Unsafe.AsRef<Box>(shapeData);
                        instance.PackedColor = Helpers.PackColor(color);
                        instance.Orientation = pose.Orientation;
                        instance.HalfWidth = box.HalfWidth;
                        instance.HalfHeight = box.HalfHeight;
                        instance.HalfLength = box.HalfLength;
                        shapeCache.Boxes.Add(instance, pool);
                    }
                    break;
                case Triangle.Id:
                    {
                        ref var triangle = ref Unsafe.AsRef<Triangle>(shapeData);
                        TriangleInstance instance;
                        instance.A = triangle.A;
                        instance.PackedColor = Helpers.PackColor(color);
                        instance.B = triangle.B;
                        instance.C = triangle.C;
                        instance.PackedOrientation = Helpers.PackOrientationU64(pose.Orientation);
                        instance.X = pose.Position.X;
                        instance.Y = pose.Position.Y;
                        instance.Z = pose.Position.Z;
                        shapeCache.Triangles.Add(instance, pool);
                    }
                    break;
                case Cylinder.Id:
                    {
                        CylinderInstance instance;
                        instance.Position = pose.Position;
                        ref var cylinder = ref Unsafe.AsRef<Cylinder>(shapeData);
                        instance.Radius = cylinder.Radius;
                        instance.HalfLength = cylinder.HalfLength;
                        instance.PackedOrientation = Helpers.PackOrientationU64(pose.Orientation);
                        instance.PackedColor = Helpers.PackColor(color);
                        shapeCache.Cylinders.Add(instance, pool);
                    }
                    break;
                case ConvexHull.Id:
                    {
                        ref var hull = ref Unsafe.AsRef<ConvexHull>(shapeData);
                        MeshInstance instance;
                        instance.Position = pose.Position;
                        instance.PackedColor = Helpers.PackColor(color);
                        instance.PackedOrientation = Helpers.PackOrientationU64(pose.Orientation);
                        instance.Scale = Vector3.One;
                        //Memory can be reused, so we slightly reduce the probability of a bad reuse by taking the first 64 bits of data into the hash.
                        var id = (ulong)hull.Points.Memory ^ (ulong)hull.Points.Length ^ (*(ulong*)hull.Points.Memory);
                        bool meshExisted;
                        Buffer<Vector4> vertices;
                        lock (MeshCache)
                        {
                            meshExisted = MeshCache.TryGetExistingMesh(id, out instance.VertexStart, out vertices);
                        }
                        if (!meshExisted)
                        {
                            int triangleCount = 0;
                            for (int i = 0; i < hull.FaceToVertexIndicesStart.Length; ++i)
                            {
                                hull.GetVertexIndicesForFace(i, out var faceVertexIndices);
                                triangleCount += faceVertexIndices.Length - 2;
                            }
                            instance.VertexCount = triangleCount * 3;
                            lock (MeshCache)
                            {
                                MeshCache.Allocate(id, instance.VertexCount, out instance.VertexStart, out vertices);
                            }
                            //This is a fresh allocation, so we need to upload vertex data.
                            int targetVertexIndex = 0;
                            for (int i = 0; i < hull.FaceToVertexIndicesStart.Length; ++i)
                            {
                                hull.GetVertexIndicesForFace(i, out var faceVertexIndices);
                                hull.GetPoint(faceVertexIndices[0], out var faceOrigin);
                                hull.GetPoint(faceVertexIndices[1], out var previousEdgeEnd);
                                for (int j = 2; j < faceVertexIndices.Length; ++j)
                                {
                                    vertices[targetVertexIndex++] = new Vector4(faceOrigin, 1.0f);
                                    vertices[targetVertexIndex++] = new Vector4(previousEdgeEnd, 1.0f);
                                    hull.GetPoint(faceVertexIndices[j], out previousEdgeEnd);
                                    vertices[targetVertexIndex++] = new Vector4(previousEdgeEnd, 1.0f);

                                }
                            }
                        }
                        else
                        {
                            instance.VertexCount = vertices.Length;
                        }
                        shapeCache.Meshes.Add(instance, pool);
                    }
                    break;
                case Compound.Id:
                    {
                        AddCompoundChildren(ref Unsafe.AsRef<Compound>(shapeData).Children, shapes, pose, color, ref shapeCache, pool);
                    }
                    break;
                case BigCompound.Id:
                    {
                        AddCompoundChildren(ref Unsafe.AsRef<BigCompound>(shapeData).Children, shapes, pose, color, ref shapeCache, pool);
                    }
                    break;
                case Mesh.Id:
                    {
                        ref var mesh = ref Unsafe.AsRef<Mesh>(shapeData);
                        MeshInstance instance;
                        instance.Position = pose.Position;
                        instance.PackedColor = Helpers.PackColor(color);
                        instance.PackedOrientation = Helpers.PackOrientationU64(pose.Orientation);
                        instance.Scale = mesh.Scale;
                        //Memory can be reused, so we slightly reduce the probability of a bad reuse by taking the first 64 bits of data into the hash.
                        var id = (ulong)mesh.Triangles.Memory ^ (ulong)mesh.Triangles.Length ^ (*(ulong*)mesh.Triangles.Memory); ;
                        instance.VertexCount = mesh.Triangles.Length * 3;
                        bool newAllocation;
                        Buffer<Vector4> vertices;
                        lock (MeshCache)
                        {
                            newAllocation = MeshCache.Allocate(id, instance.VertexCount, out instance.VertexStart, out vertices);
                        }
                        if (newAllocation)
                        {
                            //This is a fresh allocation, so we need to upload vertex data.
                            for (int i = 0; i < mesh.Triangles.Length; ++i)
                            {
                                ref var triangle = ref mesh.Triangles[i];
                                var baseVertexIndex = i * 3;
                                //Note winding flip for rendering.
                                vertices[baseVertexIndex] = new Vector4(triangle.A, 1.0f);
                                vertices[baseVertexIndex + 1] = new Vector4(triangle.C, 1.0f);
                                vertices[baseVertexIndex + 2] = new Vector4(triangle.B, 1.0f);
                            }
                        }
                        shapeCache.Meshes.Add(instance, pool);
                    }
                    break;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void AddShape(void* shapeData, int shapeType, Shapes shapes, RigidPose pose, Vector3 color)
        {
            AddShape(shapeData, shapeType, shapes, pose, color, ref ShapeCache, pool);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void AddShape(Shapes shapes, TypedIndex shapeIndex, RigidPose pose, Vector3 color, ref ShapeCache shapeCache, BufferPool pool)
        {
            if (shapeIndex.Exists)
            {
                shapes[shapeIndex.Type].GetShapeData(shapeIndex.Index, out var shapeData, out _);
                AddShape(shapeData, shapeIndex.Type, shapes, pose, color, ref shapeCache, pool);
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void AddShape(Shapes shapes, TypedIndex shapeIndex, RigidPose pose, Vector3 color)
        {
            if (shapeIndex.Exists)
            {
                shapes[shapeIndex.Type].GetShapeData(shapeIndex.Index, out var shapeData, out _);
                AddShape(shapeData, shapeIndex.Type, shapes, pose, color, ref ShapeCache, pool);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void AddShape<TShape>(TShape shape, Shapes shapes, RigidPose pose, Vector3 color) where TShape : IShape
        {
            AddShape(Unsafe.AsPointer(ref shape), TShape.TypeId, shapes, pose, color, ref ShapeCache, pool);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void AddBodyShape(Shapes shapes, Bodies bodies, int setIndex, int indexInSet, ref ShapeCache shapeCache, BufferPool pool)
        {
            ref var set = ref bodies.Sets[setIndex];
            var handle = set.IndexToHandle[indexInSet];
            //Body color is based on three factors:
            //1) Handle as a hash seed that is unpacked into a color
            //2) Dynamics vs kinematic state
            //3) Activity state
            //The handle is hashed to get variation.
            ref var activity = ref set.Activity[indexInSet];
            Vector3 color;
            Helpers.UnpackColor((uint)HashHelper.Rehash(handle.Value), out Vector3 colorVariation);
            ref var state = ref set.DynamicsState[indexInSet];
            if (Bodies.IsKinematic(state.Inertia.Local))
            {
                var kinematicBase = new Vector3(0, 0.609f, 0.37f);
                var kinematicVariationSpan = new Vector3(0.1f, 0.1f, 0.1f);
                color = kinematicBase + kinematicVariationSpan * colorVariation;
            }
            else
            {
                var dynamicBase = new Vector3(0.8f, 0.1f, 0.566f);
                var dynamicVariationSpan = new Vector3(0.2f, 0.2f, 0.2f);
                color = dynamicBase + dynamicVariationSpan * colorVariation;
            }

            if (setIndex == 0)
            {
                if (activity.SleepCandidate)
                {
                    var sleepCandidateTint = new Vector3(0.35f, 0.35f, 0.7f);
                    color *= sleepCandidateTint;
                }
            }
            else
            {
                var sleepTint = new Vector3(0.2f, 0.2f, 0.4f);
                color *= sleepTint;
            }

            AddShape(shapes, set.Collidables[indexInSet].Shape, state.Motion.Pose, color, ref shapeCache, pool);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void AddStaticShape(Shapes shapes, Statics statics, int index, ref ShapeCache shapeCache, BufferPool pool)
        {
            var handle = statics.IndexToHandle[index];
            //Statics don't have any activity states. Just some simple variation on a central static color.
            Helpers.UnpackColor((uint)HashHelper.Rehash(handle.Value), out Vector3 colorVariation);
            var staticBase = new Vector3(0.1f, 0.057f, 0.014f);
            var staticVariationSpan = new Vector3(0.07f, 0.07f, 0.03f);
            var color = staticBase + staticVariationSpan * colorVariation;
            ref var collidable = ref statics[index];
            AddShape(shapes, collidable.Shape, collidable.Pose, color, ref shapeCache, pool);
        }

        struct Job
        {
            public int SimulationIndex;
            //If the job is about statics, the set index will be -1.
            public int SetIndex;
            public int StartIndex;
            public int Count;
        }
        QuickList<Job> jobs;
        //The extractor can operate over one or multiple simulations. We cache them locally for threads to access.
        Simulation[] simulations;
        Simulation simulation;
        Buffer<ShapeCache> workerCaches;

        void PrepareForMultithreadedExecution(IThreadDispatcher threadDispatcher)
        {
            jobs = new QuickList<Job>(128, pool);
            looper.Dispatcher = threadDispatcher;
            pool.Take(threadDispatcher.ThreadCount, out workerCaches);
            for (int i = 0; i < workerCaches.Length; ++i)
            {
                workerCaches[i] = new ShapeCache(128, threadDispatcher.WorkerPools[i]);
            }
        }

        void EndMultithreadedExecution()
        {
            jobs.Dispose(pool);
            for (int i = 0; i < workerCaches.Length; ++i)
            {
                workerCaches[i].Dispose(looper.Dispatcher.WorkerPools[i]);
            }
            looper.Dispatcher = null;
            pool.Return(ref workerCaches);
        }

        static void CreateJobs(Simulation simulation, int simulationIndex, ref QuickList<Job> jobs, BufferPool pool)
        {
            const int targetBodiesPerJob = 1024;
            for (int setIndex = 0; setIndex < simulation.Bodies.Sets.Length; ++setIndex)
            {
                ref var set = ref simulation.Bodies.Sets[setIndex];
                if (set.Allocated && set.Count > 0) //active set can be allocated and have no bodies in it.
                {
                    var jobCount = (set.Count + targetBodiesPerJob - 1) / targetBodiesPerJob;
                    var bodiesPerJob = set.Count / jobCount;
                    var remainder = set.Count - bodiesPerJob * jobCount;
                    var previousEnd = 0;
                    for (int j = 0; j < jobCount; ++j)
                    {
                        var count = j < remainder ? bodiesPerJob + 1 : bodiesPerJob;
                        jobs.Allocate(pool) = new Job { SimulationIndex = simulationIndex, SetIndex = setIndex, Count = count, StartIndex = previousEnd };
                        previousEnd += count;
                    }
                }
            }
            {
                if (simulation.Statics.Count > 0)
                {
                    var jobCount = (simulation.Statics.Count + targetBodiesPerJob - 1) / targetBodiesPerJob;
                    var bodiesPerJob = simulation.Statics.Count / jobCount;
                    var remainder = simulation.Statics.Count - bodiesPerJob * jobCount;
                    var previousEnd = 0;
                    for (int j = 0; j < jobCount; ++j)
                    {
                        var count = j < remainder ? bodiesPerJob + 1 : bodiesPerJob;
                        jobs.Allocate(pool) = new Job { SimulationIndex = simulationIndex, SetIndex = -1, Count = count, StartIndex = previousEnd };
                        previousEnd += count;
                    }
                }
            }
        }

        void AddShapesForJob(int jobIndex, int workerIndex)
        {
            var job = jobs[jobIndex];
            var simulation = simulations == null ? this.simulation : this.simulations[job.SimulationIndex];
            var pool = looper.Dispatcher.WorkerPools[workerIndex];

            if (job.SetIndex >= 0)
            {
                ref var set = ref simulation.Bodies.Sets[job.SetIndex];
                var endIndex = job.StartIndex + job.Count;
                Debug.Assert(endIndex <= set.Count);
                for (int bodyIndex = job.StartIndex; bodyIndex < endIndex; ++bodyIndex)
                {
                    AddBodyShape(simulation.Shapes, simulation.Bodies, job.SetIndex, bodyIndex, ref workerCaches[workerIndex], pool);
                }
            }
            else
            {
                //It's a static.
                var endIndex = job.StartIndex + job.Count;
                Debug.Assert(endIndex <= simulation.Statics.Count);
                for (int staticIndex = job.StartIndex; staticIndex < endIndex; ++staticIndex)
                {
                    AddStaticShape(simulation.Shapes, simulation.Statics, staticIndex, ref workerCaches[workerIndex], pool);
                }
            }
        }

        object workerShapeMergeLocker = new object();

        void CopyWorkerCacheToMainCache<TShape>(ref QuickList<TShape> workerCache, ref QuickList<TShape> mainCache) where TShape : unmanaged
        {
            if (workerCache.Count > 0)
            {
                int copyStartLocation;
                lock (workerShapeMergeLocker)
                {
                    var newCount = mainCache.Count + workerCache.Count;
                    mainCache.EnsureCapacity(newCount, pool);
                    copyStartLocation = mainCache.Count;
                    mainCache.Count = newCount;
                }
                workerCache.Span.CopyTo(0, mainCache.Span, copyStartLocation, workerCache.Count);
            }
        }

        void WorkerDone(int workerIndex)
        {
            //This fires when a worker finishes its work. We should copy the results into the main buffer.
            ref var workerCache = ref workerCaches[workerIndex];
            CopyWorkerCacheToMainCache(ref workerCache.Spheres, ref ShapeCache.Spheres);
            CopyWorkerCacheToMainCache(ref workerCache.Capsules, ref ShapeCache.Capsules);
            CopyWorkerCacheToMainCache(ref workerCache.Boxes, ref ShapeCache.Boxes);
            CopyWorkerCacheToMainCache(ref workerCache.Cylinders, ref ShapeCache.Cylinders);
            CopyWorkerCacheToMainCache(ref workerCache.Triangles, ref ShapeCache.Triangles);
            CopyWorkerCacheToMainCache(ref workerCache.Meshes, ref ShapeCache.Meshes);
        }

        void AddShapesSequentially(Simulation simulation)
        {
            for (int i = 0; i < simulation.Bodies.Sets.Length; ++i)
            {
                ref var set = ref simulation.Bodies.Sets[i];
                if (set.Allocated) //Islands are stored noncontiguously; skip those which have been deallocated.
                {
                    for (int bodyIndex = 0; bodyIndex < set.Count; ++bodyIndex)
                    {
                        AddBodyShape(simulation.Shapes, simulation.Bodies, i, bodyIndex, ref ShapeCache, pool);
                    }
                }
            }
            for (int i = 0; i < simulation.Statics.Count; ++i)
            {
                AddStaticShape(simulation.Shapes, simulation.Statics, i, ref ShapeCache, pool);
            }
        }


        public void AddInstances(Simulation[] simulations, IThreadDispatcher threadDispatcher = null)
        {
            if (threadDispatcher != null && threadDispatcher.ThreadCount > 1)
            {
                this.simulations = simulations;
                PrepareForMultithreadedExecution(threadDispatcher);
                for (int simulationIndex = 0; simulationIndex < simulations.Length; ++simulationIndex)
                {
                    CreateJobs(simulations[simulationIndex], simulationIndex, ref jobs, pool);
                }
                looper.For(0, jobs.Count, AddShapesForJob, WorkerDone);
                EndMultithreadedExecution();
                this.simulations = default;
            }
            else
            {
                for (int simulationIndex = 0; simulationIndex < simulations.Length; ++simulationIndex)
                {
                    AddShapesSequentially(simulations[simulationIndex]);
                }
            }
        }

        public void AddInstances(Simulation simulation, IThreadDispatcher threadDispatcher = null)
        {
            if (threadDispatcher != null)
            {
                this.simulation = simulation;
                PrepareForMultithreadedExecution(threadDispatcher);
                CreateJobs(simulation, 0, ref jobs, pool);
                looper.For(0, jobs.Count, AddShapesForJob, WorkerDone);
                EndMultithreadedExecution();
                this.simulation = null;
            }
            else
            {
                AddShapesSequentially(simulation);
            }
        }

        public void Dispose()
        {
            ShapeCache.Dispose(pool);
            MeshCache.Dispose();
        }
    }
}
