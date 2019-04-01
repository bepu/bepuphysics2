using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.Collidables;
using System.Numerics;
using System.Runtime.CompilerServices;
using SharpDX.Direct3D11;
using System;

namespace DemoRenderer.ShapeDrawing
{
    public class ShapesExtractor : IDisposable
    {
        //For now, we only have spheres. Later, once other shapes exist, this will be responsible for bucketing the different shape types and when necessary caching shape models.
        internal QuickList<SphereInstance> spheres;
        internal QuickList<CapsuleInstance> capsules;
        internal QuickList<CylinderInstance> cylinders;
        internal QuickList<BoxInstance> boxes;
        internal QuickList<TriangleInstance> triangles;
        internal QuickList<MeshInstance> meshes;

        BufferPool pool;
        public MeshCache MeshCache;

        ParallelLooper looper;
        public ShapesExtractor(Device device, ParallelLooper looper, BufferPool pool, int initialCapacityPerShapeType = 1024)
        {
            spheres = new QuickList<SphereInstance>(initialCapacityPerShapeType, pool);
            capsules = new QuickList<CapsuleInstance>(initialCapacityPerShapeType, pool);
            cylinders = new QuickList<CylinderInstance>(initialCapacityPerShapeType, pool);
            boxes = new QuickList<BoxInstance>(initialCapacityPerShapeType, pool);
            triangles = new QuickList<TriangleInstance>(initialCapacityPerShapeType, pool);
            meshes = new QuickList<MeshInstance>(initialCapacityPerShapeType, pool);
            this.MeshCache = new MeshCache(device, pool);
            this.pool = pool;
            this.looper = looper;
        }

        public void ClearInstances()
        {
            spheres.Count = 0;
            capsules.Count = 0;
            cylinders.Count = 0;
            boxes.Count = 0;
            triangles.Count = 0;
            meshes.Count = 0;
        }

        private unsafe void AddCompoundChildren(ref Buffer<CompoundChild> children, Shapes shapes, in RigidPose pose, in Vector3 color)
        {
            for (int i = 0; i < children.Length; ++i)
            {
                ref var child = ref children[i];
                Compound.GetWorldPose(child.LocalPose, pose, out var childPose);
                AddShape(shapes, child.ShapeIndex, ref childPose, color);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void AddShape(void* shapeData, int shapeType, Shapes shapes, ref RigidPose pose, in Vector3 color)
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
                        spheres.Add(instance, pool);
                    }
                    break;
                case Capsule.Id:
                    {
                        CapsuleInstance instance;
                        instance.Position = pose.Position;
                        ref var capsule = ref Unsafe.AsRef<Capsule>(shapeData);
                        instance.Radius = capsule.Radius;
                        instance.HalfLength = capsule.HalfLength;
                        instance.PackedOrientation = Helpers.PackOrientationU64(ref pose.Orientation);
                        instance.PackedColor = Helpers.PackColor(color);
                        capsules.Add(instance, pool);
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
                        boxes.Add(instance, pool);
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
                        instance.PackedOrientation = Helpers.PackOrientationU64(ref pose.Orientation);
                        instance.X = pose.Position.X;
                        instance.Y = pose.Position.Y;
                        instance.Z = pose.Position.Z;
                        triangles.Add(instance, pool);
                    }
                    break;
                case Cylinder.Id:
                    {
                        CylinderInstance instance;
                        instance.Position = pose.Position;
                        ref var cylinder = ref Unsafe.AsRef<Cylinder>(shapeData);
                        instance.Radius = cylinder.Radius;
                        instance.HalfLength = cylinder.HalfLength;
                        instance.PackedOrientation = Helpers.PackOrientationU64(ref pose.Orientation);
                        instance.PackedColor = Helpers.PackColor(color);
                        cylinders.Add(instance, pool);
                    }
                    break;
                case ConvexHull.Id:
                    {
                        ref var hull = ref Unsafe.AsRef<ConvexHull>(shapeData);
                        MeshInstance instance;
                        instance.Position = pose.Position;
                        instance.PackedColor = Helpers.PackColor(color);
                        instance.PackedOrientation = Helpers.PackOrientationU64(ref pose.Orientation);
                        instance.Scale = Vector3.One;
                        var id = (ulong)hull.Points.Memory ^ (ulong)hull.Points.Length;
                        if (!MeshCache.TryGetExistingMesh(id, out instance.VertexStart, out var vertices))
                        {
                            int triangleCount = 0;
                            for (int i = 0; i < hull.FaceToVertexIndicesStart.Length; ++i)
                            {
                                hull.GetVertexIndicesForFace(i, out var faceVertexIndices);
                                triangleCount += faceVertexIndices.Length - 2;
                            }
                            instance.VertexCount = triangleCount * 3;
                            MeshCache.Allocate(id, instance.VertexCount, out instance.VertexStart, out vertices);
                            //This is a fresh allocation, so we need to upload vertex data.
                            int targetVertexIndex = 0;
                            for (int i = 0; i < hull.FaceToVertexIndicesStart.Length; ++i)
                            {
                                hull.GetVertexIndicesForFace(i, out var faceVertexIndices);
                                hull.GetPoint(faceVertexIndices[0], out var faceOrigin);
                                hull.GetPoint(faceVertexIndices[1], out var previousEdgeEnd);
                                for (int j = 2; j < faceVertexIndices.Length; ++j)
                                {
                                    vertices[targetVertexIndex++] = faceOrigin;
                                    vertices[targetVertexIndex++] = previousEdgeEnd;
                                    hull.GetPoint(faceVertexIndices[j], out previousEdgeEnd);
                                    vertices[targetVertexIndex++] = previousEdgeEnd;

                                }
                            }
                        }
                        else
                        {
                            instance.VertexCount = vertices.Length;
                        }
                        meshes.Add(instance, pool);
                    }
                    break;
                case Compound.Id:
                    {
                        AddCompoundChildren(ref Unsafe.AsRef<Compound>(shapeData).Children, shapes, pose, color);
                    }
                    break;
                case BigCompound.Id:
                    {
                        AddCompoundChildren(ref Unsafe.AsRef<BigCompound>(shapeData).Children, shapes, pose, color);
                    }
                    break;
                case Mesh.Id:
                    {
                        ref var mesh = ref Unsafe.AsRef<Mesh>(shapeData);
                        MeshInstance instance;
                        instance.Position = pose.Position;
                        instance.PackedColor = Helpers.PackColor(color);
                        instance.PackedOrientation = Helpers.PackOrientationU64(ref pose.Orientation);
                        instance.Scale = mesh.Scale;
                        var id = (ulong)mesh.Triangles.Memory ^ (ulong)mesh.Triangles.Length;
                        instance.VertexCount = mesh.Triangles.Length * 3;
                        if (MeshCache.Allocate(id, instance.VertexCount, out instance.VertexStart, out var vertices))
                        {
                            //This is a fresh allocation, so we need to upload vertex data.
                            for (int i = 0; i < mesh.Triangles.Length; ++i)
                            {
                                ref var triangle = ref mesh.Triangles[i];
                                var baseVertexIndex = i * 3;
                                //Note winding flip for rendering.
                                vertices[baseVertexIndex] = triangle.A;
                                vertices[baseVertexIndex + 1] = triangle.C;
                                vertices[baseVertexIndex + 2] = triangle.B;
                            }
                        }
                        meshes.Add(instance, pool);
                    }
                    break;
            }
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void AddShape(Shapes shapes, TypedIndex shapeIndex, ref RigidPose pose, in Vector3 color)
        {
            if (shapeIndex.Exists)
            {
                shapes[shapeIndex.Type].GetShapeData(shapeIndex.Index, out var shapeData, out _);
                AddShape(shapeData, shapeIndex.Type, shapes, ref pose, color);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void AddShape<TShape>(TShape shape, Shapes shapes, ref RigidPose pose, in Vector3 color) where TShape : IShape
        {
            AddShape(Unsafe.AsPointer(ref shape), shape.TypeId, shapes, ref pose, color);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void AddBodyShape(Shapes shapes, Bodies bodies, int setIndex, int indexInSet)
        {
            ref var set = ref bodies.Sets[setIndex];
            var handle = set.IndexToHandle[indexInSet];
            //Body color is based on three factors:
            //1) Handle as a hash seed that is unpacked into a color
            //2) Dynamics vs kinematic state
            //3) Activity state
            //The handle is hashed to get variation.
            ref var activity = ref set.Activity[indexInSet];
            ref var inertia = ref set.LocalInertias[indexInSet];
            Vector3 color;
            Helpers.UnpackColor((uint)HashHelper.Rehash(handle), out var colorVariation);
            if (Bodies.IsKinematic(inertia))
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

            AddShape(shapes, set.Collidables[indexInSet].Shape, ref set.Poses[indexInSet], color);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void AddStaticShape(Shapes shapes, Statics statics, int index)
        {
            var handle = statics.IndexToHandle[index];
            //Statics don't have any activity states. Just some simple variation on a central static color.
            Helpers.UnpackColor((uint)HashHelper.Rehash(handle), out var colorVariation);
            var staticBase = new Vector3(0.1f, 0.057f, 0.014f);
            var staticVariationSpan = new Vector3(0.07f, 0.07f, 0.03f);
            var color = staticBase + staticVariationSpan * colorVariation;
            AddShape(shapes, statics.Collidables[index].Shape, ref statics.Poses[index], color);
        }

        public void AddInstances(Simulation simulation, IThreadDispatcher threadDispatcher = null)
        {
            for (int i = 0; i < simulation.Bodies.Sets.Length; ++i)
            {
                ref var set = ref simulation.Bodies.Sets[i];
                if (set.Allocated) //Islands are stored noncontiguously; skip those which have been deallocated.
                {
                    for (int bodyIndex = 0; bodyIndex < set.Count; ++bodyIndex)
                    {
                        AddBodyShape(simulation.Shapes, simulation.Bodies, i, bodyIndex);
                    }
                }
            }
            for (int i = 0; i < simulation.Statics.Count; ++i)
            {
                AddStaticShape(simulation.Shapes, simulation.Statics, i);
            }
        }

        public void Dispose()
        {
            MeshCache.Dispose();
            spheres.Dispose(pool);
            capsules.Dispose(pool);
            cylinders.Dispose(pool);
            boxes.Dispose(pool);
            triangles.Dispose(pool);
            meshes.Dispose(pool);
        }
    }
}
