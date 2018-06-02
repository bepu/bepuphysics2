using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.Collidables;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace DemoRenderer.ShapeDrawing
{
    public class ShapesExtractor
    {
        //For now, we only have spheres. Later, once other shapes exist, this will be responsible for bucketing the different shape types and when necessary caching shape models.
        internal QuickList<SphereInstance, Array<SphereInstance>> spheres;
        internal QuickList<CapsuleInstance, Array<CapsuleInstance>> capsules;
        internal QuickList<BoxInstance, Array<BoxInstance>> boxes;
        internal QuickList<TriangleInstance, Array<TriangleInstance>> triangles;

        ParallelLooper looper;
        public ShapesExtractor(ParallelLooper looper, int initialCapacityPerShapeType = 1024)
        {
            QuickList<SphereInstance, Array<SphereInstance>>.Create(new PassthroughArrayPool<SphereInstance>(), initialCapacityPerShapeType, out spheres);
            QuickList<CapsuleInstance, Array<CapsuleInstance>>.Create(new PassthroughArrayPool<CapsuleInstance>(), initialCapacityPerShapeType, out capsules);
            QuickList<BoxInstance, Array<BoxInstance>>.Create(new PassthroughArrayPool<BoxInstance>(), initialCapacityPerShapeType, out boxes);
            QuickList<TriangleInstance, Array<TriangleInstance>>.Create(new PassthroughArrayPool<TriangleInstance>(), initialCapacityPerShapeType, out triangles);
            this.looper = looper;
        }

        public void ClearInstances()
        {
            spheres.Count = 0;
            capsules.Count = 0;
            boxes.Count = 0;
            triangles.Count = 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void AddShape(void* shapeData, int shapeType, Shapes shapes, ref RigidPose pose, in Vector3 color)
        {
            switch (shapeType)
            {
                case Sphere.Id:
                    {
                        SphereInstance instance;
                        instance.Position = pose.Position;
                        instance.Radius = Unsafe.AsRef<Sphere>(shapeData).Radius;
                        Helpers.PackOrientation(pose.Orientation, out instance.PackedOrientation);
                        instance.PackedColor = Helpers.PackColor(color);
                        spheres.Add(ref instance, new PassthroughArrayPool<SphereInstance>());
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
                        capsules.Add(ref instance, new PassthroughArrayPool<CapsuleInstance>());
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
                        boxes.Add(ref instance, new PassthroughArrayPool<BoxInstance>());
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
                        triangles.Add(ref instance, new PassthroughArrayPool<TriangleInstance>());
                    }
                    break;
                case Compound.Id:
                    {
                        ref var compound = ref Unsafe.AsRef<Compound>(shapeData);
                        for (int i = 0; i < compound.Children.Length; ++i)
                        {
                            ref var child = ref compound.Children[i];
                            Compound.GetWorldPose(child.LocalPose, pose, out var childPose);
                            AddShape(shapes, child.ShapeIndex, ref childPose, color);
                        }
                    }
                    break;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void AddShape(Shapes shapes, TypedIndex shapeIndex, ref RigidPose pose, in Vector3 color)
        {
            shapes[shapeIndex.Type].GetShapeData(shapeIndex.Index, out var shapeData, out _);
            AddShape(shapeData, shapeIndex.Type, shapes, ref pose, color);
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
            Vector3 color;
            Helpers.UnpackColor((uint)HashHelper.Rehash(handle), out var colorVariation);
            if (activity.Kinematic)
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
    }
}
