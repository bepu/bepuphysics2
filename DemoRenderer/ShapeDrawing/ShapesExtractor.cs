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

        ParallelLooper looper;
        public ShapesExtractor(ParallelLooper looper, int initialCapacityPerShapeType = 1024)
        {
            QuickList<SphereInstance, Array<SphereInstance>>.Create(new PassthroughArrayPool<SphereInstance>(), initialCapacityPerShapeType, out spheres);
            QuickList<CapsuleInstance, Array<CapsuleInstance>>.Create(new PassthroughArrayPool<CapsuleInstance>(), initialCapacityPerShapeType, out capsules);
            QuickList<BoxInstance, Array<BoxInstance>>.Create(new PassthroughArrayPool<BoxInstance>(), initialCapacityPerShapeType, out boxes);
            this.looper = looper;
        }

        public void ClearInstances()
        {
            spheres.Count = 0;
            capsules.Count = 0;
            boxes.Count = 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void AddShape(Shapes shapes, TypedIndex shapeIndex, ref RigidPose pose, ref Vector3 color)
        {
            switch (shapeIndex.Type)
            {
                case Sphere.Id:
                    {
                        SphereInstance instance;
                        instance.Position = pose.Position;
                        instance.Radius = shapes.GetShape<Sphere>(shapeIndex.Index).Radius;
                        Helpers.PackOrientation(ref pose.Orientation, out instance.PackedOrientation);
                        instance.PackedColor = Helpers.PackColor(ref color);
                        spheres.Add(ref instance, new PassthroughArrayPool<SphereInstance>());
                    }
                    break;
                case Capsule.Id:
                    {
                        CapsuleInstance instance;
                        instance.Position = pose.Position;
                        ref var capsule = ref shapes.GetShape<Capsule>(shapeIndex.Index);
                        instance.Radius = capsule.Radius;
                        instance.HalfLength = capsule.HalfLength;
                        instance.PackedOrientation = Helpers.PackOrientationU64(ref pose.Orientation);
                        instance.PackedColor = Helpers.PackColor(ref color);
                        capsules.Add(ref instance, new PassthroughArrayPool<CapsuleInstance>());
                    }
                    break;
                case Box.Id:
                    {
                        BoxInstance instance;
                        instance.Position = pose.Position;
                        ref var box = ref shapes.GetShape<Box>(shapeIndex.Index);
                        instance.PackedColor = Helpers.PackColor(ref color);
                        instance.Orientation = pose.Orientation;
                        instance.HalfWidth = box.HalfWidth;
                        instance.HalfHeight = box.HalfHeight;
                        instance.HalfLength = box.HalfLength;
                        boxes.Add(ref instance, new PassthroughArrayPool<BoxInstance>());
                    }
                    break;
            }
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

            AddShape(shapes, set.Collidables[indexInSet].Shape, ref set.Poses[indexInSet], ref color);
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
            AddShape(shapes, statics.Collidables[index].Shape, ref statics.Poses[index], ref color);
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
