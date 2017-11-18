using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using System;
using System.Collections.Generic;
using System.Text;

namespace DemoRenderer.Shapes
{
    public class ShapesExtractor
    {
        //For now, we only have spheres. Later, once other shapes exist, this will be responsible for bucketing the different shape types and when necessary caching shape models.
        internal QuickList<SphereInstance, Array<SphereInstance>> spheres;

        ParallelLooper looper;
        public ShapesExtractor(ParallelLooper looper, int initialCapacityPerShapeType = 1024)
        {
            var initialSpheresSpan = new Array<SphereInstance>(new SphereInstance[initialCapacityPerShapeType]);
            spheres = new QuickList<SphereInstance, Array<SphereInstance>>(ref initialSpheresSpan);
            this.looper = looper;
        }

        public void ClearInstances()
        {
            spheres.Count = 0;
        }

        void AddShape(Simulation simulation, TypedIndex shapeIndex, ref RigidPose pose)
        {
            switch (shapeIndex.Type)
            {
                case Sphere.Id:
                    {
                        SphereInstance instance;
                        instance.Position = pose.Position;
                        instance.Radius = simulation.Shapes.GetShape<Sphere>(shapeIndex.Index).Radius;
                        instance.Orientation = pose.Orientation;
                        spheres.Add(ref instance, new PassthroughArrayPool<SphereInstance>());
                    }
                    break;
            }
        }

        public void AddInstances(Simulation simulation, IThreadDispatcher threadDispatcher = null)
        {
            for (int i = 0; i < simulation.Bodies.Sets.Length; ++i)
            {
                ref var set = ref simulation.Bodies.Sets[i];
                if (set.Allocated) //Islands are stored noncontiguously; skip those which have been deallocated.
                {
                    //TODO: Would be nice to graphically distinguish inactive bodies from active bodies. All nonzero index sets are inactive, so it would be pretty easy.
                    //Just requires that the shader-provided shape information includes color data. Easy to do; 4 bytes sufficient, would be shared with handle-based per-body
                    //color variation.
                    for (int bodyIndex = 0; bodyIndex < set.Count; ++bodyIndex)
                    {
                        AddShape(simulation, set.Collidables[bodyIndex].Shape, ref set.Poses[bodyIndex]);
                    }
                }
            }
            for (int i = 0; i < simulation.Statics.Count; ++i)
            {
                AddShape(simulation, simulation.Statics.Collidables[i].Shape, ref simulation.Statics.Poses[i]);
            }
        }
    }
}
