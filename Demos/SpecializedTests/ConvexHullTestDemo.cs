using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using BepuPhysics.Collidables;
using BepuUtilities.Collections;
using DemoContentLoader;
using DemoRenderer;
using BepuPhysics;

namespace Demos.SpecializedTests
{
    public class ConvexHullTestDemo : Demo
    {
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 5, 10);
            camera.Yaw = 0;
            camera.Pitch = 0;

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));

            const int pointCount = 16;
            var points = new QuickList<Vector3>(pointCount, BufferPool);
            //points.Allocate(BufferPool) = new Vector3(0, 0, 0);
            //points.Allocate(BufferPool) = new Vector3(1, 0, 0);
            //points.Allocate(BufferPool) = new Vector3(0, 1, 0);
            //points.Allocate(BufferPool) = new Vector3(0, 0, 1);
            //points.Allocate(BufferPool) = new Vector3(1, 1, 1);
            //points.Allocate(BufferPool) = new Vector3(-0.1f, 0.25f, 0.25f);
            //points.Allocate(BufferPool) = new Vector3(1, 0.25f, -0.1f);

            points.Allocate(BufferPool) = new Vector3(0, 0, 0);
            points.Allocate(BufferPool) = new Vector3(0, 0, 1);
            points.Allocate(BufferPool) = new Vector3(0, 0, 2);
            //points.Allocate(BufferPool) = new Vector3(0, 1, 1);
            //points.Allocate(BufferPool) = new Vector3(1, 0, 0);
            //points.Allocate(BufferPool) = new Vector3(1, 0, 1);
            //points.Allocate(BufferPool) = new Vector3(1, 1, 0);
            //points.Allocate(BufferPool) = new Vector3(1, 1, 1);
            points.Allocate(BufferPool) = new Vector3(2, 2, -2);
            points.Allocate(BufferPool) = new Vector3(2, -2, -2);
            //points.Allocate(BufferPool) = new Vector3(2, 0, -2);
            //var random = new Random(5);
            //for (int i = 0; i < pointCount; ++i)
            //{
            //    points.AllocateUnsafely() = new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble());
            //}

            var pointsBuffer = points.Span.Slice(0, points.Count);
            ConvexHullHelper.ComputeHull(pointsBuffer, BufferPool, out var hullData);
            points.Dispose(BufferPool);

            ConvexHullHelper.ProcessHull(pointsBuffer, hullData, BufferPool, out var hullShape);

            Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(0, 0, 0), default, new CollidableDescription(Simulation.Shapes.Add(hullShape), 0.1f), new BodyActivityDescription(0.01f)));
        }
    }
}
