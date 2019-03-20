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

            const int pointCount = 16;
            var points = new QuickList<Vector3>(pointCount, BufferPool);
            points.Allocate(BufferPool) = new Vector3(0, 0, 0);
            points.Allocate(BufferPool) = new Vector3(0.5f, 0.5f, 0);
            points.Allocate(BufferPool) = new Vector3(0.5f, 0.5f, 0);
            points.Allocate(BufferPool) = new Vector3(0.5f, 0.5f, 0);
            points.Allocate(BufferPool) = new Vector3(1, 0, 0);
            points.Allocate(BufferPool) = new Vector3(0, 1, 0);
            //points.Allocate(BufferPool) = new Vector3(0, 0, 1);
            ConvexHullHelper.ComputeHull(points.Span.Slice(0, points.Count), BufferPool, out var hullData);
            points.Dispose(BufferPool);

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));
        }
    }
}
