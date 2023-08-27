using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Collections;
using DemoContentLoader;
using DemoRenderer;
using DemoUtilities;
using System;
using System.Numerics;

namespace Demos.SpecializedTests.Media
{
    public class ShrinkwrappedNewtsVideoDemo : Demo
    {
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(25f, 1.5f, 15f);
            camera.Yaw = 3 * MathHelper.Pi / 4;
            camera.Pitch = 0;// MathHelper.Pi * 0.15f;

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));

            var meshContent = content.Load<MeshContent>("Content\\newt.obj");

            //This is actually a pretty good example of how *not* to make a convex hull shape.
            //Generating it directly from a graphical data source tends to have way more surface complexity than needed,
            //and it tends to have a lot of near-but-not-quite-coplanar surfaces which can make the contact manifold less stable.
            //Prefer a simpler source with more distinct features, possibly created with an automated content-time tool.
            var points = new QuickList<Vector3>(meshContent.Triangles.Length * 3, BufferPool);
            for (int i = 0; i < meshContent.Triangles.Length; ++i)
            {
                ref var triangle = ref meshContent.Triangles[i];
                //resisting the urge to just reinterpret the memory
                points.AllocateUnsafely() = triangle.A * new Vector3(1, 1.5f, 1);
                points.AllocateUnsafely() = triangle.B * new Vector3(1, 1.5f, 1);
                points.AllocateUnsafely() = triangle.C * new Vector3(1, 1.5f, 1);
            }

            var newtHull = new ConvexHull(points.Span.Slice(points.Count), BufferPool, out _);
            var bodyDescription = BodyDescription.CreateConvexDynamic(RigidPose.Identity, 1, Simulation.Shapes, newtHull);
            Random random = new Random(5);
            var poseBounds = new BoundingBox { Min = new Vector3(-20, 1, 5), Max = new Vector3(20, 10, 50) };
            for (int i = 0; i < 512; ++i)
            {
                bodyDescription.Pose = TestHelpers.CreateRandomPose(random, poseBounds);
                Simulation.Bodies.Add(bodyDescription);
            }

            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -30, 250), Simulation.Shapes.Add(new Box(1000, 60, 500))));
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -60, 0), Simulation.Shapes.Add(new Box(1000, 1, 1000))));



            mesh = DemoMeshHelper.LoadModel(content, BufferPool, "Content\\newt.obj", new Vector3(1, 1.5f, 1));
            Simulation.Statics.Add(new StaticDescription(new Vector3(30, 0, 20), QuaternionEx.CreateFromAxisAngle(Vector3.UnitY, -3 * MathHelper.PiOver4), Simulation.Shapes.Add(mesh)));
        }

        Mesh mesh;

        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            if(input.WasPushed(OpenTK.Input.Key.Z))
            {
                mesh.Scale = new Vector3(30);
                Simulation.Statics.Add(new StaticDescription(new Vector3(70, 0, 50), QuaternionEx.CreateFromAxisAngle(Vector3.UnitY, -3.1f * MathHelper.PiOver4), Simulation.Shapes.Add(mesh)));
            }
            base.Update(window, camera, input, dt);
        }
    }
}
