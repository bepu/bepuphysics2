using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using Demos.Demos;
using DemoUtilities;
using System;
using System.Numerics;

namespace Demos.SpecializedTests.Media
{
    public class NewtDemandingSacrificeVideoDemo : Demo
    {
        CollidableProperty<SubgroupCollisionFilter> filters;
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-32f, 20.5f, 61f);
            camera.Yaw = MathHelper.Pi * 0.3f;
            camera.Pitch = MathHelper.Pi * -0.05f;

            filters = new CollidableProperty<SubgroupCollisionFilter>(BufferPool);
            Simulation = Simulation.Create(BufferPool, new SubgroupFilteredCallbacks(filters), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(4, 1));

            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -0.5f, 0), Simulation.Shapes.Add(new Box(1500, 1, 1500))));
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 10, 0), Simulation.Shapes.Add(new Box(70, 20, 80))));
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 7.5f, 0), Simulation.Shapes.Add(new Box(80, 15, 90))));
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 5, 0), Simulation.Shapes.Add(new Box(90, 10, 100))));
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 2.5f, 0), Simulation.Shapes.Add(new Box(100, 5, 110))));

            //High fidelity simulation isn't super important on this one.
            Simulation.Solver.VelocityIterationCount = 2;

            DemoMeshHelper.LoadModel(content, BufferPool, "Content\\newt.obj", new Vector3(30), out var mesh);
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 20, 0), QuaternionEx.CreateFromAxisAngle(Vector3.UnitY, 0), Simulation.Shapes.Add(mesh)));
        }

        Random random = new Random(5);
        int ragdollIndex = 0;

        BodyVelocity GetRandomizedVelocity(in Vector3 linearVelocity)
        {
            return new BodyVelocity { Linear = linearVelocity, Angular = new Vector3(-20) + 40 * new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle()) };
        }

        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            var pose = TestHelpers.CreateRandomPose(random, new BoundingBox
            {
                Min = new Vector3(-10, 5, 70),
                Max = new Vector3(10, 15, 70)
            });
            var linearVelocity = Vector3.Normalize(new Vector3(-2 + 4 * random.NextSingle(), 31 + 4 * random.NextSingle(), 50) - pose.Position) * 40;
            var handles = RagdollDemo.AddRagdoll(pose.Position, pose.Orientation, ragdollIndex++, filters, Simulation);
            var bodies = Simulation.Bodies;
            //This could be done better, but...  ... .... ..........
            bodies[handles.Hips].Velocity = GetRandomizedVelocity(linearVelocity);
            bodies[handles.Abdomen].Velocity = GetRandomizedVelocity(linearVelocity);
            bodies[handles.Chest].Velocity = GetRandomizedVelocity(linearVelocity);
            bodies[handles.Head].Velocity = GetRandomizedVelocity(linearVelocity);
            bodies[handles.LeftArm.UpperArm].Velocity = GetRandomizedVelocity(linearVelocity);
            bodies[handles.LeftArm.LowerArm].Velocity = GetRandomizedVelocity(linearVelocity);
            bodies[handles.LeftArm.Hand].Velocity = GetRandomizedVelocity(linearVelocity);
            bodies[handles.RightArm.UpperArm].Velocity = GetRandomizedVelocity(linearVelocity);
            bodies[handles.RightArm.LowerArm].Velocity = GetRandomizedVelocity(linearVelocity);
            bodies[handles.RightArm.Hand].Velocity = GetRandomizedVelocity(linearVelocity);
            bodies[handles.LeftLeg.UpperLeg].Velocity = GetRandomizedVelocity(linearVelocity);
            bodies[handles.LeftLeg.LowerLeg].Velocity = GetRandomizedVelocity(linearVelocity);
            bodies[handles.LeftLeg.Foot].Velocity = GetRandomizedVelocity(linearVelocity);
            bodies[handles.RightLeg.UpperLeg].Velocity = GetRandomizedVelocity(linearVelocity);
            bodies[handles.RightLeg.LowerLeg].Velocity = GetRandomizedVelocity(linearVelocity);
            bodies[handles.RightLeg.Foot].Velocity = GetRandomizedVelocity(linearVelocity);

            base.Update(window, camera, input, dt);
        }

    }
}
