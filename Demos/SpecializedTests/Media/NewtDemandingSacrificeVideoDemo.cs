using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuPhysics.Trees;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using Demos.Demos;
using DemoUtilities;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using Quaternion = BepuUtilities.Quaternion;

namespace Demos.SpecializedTests
{
    public class NewtDemandingSacrificeVideoDemo : Demo
    {
        BodyProperty<SubgroupCollisionFilter> filters;
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-32f, 20.5f, 61f);
            camera.Yaw = MathHelper.Pi * 0.3f;
            camera.Pitch = MathHelper.Pi * -0.05f;

            filters = new BodyProperty<SubgroupCollisionFilter>(BufferPool);
            Simulation = Simulation.Create(BufferPool, new SubgroupFilteredCallbacks() { CollisionFilters = filters }, new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));

            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -0.5f, 0), new CollidableDescription(Simulation.Shapes.Add(new Box(1500, 1, 1500)), 0.1f)));
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 10, 0), new CollidableDescription(Simulation.Shapes.Add(new Box(70, 20, 80)), 0.1f)));
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 7.5f, 0), new CollidableDescription(Simulation.Shapes.Add(new Box(80, 15, 90)), 0.1f)));
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 5, 0), new CollidableDescription(Simulation.Shapes.Add(new Box(90, 10, 100)), 0.1f)));
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 2.5f, 0), new CollidableDescription(Simulation.Shapes.Add(new Box(100, 5, 110)), 0.1f)));

            //High fidelity simulation isn't super important on this one.
            Simulation.Solver.IterationCount = 2;

            DemoMeshHelper.LoadModel(content, BufferPool, "Content\\newt.obj", new Vector3(30), out var mesh);
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 20, 0), Quaternion.CreateFromAxisAngle(Vector3.UnitY, 0), new CollidableDescription(Simulation.Shapes.Add(mesh), 0.1f)));
        }

        Random random = new Random(5);
        int ragdollIndex = 0;

        BodyVelocity GetRandomizedVelocity(in Vector3 linearVelocity)
        {
            return new BodyVelocity { Linear = linearVelocity, Angular = new Vector3(-20) + 40 * new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) };
        }

        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            var pose = PairDeterminismTests.CreateRandomPose(random, new BoundingBox
            {
                Min = new Vector3(-10, 5, 70),
                Max = new Vector3(10, 15, 70)
            });
            var linearVelocity = Vector3.Normalize(new Vector3(-2 + 4 * (float)random.NextDouble(), 31 + 4 * (float)random.NextDouble(), 50) - pose.Position) * 40;
            var handles = RagdollDemo.AddRagdoll(pose.Position, pose.Orientation, ragdollIndex++, filters, Simulation);
            //This could be done better, but...  ... .... ..........
            new BodyReference(handles.Hips, Simulation.Bodies).Velocity = GetRandomizedVelocity(linearVelocity);
            new BodyReference(handles.Abdomen, Simulation.Bodies).Velocity = GetRandomizedVelocity(linearVelocity);
            new BodyReference(handles.Chest, Simulation.Bodies).Velocity =GetRandomizedVelocity(linearVelocity);
            new BodyReference(handles.Head, Simulation.Bodies).Velocity = GetRandomizedVelocity(linearVelocity);
            new BodyReference(handles.LeftArm.UpperArm, Simulation.Bodies).Velocity = GetRandomizedVelocity(linearVelocity);
            new BodyReference(handles.LeftArm.LowerArm, Simulation.Bodies).Velocity = GetRandomizedVelocity(linearVelocity);
            new BodyReference(handles.LeftArm.Hand, Simulation.Bodies).Velocity = GetRandomizedVelocity(linearVelocity);
            new BodyReference(handles.RightArm.UpperArm, Simulation.Bodies).Velocity = GetRandomizedVelocity(linearVelocity);
            new BodyReference(handles.RightArm.LowerArm, Simulation.Bodies).Velocity = GetRandomizedVelocity(linearVelocity);
            new BodyReference(handles.RightArm.Hand, Simulation.Bodies).Velocity = GetRandomizedVelocity(linearVelocity);
            new BodyReference(handles.LeftLeg.UpperLeg, Simulation.Bodies).Velocity = GetRandomizedVelocity(linearVelocity);
            new BodyReference(handles.LeftLeg.LowerLeg, Simulation.Bodies).Velocity = GetRandomizedVelocity(linearVelocity);
            new BodyReference(handles.LeftLeg.Foot, Simulation.Bodies).Velocity = GetRandomizedVelocity(linearVelocity);
            new BodyReference(handles.RightLeg.UpperLeg, Simulation.Bodies).Velocity = GetRandomizedVelocity(linearVelocity);
            new BodyReference(handles.RightLeg.LowerLeg, Simulation.Bodies).Velocity = GetRandomizedVelocity(linearVelocity);
            new BodyReference(handles.RightLeg.Foot, Simulation.Bodies).Velocity = GetRandomizedVelocity(linearVelocity);

            base.Update(window, camera, input, dt);
        }

    }
}
