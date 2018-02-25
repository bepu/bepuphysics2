using BepuUtilities;
using DemoRenderer;
using DemoUtilities;
using BepuPhysics;
using BepuPhysics.Collidables;
using System;
using System.Numerics;
using System.Diagnostics;
using BepuUtilities.Memory;
using BepuUtilities.Collections;

namespace Demos
{
    public class BoxTestDemo : Demo
    {
        public unsafe override void Initialize(Camera camera)
        {
            camera.Position = new Vector3(-20, 10, -20);
            //camera.Yaw = MathHelper.Pi ; 
            camera.Yaw = MathHelper.Pi * 3f / 4;
            //camera.Pitch = MathHelper.PiOver2 * 0.999f;
            Simulation = Simulation.Create(BufferPool, new TestCallbacks());

            var shape = new Box(1f, 3f, 2f);
            BodyInertia localInertia;
            localInertia.InverseMass = 1f;
            shape.ComputeLocalInverseInertia(localInertia.InverseMass, out localInertia.InverseInertiaTensor);
            //capsuleInertia.InverseInertiaTensor = new Triangular3x3();
            var shapeIndex = Simulation.Shapes.Add(ref shape);
            const int width = 8;
            const int height = 16;
            const int length = 8;
            var latticeSpacing = 1.0f;
            var latticeOffset = 0;// -0.5f * width * latticeSpacing;
            SimulationSetup.BuildLattice(
                new RegularGridBuilder(new Vector3(1, 3, 2), new Vector3(latticeOffset, 1.5f, latticeOffset), localInertia, shapeIndex),
                new ConstraintlessLatticeBuilder(),
                width, height, length, Simulation, out var bodyHandles, out var constraintHandles);
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);
            Simulation.Deterministic = false;
            //Simulation.Bodies.ActiveSet.Velocities[0].Linear = new Vector3(-2, 0, 0);
            //Simulation.Solver.IterationCount = 100;

            //var sphere = new Sphere(0.5f);
            //var sphereShapeIndex = Simulation.Shapes.Add(ref sphere);
            //BodyInertia sphereLocalInertia;
            //sphereLocalInertia.InverseMass = 1;
            //sphere.ComputeLocalInverseInertia(sphereLocalInertia.InverseMass, out sphereLocalInertia.InverseInertiaTensor);
            //for (int j = 0; j < 16; ++j)
            //{
            //    var bodyDescription = new BodyDescription
            //    {
            //        Pose = new RigidPose
            //        {
            //            Position = new Vector3(-16, 0.5f + j, 0),
            //            Orientation = BepuUtilities.Quaternion.Identity
            //        },
            //        LocalInertia = sphereLocalInertia,
            //        Collidable = new CollidableDescription
            //        {
            //            Continuity = new ContinuousDetectionSettings(),
            //            SpeculativeMargin = 0.04f,
            //            Shape = sphereShapeIndex
            //        },
            //        Activity = new BodyActivityDescription
            //        {
            //            SleepThreshold = -.1f,
            //            MinimumTimestepCountUnderThreshold = 32
            //        }
            //    };
            //    Simulation.Bodies.Add(ref bodyDescription);
            //}

            var staticShape = new Box(1, 1, 1);
            var staticShapeIndex = Simulation.Shapes.Add(ref staticShape);
            const int staticGridWidth = 100;
            const float staticSpacing = 1.2f;
            var gridOffset = -0.5f * staticGridWidth * staticSpacing;
            for (int i = 0; i < staticGridWidth; ++i)
            {
                for (int j = 0; j < staticGridWidth; ++j)
                {
                    var staticDescription = new StaticDescription
                    {
                        Collidable = new CollidableDescription
                        {
                            Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                            Shape = staticShapeIndex,
                            SpeculativeMargin = 0.1f
                        },
                        Pose = new RigidPose
                        {
                            Position = new Vector3(
                                1 + gridOffset + i * staticSpacing,
                                -0.5f,
                                0.5f + gridOffset + j * staticSpacing),
                            //Orientation = BepuUtilities.Quaternion.Identity
                            Orientation = BepuUtilities.Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1 + i, i * j % 10, -10 + -j)), (i ^ j) * 0.5f * (MathHelper.PiOver4))
                            //Orientation = BepuUtilities.Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(0, 0, 1)), MathHelper.Pi)
                        }
                    };
                    Simulation.Statics.Add(ref staticDescription);
                }
            }

        }
        public override void Update(Input input, float dt)
        {
            if (input.WasPushed(OpenTK.Input.Key.P))
                Console.WriteLine("asd");
            base.Update(input, dt);
        }

    }
}


