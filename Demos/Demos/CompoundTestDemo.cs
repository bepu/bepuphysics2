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
using System.Runtime.CompilerServices;

namespace Demos
{
    public class CompoundTestDemo : Demo
    {
        public unsafe override void Initialize(Camera camera)
        {
            camera.Position = new Vector3(-3f, 3, -3f);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = MathHelper.Pi * 0.1f;
            Simulation = Simulation.Create(BufferPool, new TestCallbacks(),
            new SimulationAllocationSizes
            {
                Bodies = 1,
                ConstraintCountPerBodyEstimate = 1,
                Constraints = 1,
                ConstraintsPerTypeBatch = 1,
                Islands = 1,
                ShapesPerType = 1,
                Statics = 1
            });
            Simulation.PoseIntegrator.Gravity = new Vector3(0, 0, 0);

            //Note that, in bepuphysics v2, there is no 'recentering'. The pose you pass in for a child is exactly the pose that the compound will use, 
            //even if the 'true' center of mass isn't at the local origin. (You can do the recentering like before by computing the weighted center of mass of the children,
            //and then offsetting their local poses to match.)
            var capsuleChildShape = new Capsule(0.5f, 0.5f);
            var capsuleLocalPose = new RigidPose { Position = new Vector3(-0.5f, 0, 0), Orientation = BepuUtilities.Quaternion.Identity };
            var boxChildShape = new Box(0.5f, 1f, 1.5f);
            var boxLocalPose = new RigidPose { Position = new Vector3(0.5f, 0, 0), Orientation = BepuUtilities.Quaternion.Identity };

            //All allocations from the buffer pool used for the final compound shape will be disposed when the demo is disposed. Don't have to worry about leaks in these demos.
            var compoundBuilder = new CompoundBuilder(BufferPool, Simulation.Shapes, 8);
            compoundBuilder.Add(ref capsuleChildShape, ref capsuleLocalPose, 1);
            compoundBuilder.Add(ref boxChildShape, ref boxLocalPose, 1);
            compoundBuilder.BuildCompound(out var compound, out var compoundInertia);
            //We do, however, clean up the compound builder because we can.
            compoundBuilder.Dispose();

            var compoundIndex = Simulation.Shapes.Add(ref compound);
            var compoundDescription = new BodyDescription
            {
                Activity = new BodyActivityDescription { SleepThreshold = 0.01f, MinimumTimestepCountUnderThreshold = 32 },
                Collidable = new CollidableDescription
                {
                    Shape = compoundIndex,
                    SpeculativeMargin = 0.1f,
                },
                LocalInertia = compoundInertia,
                Pose = new RigidPose { Position = new Vector3(0, 10, 0), Orientation = BepuUtilities.Quaternion.Identity },
                Velocity = new BodyVelocity { Angular = new Vector3(1, 1, 1) }
            };
            Simulation.Bodies.Add(ref compoundDescription);

            boxChildShape = new Box(100, 1, 100);
            var groundShapeIndex = Simulation.Shapes.Add(ref boxChildShape);
            var groundDescription = new StaticDescription
            {
                Collidable = new CollidableDescription
                {
                    Shape = groundShapeIndex,
                    SpeculativeMargin = 0.1f,
                },
                Pose = new RigidPose { Position = new Vector3(0, 0, 0), Orientation = BepuUtilities.Quaternion.Identity }
            };
            Simulation.Statics.Add(ref groundDescription);
        }

    }
}
