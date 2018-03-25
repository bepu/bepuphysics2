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
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);

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
            var compoundDescription = new BodyDescription
            {
                Activity = new BodyActivityDescription { SleepThreshold = 0.01f, MinimumTimestepCountUnderThreshold = 32 },
                Collidable = new CollidableDescription
                {
                    Shape = Simulation.Shapes.Add(ref compound),
                    SpeculativeMargin = 0.1f,
                },
                LocalInertia = compoundInertia,
                Pose = new RigidPose { Orientation = BepuUtilities.Quaternion.Identity },
            };

            compoundBuilder.Reset();
            var gridBoxShape = new Sphere(0.5f);// Box(1, 1, 1);
            var gridBoxShapeIndex = Simulation.Shapes.Add(ref gridBoxShape);
            gridBoxShape.ComputeLocalInverseInertia(1, out var gridBoxInertia);
            //The compound builder takes non-inverse inertias as input. Not exactly the most user friendly thing in this use case; 
            //would be nice to improve the API without sacrificing directness of the codepath.
            Triangular3x3.SymmetricInvert(ref gridBoxInertia, out gridBoxInertia);
            const int gridCompoundWidth = 3;
            const float gridCompoundSpacing = 1.25f;
            const float localPoseOffset = -0.5f * gridCompoundSpacing * (gridCompoundWidth - 1);
            for (int i = 0; i < gridCompoundWidth; ++i)
            {
                for (int j = 0; j < gridCompoundWidth; ++j)
                {
                    var localPose = new RigidPose
                    {
                        Orientation = BepuUtilities.Quaternion.Identity,
                        Position = new Vector3(localPoseOffset, 0, localPoseOffset) + new Vector3(gridCompoundSpacing) * new Vector3(i, 0, j)
                    };
                    compoundBuilder.Add(gridBoxShapeIndex, ref localPose, 1, ref gridBoxInertia);
                }
            }
            compoundBuilder.BuildCompound(out var gridCompound, out var gridInertia);
            var gridDescription = new BodyDescription
            {
                Activity = new BodyActivityDescription { SleepThreshold = 0.00f, MinimumTimestepCountUnderThreshold = 32 },
                Collidable = new CollidableDescription
                {
                    Shape = Simulation.Shapes.Add(ref gridCompound),
                    SpeculativeMargin = 0.1f,
                },
                LocalInertia = gridInertia,
                Pose = new RigidPose { Orientation = BepuUtilities.Quaternion.Identity }
            };

            //We do, however, clean up the compound builder because we can.
            compoundBuilder.Dispose();


            for (int i = 0; i < 4; ++i)
            {
                gridDescription.Pose.Position = new Vector3(0, 2 + i * 3, 0);
                //if (i == 0)
                //    gridDescription.LocalInertia = new BodyInertia();
                //else
                //    gridDescription.LocalInertia = gridInertia; 
                Simulation.Bodies.Add(ref gridDescription);
            }


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
