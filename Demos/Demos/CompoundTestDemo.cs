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

            using (var compoundBuilder = new CompoundBuilder(BufferPool, Simulation.Shapes, 8))
            {
                {
                    //Note that, in bepuphysics v2, there is no 'recentering' when constructing a shape. The pose you pass in for a child is exactly the pose that the compound will use, 
                    //even if the 'true' center of mass isn't at the local origin.
                    //Instead, if recentering is desired, it should performed ahead of time. The CompoundBuilder can help with this.
                    //We'll construct this compound using shapes far from the origin, and then use the CompoundBuilder overload that recenters the children and outputs the computed center.
                    var capsuleChildShape = new Capsule(0.5f, 0.5f);
                    var capsuleLocalPose = new RigidPose { Position = new Vector3(-0.5f, 4, 4), Orientation = BepuUtilities.Quaternion.Identity };
                    var boxChildShape = new Box(0.5f, 1f, 1.5f);
                    var boxLocalPose = new RigidPose { Position = new Vector3(0.5f, 4, 4), Orientation = BepuUtilities.Quaternion.Identity };

                    //All allocations from the buffer pool used for the final compound shape will be disposed when the demo is disposed. Don't have to worry about leaks in these demos.
                    compoundBuilder.Add(ref capsuleChildShape, ref capsuleLocalPose, 1);
                    compoundBuilder.Add(ref boxChildShape, ref boxLocalPose, 1);
                    compoundBuilder.BuildDynamicCompound(out var compoundChildren, out var compoundInertia, out var compoundCenter);
                    compoundBuilder.Reset();
                    var compound = new Compound(compoundChildren);
                    var compoundDescription = new BodyDescription
                    {
                        Activity = new BodyActivityDescription { SleepThreshold = 0.01f, MinimumTimestepCountUnderThreshold = 32 },
                        Collidable = new CollidableDescription
                        {
                            Shape = Simulation.Shapes.Add(ref compound),
                            SpeculativeMargin = 0.1f,
                        },
                        LocalInertia = compoundInertia,
                        Pose = new RigidPose { Position = compoundCenter, Orientation = BepuUtilities.Quaternion.Identity },
                    };

                    Simulation.Bodies.Add(ref compoundDescription);
                }

                //Build a stack of sphere grids to stress manifold reduction heuristics in a convex-ish situation.
                {
                    var gridShape = new Sphere(0.5f);
                    const float gridSpacing = 1.5f;
                    const int gridWidth = 3;
                    var gridShapeIndex = Simulation.Shapes.Add(ref gridShape);
                    gridShape.ComputeLocalInverseInertia(1, out var gridBoxInertia);
                    float localPoseOffset = -0.5f * gridSpacing * (gridWidth - 1);
                    for (int i = 0; i < gridWidth; ++i)
                    {
                        for (int j = 0; j < gridWidth; ++j)
                        {
                            var localPose = new RigidPose
                            {
                                Orientation = BepuUtilities.Quaternion.Identity,
                                Position = new Vector3(localPoseOffset, 0, localPoseOffset) + new Vector3(gridSpacing) * new Vector3(i, 0, j)
                            };
                            compoundBuilder.Add(gridShapeIndex, ref localPose, ref gridBoxInertia, 1);
                        }
                    }
                    compoundBuilder.BuildDynamicCompound(out var gridChildren, out var gridInertia, out var center);
                    compoundBuilder.Reset();
                    var gridCompound = new Compound(gridChildren);
                    var bodyDescription = new BodyDescription
                    {
                        Activity = new BodyActivityDescription { SleepThreshold = 0.0f, MinimumTimestepCountUnderThreshold = 32 },
                        Collidable = new CollidableDescription
                        {
                            Shape = Simulation.Shapes.Add(ref gridCompound),
                            SpeculativeMargin = 0.1f,
                        },
                        LocalInertia = gridInertia,
                        Pose = new RigidPose { Orientation = BepuUtilities.Quaternion.Identity }
                    };

                    for (int i = 0; i < 4; ++i)
                    {
                        bodyDescription.Pose.Position = new Vector3(0, 2 + i * 3, 0);
                        //if (i == 0)
                        //    gridDescription.LocalInertia = new BodyInertia();
                        //else
                        //    gridDescription.LocalInertia = gridInertia; 
                        Simulation.Bodies.Add(ref bodyDescription);
                    }
                }

                //Build a table and use it for a couple of different tests. 
                {
                    var legShape = new Box(0.2f, 1, 0.2f);
                    legShape.ComputeLocalInverseInertia(1f, out var legInverseInertia);
                    var legShapeIndex = Simulation.Shapes.Add(ref legShape);
                    var legPose0 = new RigidPose { Position = new Vector3(-1.5f, 0, -1.5f), Orientation = BepuUtilities.Quaternion.Identity };
                    var legPose1 = new RigidPose { Position = new Vector3(-1.5f, 0, 1.5f), Orientation = BepuUtilities.Quaternion.Identity };
                    var legPose2 = new RigidPose { Position = new Vector3(1.5f, 0, -1.5f), Orientation = BepuUtilities.Quaternion.Identity };
                    var legPose3 = new RigidPose { Position = new Vector3(1.5f, 0, 1.5f), Orientation = BepuUtilities.Quaternion.Identity };
                    compoundBuilder.Add(legShapeIndex, ref legPose0, ref legInverseInertia, 1);
                    compoundBuilder.Add(legShapeIndex, ref legPose1, ref legInverseInertia, 1);
                    compoundBuilder.Add(legShapeIndex, ref legPose2, ref legInverseInertia, 1);
                    compoundBuilder.Add(legShapeIndex, ref legPose3, ref legInverseInertia, 1);
                    var tableTopPose = new RigidPose { Position = new Vector3(0, 0.6f, 0), Orientation = BepuUtilities.Quaternion.Identity };
                    var tableTopShape = new Box(3.2f, 0.2f, 3.2f);
                    compoundBuilder.Add(ref tableTopShape, ref tableTopPose, 3);

                    compoundBuilder.BuildDynamicCompound(out var tableChildren, out var tableInertia, out var tableCenter);
                    compoundBuilder.Reset();
                    var table = new Compound(tableChildren);
                    var tableDescription = new BodyDescription
                    {
                        Activity = new BodyActivityDescription { SleepThreshold = 0.0f, MinimumTimestepCountUnderThreshold = 32 },
                        Collidable = new CollidableDescription
                        {
                            Shape = Simulation.Shapes.Add(ref table),
                            SpeculativeMargin = 0.1f,
                        },
                        LocalInertia = tableInertia,
                        Pose = new RigidPose {  Orientation = BepuUtilities.Quaternion.Identity }
                    };

                    //Drop one table by itself.
                    {
                        tableDescription.Pose.Position = new Vector3(10, 3, 10);
                        Simulation.Bodies.Add(ref tableDescription);
                    }

                    //Put a table on top of a sphere to stress out nonconvex reduction for divergent normals.
                    {
                        tableDescription.Pose.Position = new Vector3(10, 6, 0);
                        Simulation.Bodies.Add(ref tableDescription);

                        var sphereShape = new Sphere(3);
                        var sphereIndex = Simulation.Shapes.Add(ref sphereShape);
                        var sphereDescription = new StaticDescription
                        {
                            Collidable = new CollidableDescription
                            {
                                Shape = sphereIndex,
                                SpeculativeMargin = 0.1f,
                            },
                            Pose = new RigidPose { Position = new Vector3(10, 2, 0), Orientation = BepuUtilities.Quaternion.Identity }
                        };
                        Simulation.Statics.Add(ref sphereDescription);
                    }

                    //Put another table on the ground, but with a clamp-ish thing on it that generates opposing normals.
                    {
                        tableDescription.Pose.Position = new Vector3(10, 3, -10);
                        Simulation.Bodies.Add(ref tableDescription);

                        var clampPieceShape = new Box(2f, 0.1f, 0.3f);
                        clampPieceShape.ComputeLocalInverseInertia(1f, out var clampPieceInverseInertia);
                        var clampPieceShapeIndex = Simulation.Shapes.Add(ref clampPieceShape);
                        var clamp0 = new RigidPose { Position = new Vector3(0, -0.2f, -1.3f), Orientation = BepuUtilities.Quaternion.Identity };
                        var clamp1 = new RigidPose { Position = new Vector3(0, 0.2f, -1.3f), Orientation = BepuUtilities.Quaternion.Identity };
                        var clamp2 = new RigidPose { Position = new Vector3(0, -0.2f, 0), Orientation = BepuUtilities.Quaternion.Identity };
                        var clamp3 = new RigidPose { Position = new Vector3(0, 0.2f, 0), Orientation = BepuUtilities.Quaternion.Identity };
                        var clamp4 = new RigidPose { Position = new Vector3(0, -0.2f, 1.3f), Orientation = BepuUtilities.Quaternion.Identity };
                        var clamp5 = new RigidPose { Position = new Vector3(0, 0.2f, 1.3f), Orientation = BepuUtilities.Quaternion.Identity };
                        compoundBuilder.Add(clampPieceShapeIndex, ref clamp0, ref clampPieceInverseInertia, 1);
                        compoundBuilder.Add(clampPieceShapeIndex, ref clamp1, ref clampPieceInverseInertia, 1);
                        compoundBuilder.Add(clampPieceShapeIndex, ref clamp2, ref clampPieceInverseInertia, 1);
                        compoundBuilder.Add(clampPieceShapeIndex, ref clamp3, ref clampPieceInverseInertia, 1);
                        compoundBuilder.Add(clampPieceShapeIndex, ref clamp4, ref clampPieceInverseInertia, 1);
                        compoundBuilder.Add(clampPieceShapeIndex, ref clamp5, ref clampPieceInverseInertia, 1);

                        compoundBuilder.BuildDynamicCompound(out var clampChildren, out var clampInertia, out var clampCenter);
                        compoundBuilder.Reset();
                        var clamp = new Compound(clampChildren);
                        var clampDescription = new BodyDescription
                        {
                            Activity = new BodyActivityDescription { SleepThreshold = 0.0f, MinimumTimestepCountUnderThreshold = 32 },
                            Collidable = new CollidableDescription
                            {
                                Shape = Simulation.Shapes.Add(ref clamp),
                                SpeculativeMargin = 0.1f,
                            },
                            LocalInertia = tableInertia,
                            Pose = new RigidPose { Position = tableDescription.Pose.Position + new Vector3(2f, 0.3f, 0), Orientation = BepuUtilities.Quaternion.Identity }
                        };
                        Simulation.Bodies.Add(ref clampDescription);
                    }

                }
            }
            
            //Prevent stuff from falling into the infinite void.
            {
                var boxShape = new Box(100, 1, 100);
                var groundShapeIndex = Simulation.Shapes.Add(ref boxShape);
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
}
