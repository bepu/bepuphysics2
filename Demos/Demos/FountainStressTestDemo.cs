using BepuUtilities;
using DemoRenderer;
using DemoUtilities;
using BepuPhysics;
using BepuPhysics.Collidables;
using System;
using System.Numerics;
using BepuUtilities.Memory;
using BepuUtilities.Collections;
using System.Diagnostics;

namespace Demos
{
    public class FountainStressTestDemo : Demo
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
            Simulation.Deterministic = false;


            var staticShape = new Sphere(6);
            var staticShapeIndex = Simulation.Shapes.Add(ref staticShape);
            const int staticGridWidthInSpheres = 128;
            const float staticSpacing = 8;
            for (int i = 0; i < staticGridWidthInSpheres; ++i)
            {
                for (int j = 0; j < staticGridWidthInSpheres; ++j)
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
                            -staticGridWidthInSpheres * staticSpacing * 0.5f + i * staticSpacing,
                            -4 + 4 * (float)Math.Cos(i * 0.3) + 4 * (float)Math.Cos(j * 0.3),
                            -staticGridWidthInSpheres * staticSpacing * 0.5f + j * staticSpacing),
                            Orientation = BepuUtilities.Quaternion.Identity
                        }
                    };
                    Simulation.Statics.Add(ref staticDescription);
                }
            }

            //A bunch of kinematic balls do acrobatics as an extra stressor.
            var kinematicShape = new Sphere(8);
            var kinematicShapeIndex = Simulation.Shapes.Add(ref staticShape);
            var kinematicCount = 64;
            var anglePerKinematic = MathHelper.TwoPi / kinematicCount;
            var startingRadius = 256;
            kinematicHandles = new int[kinematicCount];
            for (int i = 0; i < kinematicCount; ++i)
            {
                var angle = anglePerKinematic * i;
                var description = new BodyDescription
                {
                    Collidable = new CollidableDescription
                    {
                        Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                        Shape = kinematicShapeIndex,
                        SpeculativeMargin = 0.1f
                    },
                    Pose = new RigidPose
                    {
                        Position = new Vector3(
                            startingRadius * (float)Math.Cos(angle),
                            0,
                            startingRadius * (float)Math.Sin(angle)),
                        Orientation = BepuUtilities.Quaternion.Identity
                    },
                    Activity = new BodyActivityDescription { DeactivationThreshold = 0, MinimumTimestepCountUnderThreshold = 4 },
                };
                kinematicHandles[i] = Simulation.Bodies.Add(ref description);
            }
        }

        double time;
        double t;
        int[] kinematicHandles;

        int frameIndex;
        public override void Update(Input input, float dt)
        {
            time += dt;

            //Occasionally, the animation stops completely. The resulting velocities will be zero, so the kinematics will have a chance to rest (testing kinematic rest states).
            var dip = 0.1;
            var progressionMultiplier = 0.5 - dip + (1 + dip) * 0.5 * Math.Cos(time * 0.25);
            if (progressionMultiplier < 0)
                progressionMultiplier = 0;
            t += dt * progressionMultiplier;

            var baseAngle = (float)(t * 0.015);
            var anglePerKinematic = MathHelper.TwoPi / kinematicHandles.Length;
            var maxDisplacement = 30 * dt;
            var inverseDt = 1f / dt;
            for (int i = 0; i < kinematicHandles.Length; ++i)
            {
                ref var bodyLocation = ref Simulation.Bodies.HandleToLocation[kinematicHandles[i]];

                ref var set = ref Simulation.Bodies.Sets[bodyLocation.SetIndex];
                var angle = anglePerKinematic * i;
                var positionAngle = baseAngle + angle;
                var radius = 128 + 32 * (float)Math.Cos(3 * (angle + t * (1f / 3f))) + 32 * (float)Math.Cos(t * (1f / 3f));
                var targetLocation = new Vector3(
                    radius * (float)Math.Cos(positionAngle),
                    16 + 16 * (float)Math.Cos(4 * (angle + t * 0.5)),
                    radius * (float)Math.Sin(positionAngle));

                var correction = targetLocation - set.Poses[bodyLocation.Index].Position;
                var distance = correction.Length();
                if (distance > 1e-4)
                {
                    if (bodyLocation.SetIndex > 0)
                    {
                        //We're requesting a nonzero velocity, so it must be active.
                        Simulation.Activator.ActivateSet(bodyLocation.SetIndex);
                    }
                    if (distance > maxDisplacement)
                    {
                        correction *= maxDisplacement / distance;
                    }
                    Debug.Assert(bodyLocation.SetIndex == 0);
                    Simulation.Bodies.ActiveSet.Velocities[bodyLocation.Index].Linear = correction * inverseDt;
                }
                else
                {
                    if(bodyLocation.SetIndex == 0)
                    {
                        Simulation.Bodies.ActiveSet.Velocities[bodyLocation.Index].Linear = new Vector3();
                    }
                }
            }
            base.Update(input, dt);

        }

    }
}
