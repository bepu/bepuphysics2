using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using DemoUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace Demos.Demos
{
    public class DancerDemo : Demo
    {
        struct DollBodyHandles
        {
            public BodyHandle Hips;
            public BodyHandle Abdomen;
            public BodyHandle Chest;
            public BodyHandle Head;
            public BodyHandle UpperLeftLeg;
            public BodyHandle LowerLeftLeg;
            public BodyHandle UpperRightLeg;
            public BodyHandle LowerRightLeg;
            public BodyHandle UpperLeftArm;
            public BodyHandle LowerLeftArm;
            public BodyHandle UpperRightArm;
            public BodyHandle LowerRightArm;
        }

        struct Dancer
        {
            //Each dancer has its own simulation. The goal is to show a common use case for cosmetic physics- single threaded simulations that don't interact with the main simulation, running in parallel with other simulations.
            public Simulation Simulation;
            //The body handles are cached in each simulation so the source states can be mapped onto each dancer.
            public DollBodyHandles BodyHandles;
        }

        DollBodyHandles sourceBodyHandles;
        Dancer[] dancers;


        struct Control
        {
            public ConstraintHandle Servo;
            public Vector3 LocalOffset;
            public ServoSettings ServoSettings;
            public SpringSettings SpringSettings;

            public Control(Simulation simulation, BodyHandle body, Vector3 worldControlPoint, ServoSettings servoSettings, SpringSettings springSettings)
            {

                ServoSettings = servoSettings;
                SpringSettings = springSettings;
                var pose = simulation.Bodies[body].Pose;
                LocalOffset = QuaternionEx.Transform(worldControlPoint - pose.Position, Quaternion.Conjugate(pose.Orientation));
                Servo = simulation.Solver.Add(body, new OneBodyLinearServo { ServoSettings = servoSettings, SpringSettings = springSettings, LocalOffset = LocalOffset, Target = worldControlPoint });
            }

            public void UpdateTarget(Simulation simulation, Vector3 target)
            {
                simulation.Solver.ApplyDescription(Servo, new OneBodyLinearServo { ServoSettings = ServoSettings, SpringSettings = SpringSettings, LocalOffset = LocalOffset, Target = target });
            }

        }
        Control hipsControl;
        Control leftFootControl;
        Control rightFootControl;
        Control leftHandControl;
        Control rightHandControl;




        static void Connect(Simulation simulation, BodyHandle a, BodyHandle b, Vector3 jointLocation, SpringSettings springSettings, ref SubgroupCollisionFilter filterA, ref SubgroupCollisionFilter filterB)
        {
            var poseA = simulation.Bodies[a].Pose;
            var poseB = simulation.Bodies[b].Pose;
            simulation.Solver.Add(a, b,
                new BallSocket
                {
                    LocalOffsetA = QuaternionEx.Transform(jointLocation - poseA.Position, Quaternion.Conjugate(poseA.Orientation)),
                    LocalOffsetB = QuaternionEx.Transform(jointLocation - poseB.Position, Quaternion.Conjugate(poseB.Orientation)),
                    SpringSettings = springSettings
                });
            SubgroupCollisionFilter.DisableCollision(ref filterA, ref filterB);
        }

        static void CreateLimb(Simulation simulation, CollidableProperty<SubgroupCollisionFilter> collisionFilters,
            BodyHandle body, BodyHandle upperLimb, BodyHandle lowerLimb,
            Vector3 bodyToUpperJointLocation, Vector3 upperToLowerJointLocation, SpringSettings springSettings,
            int groupIndex, int limbSubgroupIndexStart)
        {
            ref var bodyFilter = ref collisionFilters[body];
            ref var upperFilter = ref collisionFilters.Allocate(upperLimb);
            ref var lowerFilter = ref collisionFilters.Allocate(lowerLimb);
            upperFilter = new SubgroupCollisionFilter(groupIndex, limbSubgroupIndexStart);
            lowerFilter = new SubgroupCollisionFilter(groupIndex, limbSubgroupIndexStart + 1);
            Connect(simulation, body, upperLimb, bodyToUpperJointLocation, springSettings, ref bodyFilter, ref upperFilter);
            Connect(simulation, upperLimb, lowerLimb, upperToLowerJointLocation, springSettings, ref upperFilter, ref lowerFilter);
            //While this demo isn't trying to make a full ragdoll, it's useful to stop the joints from doing obviously gross stuff.
            var bodyPose = simulation.Bodies[body].Pose;
            var upperPose = simulation.Bodies[upperLimb].Pose;
            var lowerPose = simulation.Bodies[lowerLimb].Pose;
            //Prevent the hip from spinning 360 degrees.
            simulation.Solver.Add(body, upperLimb, new TwistServo
            {
                LocalBasisA = QuaternionEx.Concatenate(RagdollDemo.CreateBasis(new Vector3(0, -1, 0), new Vector3(0, 0, 1)), Quaternion.Conjugate(bodyPose.Orientation)),
                LocalBasisB = QuaternionEx.Concatenate(RagdollDemo.CreateBasis(new Vector3(0, -1, 0), new Vector3(0, 0, 1)), Quaternion.Conjugate(upperPose.Orientation)),
                ServoSettings = ServoSettings.Default,
                SpringSettings = new SpringSettings(30, 1)
            });
            //Stop knee from flopping every which way.
            simulation.Solver.Add(upperLimb, lowerLimb, new AngularHinge
            {
                LocalHingeAxisA = QuaternionEx.Transform(Vector3.UnitX, Quaternion.Conjugate(upperPose.Orientation)),
                LocalHingeAxisB = QuaternionEx.Transform(Vector3.UnitX, Quaternion.Conjugate(lowerPose.Orientation)),
                SpringSettings = new SpringSettings(30, 1)
            });
            //Prevent knee hyperextension.
            var swingLimit = new SwingLimit
            {
                AxisLocalA = QuaternionEx.Transform(new Vector3(0, 0, 1), QuaternionEx.Conjugate(upperPose.Orientation)),
                AxisLocalB = QuaternionEx.Transform(new Vector3(0, 1, 0), QuaternionEx.Conjugate(lowerPose.Orientation)),
                MaximumSwingAngle = MathF.PI * 0.4f,
                SpringSettings = new SpringSettings(15, 1),
            };
            simulation.Solver.Add(upperLimb, lowerLimb, swingLimit);

        }

        static void ConstrainOrientation(Simulation simulation, BodyHandle a, BodyHandle b)
        {
            simulation.Solver.Add(a, b, new AngularServo
            {
                ServoSettings = ServoSettings.Default,
                SpringSettings = new SpringSettings(6, 1),
                TargetRelativeRotationLocalA = Quaternion.Concatenate(simulation.Bodies[b].Pose.Orientation, Quaternion.Conjugate(simulation.Bodies[a].Pose.Orientation))
            });
        }

        const float legOffsetX = 0.135f;
        const float armOffsetX = 0.25f;
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 4, 10);
            camera.Yaw = 0;
            camera.Pitch = 0;

            var collisionFilters = new CollidableProperty<SubgroupCollisionFilter>();
            //Note very high damping on the main ragdoll simulation; makes it easier to pose.
            Simulation = Simulation.Create(BufferPool, new SubgroupFilteredCallbacks { CollisionFilters = collisionFilters }, new DemoPoseIntegratorCallbacks(new Vector3(0, 0, 0), 0.95f, 0.95f), new SolveDescription(8, 1));

            var hipsPosition = new Vector3(0, 0, 0);
            var abdomenPosition = hipsPosition + new Vector3(0, 0.25f, 0);
            var chestPosition = abdomenPosition + new Vector3(0, 0.25f, 0);
            var headPosition = chestPosition + new Vector3(0, 0.4f, 0);
            //It's helpful to have joint locations for the limbs so we can create capsules from the endpoints.
            //There's not going to be a foot or hand body in this demo, since those aren't important for scooting a dress around.
            var kneePosition = hipsPosition - new Vector3(0, 0.5f, 0);
            var anklePosition = kneePosition - new Vector3(0, 0.5f, 0);
            var elbowPosition = chestPosition + new Vector3(0, 0.39f, 0);
            var wristPosition = elbowPosition + new Vector3(0, 0.39f, 0);
            var armOffset = new Vector3(armOffsetX, 0, 0);
            var legOffset = new Vector3(legOffsetX, 0, 0);
            const int groupIndex = 0;

            //Build the torso and head bodies.
            RagdollDemo.GetCapsuleForLineSegment(hipsPosition - legOffset, hipsPosition + legOffset, 0.14f, out var hipShape, out _, out var hipOrientation);
            sourceBodyHandles.Hips = Simulation.Bodies.Add(BodyDescription.CreateDynamic((hipsPosition, hipOrientation), hipShape.ComputeInertia(1), Simulation.Shapes.Add(hipShape), 0.01f));
            ref var hipsFilter = ref collisionFilters.Allocate(sourceBodyHandles.Hips);
            hipsFilter = new SubgroupCollisionFilter(groupIndex, 0);

            RagdollDemo.GetCapsuleForLineSegment(abdomenPosition - legOffset * 0.8f, abdomenPosition + legOffset * 0.8f, 0.13f, out var abdomenShape, out _, out var abdomenOrientation);
            sourceBodyHandles.Abdomen = Simulation.Bodies.Add(BodyDescription.CreateDynamic((abdomenPosition, abdomenOrientation), abdomenShape.ComputeInertia(1), Simulation.Shapes.Add(abdomenShape), 0.01f));
            ref var abdomenFilter = ref collisionFilters.Allocate(sourceBodyHandles.Abdomen);
            abdomenFilter = new SubgroupCollisionFilter(groupIndex, 1);

            RagdollDemo.GetCapsuleForLineSegment(abdomenPosition - legOffset * 0.8f, abdomenPosition + legOffset * 0.8f, 0.165f, out var chestShape, out _, out var chestOrientation);
            sourceBodyHandles.Chest = Simulation.Bodies.Add(BodyDescription.CreateDynamic((chestPosition, chestOrientation), chestShape.ComputeInertia(1), Simulation.Shapes.Add(chestShape), 0.01f));
            ref var chestFilter = ref collisionFilters.Allocate(sourceBodyHandles.Chest);
            chestFilter = new SubgroupCollisionFilter(groupIndex, 2);

            var headShape = new Sphere(0.17f);
            sourceBodyHandles.Head = Simulation.Bodies.Add(BodyDescription.CreateDynamic(headPosition, headShape.ComputeInertia(1), Simulation.Shapes.Add(headShape), 1e-2f));
            ref var headFilter = ref collisionFilters.Allocate(sourceBodyHandles.Head);
            headFilter = new SubgroupCollisionFilter(groupIndex, 3);

            var springSettings = new SpringSettings(30, 1);
            Connect(Simulation, sourceBodyHandles.Hips, sourceBodyHandles.Abdomen, 0.5f * (hipsPosition + abdomenPosition), springSettings, ref hipsFilter, ref abdomenFilter);
            ConstrainOrientation(Simulation, sourceBodyHandles.Hips, sourceBodyHandles.Abdomen);
            Connect(Simulation, sourceBodyHandles.Abdomen, sourceBodyHandles.Chest, 0.5f * (abdomenPosition + chestPosition), springSettings, ref abdomenFilter, ref chestFilter);
            ConstrainOrientation(Simulation, sourceBodyHandles.Abdomen, sourceBodyHandles.Chest);
            Connect(Simulation, sourceBodyHandles.Chest, sourceBodyHandles.Head, 0.5f * (chestPosition + headPosition), springSettings, ref chestFilter, ref headFilter);
            ConstrainOrientation(Simulation, sourceBodyHandles.Chest, sourceBodyHandles.Head);

            //Create the legs.
            RagdollDemo.GetCapsuleForLineSegment(hipsPosition, kneePosition, 0.11f, out var upperLegShape, out var upperLegPosition, out var upperLegOrientation);
            RagdollDemo.GetCapsuleForLineSegment(kneePosition, anklePosition, 0.1f, out var lowerLegShape, out var lowerLegPosition, out var lowerLegOrientation);
            var upperLegDescription = BodyDescription.CreateDynamic((upperLegPosition, upperLegOrientation), upperLegShape.ComputeInertia(1), Simulation.Shapes.Add(upperLegShape), 0.01f);
            var lowerLegDescription = BodyDescription.CreateDynamic((lowerLegPosition, lowerLegOrientation), lowerLegShape.ComputeInertia(1), Simulation.Shapes.Add(lowerLegShape), 0.01f);

            upperLegDescription.Pose.Position -= legOffset;
            lowerLegDescription.Pose.Position -= legOffset;
            sourceBodyHandles.UpperLeftLeg = Simulation.Bodies.Add(upperLegDescription);
            sourceBodyHandles.LowerLeftLeg = Simulation.Bodies.Add(lowerLegDescription);
            upperLegDescription.Pose.Position += 2 * legOffset;
            lowerLegDescription.Pose.Position += 2 * legOffset;
            sourceBodyHandles.UpperRightLeg = Simulation.Bodies.Add(upperLegDescription);
            sourceBodyHandles.LowerRightLeg = Simulation.Bodies.Add(lowerLegDescription);

            CreateLimb(Simulation, collisionFilters, sourceBodyHandles.Hips, sourceBodyHandles.UpperLeftLeg, sourceBodyHandles.LowerLeftLeg, hipsPosition - legOffset, kneePosition - legOffset, springSettings, groupIndex, 4);
            CreateLimb(Simulation, collisionFilters, sourceBodyHandles.Hips, sourceBodyHandles.UpperRightLeg, sourceBodyHandles.LowerRightLeg, hipsPosition + legOffset, kneePosition + legOffset, springSettings, groupIndex, 6);

            //Create the arms.
            RagdollDemo.GetCapsuleForLineSegment(chestPosition, elbowPosition, 0.08f, out var upperArmShape, out var upperArmPosition, out var upperArmOrientation);
            RagdollDemo.GetCapsuleForLineSegment(elbowPosition, wristPosition, 0.075f, out var lowerArmShape, out var lowerArmPosition, out var lowerArmOrientation);
            var upperArmDescription = BodyDescription.CreateDynamic((upperArmPosition, upperArmOrientation), upperArmShape.ComputeInertia(1), Simulation.Shapes.Add(upperArmShape), 0.01f);
            var lowerArmDescription = BodyDescription.CreateDynamic((lowerArmPosition, lowerArmOrientation), lowerArmShape.ComputeInertia(1), Simulation.Shapes.Add(lowerArmShape), 0.01f);

            upperArmDescription.Pose.Position -= armOffset;
            lowerArmDescription.Pose.Position -= armOffset;
            sourceBodyHandles.UpperLeftArm = Simulation.Bodies.Add(upperArmDescription);
            sourceBodyHandles.LowerLeftArm = Simulation.Bodies.Add(lowerArmDescription);
            upperArmDescription.Pose.Position += 2 * armOffset;
            lowerArmDescription.Pose.Position += 2 * armOffset;
            sourceBodyHandles.UpperRightArm = Simulation.Bodies.Add(upperArmDescription);
            sourceBodyHandles.LowerRightArm = Simulation.Bodies.Add(lowerArmDescription);

            CreateLimb(Simulation, collisionFilters, sourceBodyHandles.Chest, sourceBodyHandles.UpperLeftArm, sourceBodyHandles.LowerLeftArm, chestPosition - armOffset, elbowPosition - armOffset, springSettings, groupIndex, 8);
            CreateLimb(Simulation, collisionFilters, sourceBodyHandles.Chest, sourceBodyHandles.UpperRightArm, sourceBodyHandles.LowerRightArm, chestPosition + armOffset, elbowPosition + armOffset, springSettings, groupIndex, 10);

            //Create controls.
            hipsControl = new Control(Simulation, sourceBodyHandles.Hips, hipsPosition, ServoSettings.Default, new SpringSettings(5, 1));
            Simulation.Solver.Add(sourceBodyHandles.Hips, new OneBodyAngularServo { ServoSettings = ServoSettings.Default, SpringSettings = new SpringSettings(30, 1), TargetOrientation = Simulation.Bodies[sourceBodyHandles.Hips].Pose.Orientation });

            var limbServoSettings = ServoSettings.Default;
            var limbSpringSettings = new SpringSettings(4, 1);
            leftFootControl = new Control(Simulation, sourceBodyHandles.LowerLeftLeg, anklePosition - legOffset, limbServoSettings, limbSpringSettings);
            rightFootControl = new Control(Simulation, sourceBodyHandles.LowerRightLeg, anklePosition + legOffset, limbServoSettings, limbSpringSettings);
            leftHandControl = new Control(Simulation, sourceBodyHandles.LowerLeftArm, wristPosition - armOffset, limbServoSettings, limbSpringSettings);
            rightHandControl = new Control(Simulation, sourceBodyHandles.LowerRightArm, wristPosition + armOffset, limbServoSettings, limbSpringSettings);


            Simulation.Statics.Add(new(new Vector3(0, -10.5f, 0), Simulation.Shapes.Add(new Box(1000, 1, 1000))));
        }

        double time;

        float Smoothstep(float v)
        {
            var v2 = v * v;
            return 3 * v2 - 2 * v2 * v;
        }

        Vector3 CreateLegTarget(float t)
        {
            var z = MathF.Cos(t * MathF.Tau);
            var zOffset = (Smoothstep(z * 0.5f + 0.5f) * 2 - 1) * 0.7f;
            var offset = 0.5f + 0.5f * MathF.Cos(MathF.PI + t * 4 * MathF.PI);
            var xOffset = -0.2f + 0.4f * offset;
            var yOffset = -0.7f + 0.2f * offset;
            return new Vector3(-xOffset - legOffsetX, yOffset, zOffset);
        }
        Vector3 CreateArmTarget(float t)
        {
            var z = MathF.Cos(t * MathF.Tau);
            var zOffset = (Smoothstep(z * 0.5f + 0.5f) * 2 - 1);
            var offset = 0.5f + 0.5f * MathF.Cos(MathF.PI + t * 4 * MathF.PI);
            var xOffset = -0.2f + 0.6f * offset;
            var yOffset = 0.9f - 0.2f * offset;
            return new Vector3(-xOffset - armOffsetX, yOffset, zOffset);
        }

        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            time += dt;
            var hipsTarget = new Vector3(0, 0, 3 * (float)Math.Sin(time / 4));
            hipsControl.UpdateTarget(Simulation, hipsTarget);
            const float stepDuration = 3;
            var scaledTime = time / stepDuration;
            var t = (float)(scaledTime - Math.Floor(scaledTime));
            var tOffset = (t + 0.5f) % 1;
            var leftFootLocalTarget = CreateLegTarget(t);
            var rightFootLocalTarget = CreateLegTarget(tOffset);
            rightFootLocalTarget.X *= -1;
            leftFootControl.UpdateTarget(Simulation, hipsTarget + leftFootLocalTarget);
            rightFootControl.UpdateTarget(Simulation, hipsTarget + rightFootLocalTarget);


            var leftArmLocalTarget = CreateArmTarget(tOffset);
            var rightArmLocalTarget = CreateArmTarget(t);
            rightArmLocalTarget.X *= -1;
            leftHandControl.UpdateTarget(Simulation, hipsTarget + leftArmLocalTarget);
            rightHandControl.UpdateTarget(Simulation, hipsTarget + rightArmLocalTarget);
            base.Update(window, camera, input, dt);
        }
    }
}
