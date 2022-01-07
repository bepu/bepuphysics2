using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Demos.Demos
{
    public class DancerDemo : Demo
    {
        [StructLayout(LayoutKind.Sequential, Pack = 4)]
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

            internal static unsafe Buffer<BodyHandle> AsBuffer(DollBodyHandles* sourceBodyHandles)
            {
                return new Buffer<BodyHandle>(sourceBodyHandles, 12);
            }
        }

        struct Dancer
        {
            //Each dancer has its own simulation. The goal is to show a common use case for cosmetic physics- single threaded simulations that don't interact with the main simulation, running in parallel with other simulations.
            public Simulation Simulation;
            //The body handles are cached in each simulation so the source states can be mapped onto each dancer.
            public DollBodyHandles BodyHandles;
        }

        DollBodyHandles sourceBodyHandles;
        ParallelLooper looper;
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

        static (int columnIndex, int rowIndex) GetRowAndColumnForDancer(int dancerIndex)
        {
            var rowIndex = dancerIndex / dancerGridWidth;
            return (dancerIndex - rowIndex * dancerGridWidth, rowIndex);
        }

        QuickQueue<MotionState> motionHistory;
        static Vector3 GetOffsetForDancer(int i)
        {
            const float spacing = 2;
            var (columnIndex, rowIndex) = GetRowAndColumnForDancer(i);
            return new Vector3(dancerGridWidth * spacing / -2 + (columnIndex + 0.5f) * spacing, 0, -2 + rowIndex * -spacing);
        }

        static BodyHandle[,] CreateBodyGrid(Vector3 position, int widthInNodes, float spacing, float bodyRadius, float massPerBody,
            int instanceId, Simulation simulation, CollidableProperty<ClothCollisionFilter> filters)
        {
            var description = BodyDescription.CreateDynamic(QuaternionEx.Identity, new BodyInertia { InverseMass = 1f / massPerBody }, simulation.Shapes.Add(new Sphere(bodyRadius)), 0.01f);
            BodyHandle[,] handles = new BodyHandle[widthInNodes, widthInNodes];
            var armHoleCenter = new Vector2(armOffsetX + 0.05f, 0);
            var armHoleRadius = 0.07f;
            var armHoleRadiusSquared = armHoleRadius * armHoleRadius;
            var halfWidth = widthInNodes * spacing / 2;
            var halfWidthSquared = halfWidth * halfWidth;
            var halfWidthOffset = new Vector2(halfWidth);
            for (int rowIndex = 0; rowIndex < widthInNodes; ++rowIndex)
            {
                for (int columnIndex = 0; columnIndex < widthInNodes; ++columnIndex)
                {
                    var horizontalPosition = new Vector2(columnIndex, rowIndex) * spacing - halfWidthOffset;
                    var distanceSquared0 = Vector2.DistanceSquared(horizontalPosition, armHoleCenter);
                    var distanceSquared1 = Vector2.DistanceSquared(horizontalPosition, -armHoleCenter);
                    var centerDistanceSquared = horizontalPosition.LengthSquared();
                    if (distanceSquared0 < armHoleRadiusSquared || distanceSquared1 < armHoleRadiusSquared || centerDistanceSquared > halfWidthSquared)
                    {
                        //Too close to an arm, don't create any bodies here.
                        handles[rowIndex, columnIndex] = new BodyHandle { Value = -1 };
                    }
                    else
                    {
                        description.Pose.Position = new Vector3(horizontalPosition.X, 0, horizontalPosition.Y) + position;
                        var handle = simulation.Bodies.Add(description);
                        handles[rowIndex, columnIndex] = handle;
                        if (filters != null)
                            filters.Allocate(handle) = new ClothCollisionFilter(rowIndex, columnIndex, instanceId);
                    }
                }
            }
            return handles;
        }

        static void CreateAreaConstraints(BodyHandle[,] bodyHandles, SpringSettings springSettings, Simulation simulation)
        {
            for (int rowIndex = 0; rowIndex < bodyHandles.GetLength(0) - 1; ++rowIndex)
            {
                for (int columnIndex = 0; columnIndex < bodyHandles.GetLength(1) - 1; ++columnIndex)
                {
                    var aHandle = bodyHandles[rowIndex, columnIndex];
                    var bHandle = bodyHandles[rowIndex + 1, columnIndex];
                    var cHandle = bodyHandles[rowIndex, columnIndex + 1];
                    var dHandle = bodyHandles[rowIndex + 1, columnIndex + 1];
                    //Only create a constraint if bodies on all sides of the quad actually exist.
                    //In this demo, we use -1 in the body handle slot to represent 'no body'.
                    if (aHandle.Value >= 0 && bHandle.Value >= 0 && cHandle.Value >= 0 && dHandle.Value >= 0)
                    {
                        var a = simulation.Bodies[aHandle];
                        var b = simulation.Bodies[bHandle];
                        var c = simulation.Bodies[cHandle];
                        var d = simulation.Bodies[dHandle];
                        simulation.Solver.Add(aHandle, bHandle, cHandle, new AreaConstraint(a.Pose.Position, b.Pose.Position, c.Pose.Position, springSettings));
                        simulation.Solver.Add(bHandle, cHandle, dHandle, new AreaConstraint(b.Pose.Position, c.Pose.Position, d.Pose.Position, springSettings));
                    }
                }
            }
        }
        static void CreateDistanceConstraints(BodyHandle[,] bodyHandles, SpringSettings springSettings, Simulation simulation)
        {
            void CreateConstraintBetweenBodies(BodyHandle aHandle, BodyHandle bHandle)
            {
                //Only create a constraint if bodies on both sides of the pair actually exist.
                //In this demo, we use -1 in the body handle slot to represent 'no body'.
                if (aHandle.Value >= 0 && bHandle.Value >= 0)
                {
                    var a = simulation.Bodies[aHandle];
                    var b = simulation.Bodies[bHandle];
                    simulation.Solver.Add(aHandle, bHandle, new CenterDistanceConstraint(Vector3.Distance(a.Pose.Position, b.Pose.Position), springSettings));
                }
            }
            for (int rowIndex = 0; rowIndex < bodyHandles.GetLength(0); ++rowIndex)
            {
                for (int columnIndex = 0; columnIndex < bodyHandles.GetLength(1) - 1; ++columnIndex)
                {
                    CreateConstraintBetweenBodies(bodyHandles[rowIndex, columnIndex], bodyHandles[rowIndex, columnIndex + 1]);
                }
            }
            for (int rowIndex = 0; rowIndex < bodyHandles.GetLength(0) - 1; ++rowIndex)
            {
                for (int columnIndex = 0; columnIndex < bodyHandles.GetLength(1); ++columnIndex)
                {
                    CreateConstraintBetweenBodies(bodyHandles[rowIndex, columnIndex], bodyHandles[rowIndex + 1, columnIndex]);
                }
            }
            for (int rowIndex = 0; rowIndex < bodyHandles.GetLength(0) - 1; ++rowIndex)
            {
                for (int columnIndex = 0; columnIndex < bodyHandles.GetLength(1) - 1; ++columnIndex)
                {
                    CreateConstraintBetweenBodies(bodyHandles[rowIndex, columnIndex], bodyHandles[rowIndex + 1, columnIndex + 1]);
                    CreateConstraintBetweenBodies(bodyHandles[rowIndex, columnIndex + 1], bodyHandles[rowIndex + 1, columnIndex]);
                }
            }
        }

        static void TailorDress(Simulation simulation, CollidableProperty<ClothCollisionFilter> filters, int dancerIndex)
        {
            var bodies = CreateBodyGrid(new Vector3(0, 0.8f, 0) + GetOffsetForDancer(dancerIndex), 40, 0.05f, 0.025f, 0.01f, dancerIndex, simulation, filters);
            CreateDistanceConstraints(bodies, new SpringSettings(30, 1), simulation);
        }
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 4, 10);
            camera.Yaw = 0;
            camera.Pitch = 0;

            var collisionFilters = new CollidableProperty<SubgroupCollisionFilter>();
            //Note very high damping on the main ragdoll simulation; makes it easier to pose.
            Simulation = Simulation.Create(BufferPool, new SubgroupFilteredCallbacks { CollisionFilters = collisionFilters }, new DemoPoseIntegratorCallbacks(new Vector3(0, 0, 0), 0, 0), new SolveDescription(8, 1));

            looper = new ParallelLooper() { Dispatcher = ThreadDispatcher };

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


            Simulation.Statics.Add(new(new Vector3(0, -1.24f, 0), Simulation.Shapes.Add(new Box(1000, 1, 1000))));

            //All the background dancers read different historical motion states so that they're not just all doing the exact same thing.
            //Keep the states in a queue. Each batch of 12 motion states is the state for a single frame.
            motionHistory = new QuickQueue<MotionState>(historyLength * 12, BufferPool);

            dancers = new Dancer[dancerCount];
            static BodyHandle CreateCopyForDancer(Simulation sourceSimulation, BodyHandle sourceHandle, TypedIndex shapeIndexInTargetSimulation, Simulation targetSimulation, int dancerIndex)
            {
                var description = sourceSimulation.Bodies.GetDescription(sourceHandle);
                description.Pose.Position += GetOffsetForDancer(dancerIndex);
                description.Collidable.Shape = shapeIndexInTargetSimulation;
                description.LocalInertia = default;
                return targetSimulation.Bodies.Add(description);
            }

            for (int i = 0; i < dancers.Length; ++i)
            {
                ref var dancer = ref dancers[i];
                var dancerFilters = new CollidableProperty<ClothCollisionFilter>();
                dancer.Simulation = Simulation.Create(new BufferPool(), new ClothCallbacks
                {
                    Filters = dancerFilters,
                    Material = new PairMaterialProperties
                    {
                        SpringSettings = new SpringSettings(120, 1),
                        FrictionCoefficient = .4f,
                        MaximumRecoveryVelocity = 20
                    }
                }, new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(1, 4));
                dancer.BodyHandles.Hips = CreateCopyForDancer(Simulation, sourceBodyHandles.Hips, dancer.Simulation.Shapes.Add(hipShape), dancer.Simulation, i);
                dancer.BodyHandles.Abdomen = CreateCopyForDancer(Simulation, sourceBodyHandles.Abdomen, dancer.Simulation.Shapes.Add(abdomenShape), dancer.Simulation, i);
                dancer.BodyHandles.Chest = CreateCopyForDancer(Simulation, sourceBodyHandles.Chest, dancer.Simulation.Shapes.Add(chestShape), dancer.Simulation, i);
                dancer.BodyHandles.Head = CreateCopyForDancer(Simulation, sourceBodyHandles.Head, dancer.Simulation.Shapes.Add(headShape), dancer.Simulation, i);

                var upperLegShapeInTarget = dancer.Simulation.Shapes.Add(upperLegShape);
                var lowerLegShapeInTarget = dancer.Simulation.Shapes.Add(lowerLegShape);
                dancer.BodyHandles.UpperLeftLeg = CreateCopyForDancer(Simulation, sourceBodyHandles.UpperLeftLeg, upperLegShapeInTarget, dancer.Simulation, i);
                dancer.BodyHandles.LowerLeftLeg = CreateCopyForDancer(Simulation, sourceBodyHandles.LowerLeftLeg, lowerLegShapeInTarget, dancer.Simulation, i);
                dancer.BodyHandles.UpperRightLeg = CreateCopyForDancer(Simulation, sourceBodyHandles.UpperRightLeg, upperLegShapeInTarget, dancer.Simulation, i);
                dancer.BodyHandles.LowerRightLeg = CreateCopyForDancer(Simulation, sourceBodyHandles.LowerRightLeg, lowerLegShapeInTarget, dancer.Simulation, i);

                var upperArmShapeInTarget = dancer.Simulation.Shapes.Add(upperArmShape);
                var lowerArmShapeInTarget = dancer.Simulation.Shapes.Add(lowerArmShape);
                dancer.BodyHandles.UpperLeftArm = CreateCopyForDancer(Simulation, sourceBodyHandles.UpperLeftArm, upperArmShapeInTarget, dancer.Simulation, i);
                dancer.BodyHandles.LowerLeftArm = CreateCopyForDancer(Simulation, sourceBodyHandles.LowerLeftArm, lowerArmShapeInTarget, dancer.Simulation, i);
                dancer.BodyHandles.UpperRightArm = CreateCopyForDancer(Simulation, sourceBodyHandles.UpperRightArm, upperArmShapeInTarget, dancer.Simulation, i);
                dancer.BodyHandles.LowerRightArm = CreateCopyForDancer(Simulation, sourceBodyHandles.LowerRightArm, lowerArmShapeInTarget, dancer.Simulation, i);

                TailorDress(dancer.Simulation, dancerFilters, i);
            }
        }

        const int dancerGridWidth = 6;
        const int dancerGridLength = 6;
        const int dancerCount = dancerGridWidth * dancerGridLength;
        const int historyLength = 256;
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
        unsafe void ExecuteDancer(int dancerIndex)
        {
            //Copy historical motion states to the dancers.
            ref var dancer = ref dancers[dancerIndex];
            var sourceHandleBuffer = DollBodyHandles.AsBuffer((DollBodyHandles*)Unsafe.AsPointer(ref sourceBodyHandles));
            //Delay is greater for the dancers that are further away, plus a little randomized component to desynchronize them.
            var (columnIndex, rowIndex) = GetRowAndColumnForDancer(dancerIndex);
            var offsetX = columnIndex - (dancerGridWidth / 2 - 0.5f);
            var distanceFromMainDancer = (int)MathF.Sqrt(offsetX * offsetX + rowIndex * rowIndex);
            var historicalStateStartIndex = motionHistory.Count - sourceHandleBuffer.Length * (distanceFromMainDancer * 8 + 1 + (HashHelper.Rehash(dancerIndex) & 0xF));
            if (historicalStateStartIndex < 0)
                historicalStateStartIndex = 0;
            var targetHandleBuffer = DollBodyHandles.AsBuffer((DollBodyHandles*)Unsafe.AsPointer(ref dancer.BodyHandles));
            for (int j = 0; j < sourceHandleBuffer.Length; ++j)
            {
                ref var targetMotionState = ref dancer.Simulation.Bodies[targetHandleBuffer[j]].MotionState;
                targetMotionState = motionHistory[historicalStateStartIndex + j];
                targetMotionState.Pose.Position += GetOffsetForDancer(dancerIndex);
            }
            //Update the simulation for the dancer.
            dancer.Simulation.Timestep(1 / 60f);
        }

        public unsafe override void Update(Window window, Camera camera, Input input, float dt)
        {
            time += 1 / 60f;
            var hipsTarget = new Vector3(0, 0, 3 * (float)Math.Sin(time / 4));
            hipsControl.UpdateTarget(Simulation, hipsTarget);
            const float stepDuration = 3.5f;
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

            //Record the latest motion state from the source dancer.
            var sourceHandleBuffer = DollBodyHandles.AsBuffer((DollBodyHandles*)Unsafe.AsPointer(ref sourceBodyHandles));
            if (motionHistory.Count == historyLength * sourceHandleBuffer.Length)
                for (int i = 0; i < sourceHandleBuffer.Length; ++i)
                    motionHistory.Dequeue();

            for (int i = 0; i < sourceHandleBuffer.Length; ++i)
            {
                motionHistory.EnqueueUnsafely(Simulation.Bodies[sourceHandleBuffer[i]].MotionState);
            }
            var startTime = Stopwatch.GetTimestamp();
            looper.For(0, dancers.Length, ExecuteDancer);
            var endTime = Stopwatch.GetTimestamp();
            var executionTime = (endTime - startTime) / (double)Stopwatch.Frequency;
            Console.WriteLine($"Time (ms): {executionTime * 1000}");
            Console.WriteLine($"Time per dancer, amortized (us): {executionTime * 1e6 / dancers.Length}");
            base.Update(window, camera, input, dt);
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            for (int i = 0; i < dancers.Length; ++i)
            {
                renderer.Shapes.AddInstances(dancers[i].Simulation);
            }
            base.Render(renderer, camera, input, text, font);
        }
    }
}
