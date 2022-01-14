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
        struct DancerBodyHandles
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

            internal static unsafe Buffer<BodyHandle> AsBuffer(DancerBodyHandles* sourceBodyHandles)
            {
                return new Buffer<BodyHandle>(sourceBodyHandles, 12);
            }
        }

        struct Dancer
        {
            public Simulation Simulation;
            public DancerBodyHandles BodyHandles;
        }

        DancerBodyHandles sourceBodyHandles;
        ParallelLooper looper;
        //Each dancer has its own simulation. The goal is to show a common use case for cosmetic physics- single threaded simulations that don't interact with the main simulation, running in parallel with other simulations.
        //(The simulations and handles are kept separate just because the simulations are handed over as a group to the renderer to draw stuff.)
        Simulation[] dancerSimulations;
        //The body handles are cached in each simulation so the source states can be mapped onto each dancer.
        DancerBodyHandles[] dancerHandles;

        const float legOffsetX = 0.135f;
        const float armOffsetX = 0.25f;
        const int dancerGridWidth = 16;
        const int dancerGridLength = 16;
        const int dancerCount = dancerGridWidth * dancerGridLength;
        const int historyLength = 256;


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

        static BodyHandle[,] CreateDressBodyGrid(Vector3 position, int widthInNodes, float spacing, float bodyRadius, float massPerBody,
            int instanceId, Simulation simulation, CollidableProperty<ClothCollisionFilter> filters)
        {
            var description = BodyDescription.CreateDynamic(QuaternionEx.Identity, new BodyInertia { InverseMass = 1f / massPerBody }, simulation.Shapes.Add(new Sphere(bodyRadius)), 0.01f);
            BodyHandle[,] handles = new BodyHandle[widthInNodes, widthInNodes];
            var armHoleCenter = new Vector2(armOffsetX + 0.065f, 0);
            var armHoleRadius = 0.095f;
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
                        //Too close to an arm or too far from the center, don't create any bodies here.
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
                    //Note the use of a limit; the distance is allowed to go smaller.
                    //This helps stop the cloth from having unnatural rigidity.
                    var distance = Vector3.Distance(a.Pose.Position, b.Pose.Position);
                    simulation.Solver.Add(aHandle, bHandle, new CenterDistanceLimit(distance * 0.15f, distance, springSettings));
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

        static float GetDistanceFromMainDancer(int dancerIndex)
        {
            var (columnIndex, rowIndex) = GetRowAndColumnForDancer(dancerIndex);
            var offsetX = columnIndex - (dancerGridWidth / 2 - 0.5f);
            return MathF.Sqrt(offsetX * offsetX + rowIndex * rowIndex);
        }

        static void TailorDress(Simulation simulation, CollidableProperty<ClothCollisionFilter> filters, DancerBodyHandles bodyHandles, int dancerIndex, float levelOfDetail)
        {
            //The demo uses lower resolution grids on dancers further away from the main dancer.
            //This is a sorta-example of level of detail. In a 'real' use case, you'd probably want to transition between levels of detail dynamically as the camera moved around.
            //That's a little trickier, but doable. Going low to high, for example, requires creating bodies at interpolated positions between existing bodies, while going to a lower level of detail removes them.
            var targetDressDiameter = 2.6f;
            var fullDetailWidthInBodies = 35;
            float spacingAtFullDetail = targetDressDiameter / fullDetailWidthInBodies;
            float bodyRadius = spacingAtFullDetail / 1.75f;
            var scale = MathF.Pow(2, levelOfDetail);
            var widthInBodies = (int)MathF.Ceiling(fullDetailWidthInBodies / scale);
            var spacing = spacingAtFullDetail * scale;
            var chest = simulation.Bodies[bodyHandles.Chest];
            ref var chestShape = ref simulation.Shapes.GetShape<Capsule>(chest.Collidable.Shape.Index);
            var topOfChestHeight = chest.Pose.Position.Y + chestShape.Radius + bodyRadius;
            var bodies = CreateDressBodyGrid(new Vector3(0, topOfChestHeight, 0) + GetOffsetForDancer(dancerIndex), widthInBodies, spacing, bodyRadius, 0.01f, dancerIndex, simulation, filters);
            //Create constraints that bind the cloth bodies closest to the chest, to the chest. This keeps the dress from sliding around.
            //In the higher resolution simulations, the arm holes and cloth bodies can actually handle it with no help, but for lower levels of detail it can be useful.
            //Also, it's very common to want to control how cloth sticks to a character. You could extend this approach to, for example, keep cloth near the body at the waist like a belt.
            //This demo uses constraints to attach a subset of the cloth bodies to the chest.
            //You could also either treat the bodies as kinematic and have them follow the body, or attach any constraints that would have involved the cloth body to the body instead.
            //Using constraints gives you more options in configuration- the attachment doesn't have to be perfectly rigid.
            //For the purposes of this demo, it's also simpler to just use some more constraints.
            var midpoint = (widthInBodies * 0.5f - 0.5f);
            var zRange = (chestShape.Radius * 0.65f) / spacing;
            var xRange = (chestShape.Radius * 0.5f + chestShape.HalfLength) / spacing;
            var minX = (int)MathF.Ceiling(midpoint - xRange);
            var maxX = (int)(midpoint + xRange);
            var minZ = (int)MathF.Ceiling(midpoint - zRange);
            var maxZ = (int)(midpoint + zRange);
            for (int z = minZ; z <= maxZ; ++z)
            {
                for (int x = minX; x <= maxX; ++x)
                {
                    var clothNodeHandle = bodies[z, x];
                    //When creating bodies, we set handles for bodies that don't exist to -1.
                    if (clothNodeHandle.Value >= 0)
                    {
                        var clothNodeBody = simulation.Bodies[clothNodeHandle];
                        simulation.Solver.Add(chest.Handle, clothNodeBody.Handle,
                            new BallSocket
                            {
                                LocalOffsetA = QuaternionEx.Transform(clothNodeBody.Pose.Position - chest.Pose.Position, Quaternion.Conjugate(chest.Pose.Orientation)),
                                SpringSettings = new SpringSettings(30, 1)
                            });
                    }
                }
            }
            CreateDistanceConstraints(bodies, new SpringSettings(60, 1), simulation);
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

            dancerHandles = new DancerBodyHandles[dancerCount];
            dancerSimulations = new Simulation[dancerCount];
            static BodyHandle CreateCopyForDancer(Simulation sourceSimulation, BodyHandle sourceHandle, TypedIndex shapeIndexInTargetSimulation, Simulation targetSimulation, int dancerIndex, CollidableProperty<ClothCollisionFilter> filters)
            {
                var description = sourceSimulation.Bodies.GetDescription(sourceHandle);
                description.Pose.Position += GetOffsetForDancer(dancerIndex);
                description.Collidable.Shape = shapeIndexInTargetSimulation;
                description.LocalInertia = default;
                var handle = targetSimulation.Bodies.Add(description);
                //Kinematic bodies collide with all cloth, so give it a -1 instance id.
                filters.Allocate(handle) = new ClothCollisionFilter(0, 0, -1);
                return handle;
            }

            for (int i = 0; i < dancerHandles.Length; ++i)
            {
                ref var dancer = ref dancerHandles[i];
                var dancerFilters = new CollidableProperty<ClothCollisionFilter>();
                //Distance from the main dancer is used to select clothing level of detail. This isn't dynamic based on camera motion, but shows the general idea.
                //Since we don't have to worry about transitions, the level of detail is a continuous value here.
                var distanceFromMainDancer = GetDistanceFromMainDancer(i);
                var levelOfDetail = MathF.Max(0f, MathF.Min(1.5f, MathF.Log2(MathF.Max(1, distanceFromMainDancer) - 0.8f)));
                //Note that we use a smaller allocation block size for dancer simulations.
                //This demo is creating a *lot* of buffer pools just because that's the simplest way to keep things thread safe.
                //If you wanted to reduce the amount of pool-induced memory overhead, you could consider sharing buffer pools between multiple simulations
                //and making sure those simulations never run on multiple threads at the same time to avoid allocation related race conditions.
                //Depending on the simulation, it could also be worth having multiple characters simulated in the same simulation even if you don't care about their interactions.
                //For example, if you wanted to train a motorized ragdoll using reinforcement learning, it is likely that having multiple ragdolls in each simulation would improve hardware utilization.
                //All narrow phase collision tests and constraint solves are vectorized over multiple pairs; having only one ragdoll would likely leave many bundles partially filled.
                //In this demo, occupancy is less of a concern since there are a decent number of constraints associated with a single dancer.
                var dancerSimulation = Simulation.Create(new BufferPool(16384),
                    //If the required detail goes low enough, note that this demo disables cloth self collision to save some extra time.
                    //The ClothCallbacks specify a minimum distance required for self collision, and low detail (higher 'level of detail' values) results in MaxValue minimum distance.
                    new ClothCallbacks(dancerFilters, new PairMaterialProperties(0.4f, 20, new SpringSettings(120, 1)), levelOfDetail <= 0.5f ? 3 : int.MaxValue),
                    new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(1, 4),
                    //To save some memory, initialize the dancer simulations with smaller starting sizes. For the higher level of detail simulations this could require some resizing. 
                    //More precise estimates could be made without too much work, but the demo will keep it simple.
                    initialAllocationSizes: new SimulationAllocationSizes(128, 1, 1, 8, 512, 64, 8));
                dancer.Hips = CreateCopyForDancer(Simulation, sourceBodyHandles.Hips, dancerSimulation.Shapes.Add(hipShape), dancerSimulation, i, dancerFilters);
                dancer.Abdomen = CreateCopyForDancer(Simulation, sourceBodyHandles.Abdomen, dancerSimulation.Shapes.Add(abdomenShape), dancerSimulation, i, dancerFilters);
                dancer.Chest = CreateCopyForDancer(Simulation, sourceBodyHandles.Chest, dancerSimulation.Shapes.Add(chestShape), dancerSimulation, i, dancerFilters);
                dancer.Head = CreateCopyForDancer(Simulation, sourceBodyHandles.Head, dancerSimulation.Shapes.Add(headShape), dancerSimulation, i, dancerFilters);

                var upperLegShapeInTarget = dancerSimulation.Shapes.Add(upperLegShape);
                var lowerLegShapeInTarget = dancerSimulation.Shapes.Add(lowerLegShape);
                dancer.UpperLeftLeg = CreateCopyForDancer(Simulation, sourceBodyHandles.UpperLeftLeg, upperLegShapeInTarget, dancerSimulation, i, dancerFilters);
                dancer.LowerLeftLeg = CreateCopyForDancer(Simulation, sourceBodyHandles.LowerLeftLeg, lowerLegShapeInTarget, dancerSimulation, i, dancerFilters);
                dancer.UpperRightLeg = CreateCopyForDancer(Simulation, sourceBodyHandles.UpperRightLeg, upperLegShapeInTarget, dancerSimulation, i, dancerFilters);
                dancer.LowerRightLeg = CreateCopyForDancer(Simulation, sourceBodyHandles.LowerRightLeg, lowerLegShapeInTarget, dancerSimulation, i, dancerFilters);

                var upperArmShapeInTarget = dancerSimulation.Shapes.Add(upperArmShape);
                var lowerArmShapeInTarget = dancerSimulation.Shapes.Add(lowerArmShape);
                dancer.UpperLeftArm = CreateCopyForDancer(Simulation, sourceBodyHandles.UpperLeftArm, upperArmShapeInTarget, dancerSimulation, i, dancerFilters);
                dancer.LowerLeftArm = CreateCopyForDancer(Simulation, sourceBodyHandles.LowerLeftArm, lowerArmShapeInTarget, dancerSimulation, i, dancerFilters);
                dancer.UpperRightArm = CreateCopyForDancer(Simulation, sourceBodyHandles.UpperRightArm, upperArmShapeInTarget, dancerSimulation, i, dancerFilters);
                dancer.LowerRightArm = CreateCopyForDancer(Simulation, sourceBodyHandles.LowerRightArm, lowerArmShapeInTarget, dancerSimulation, i, dancerFilters);

                TailorDress(dancerSimulation, dancerFilters, dancer, i, levelOfDetail);
                dancerBodyCount += dancerSimulation.Bodies.ActiveSet.Count;
                dancerConstraintCount += dancerSimulation.Solver.CountConstraints();
                dancerSimulations[i] = dancerSimulation;
            }
        }
        int dancerBodyCount;
        int dancerConstraintCount;

        unsafe void ExecuteDancer(int dancerIndex, int workerIndex)
        {
            //Copy historical motion states to the dancers.
            ref var dancerHandles = ref this.dancerHandles[dancerIndex];
            var dancerSimulation = dancerSimulations[dancerIndex];
            var sourceHandleBuffer = DancerBodyHandles.AsBuffer((DancerBodyHandles*)Unsafe.AsPointer(ref sourceBodyHandles));
            //Delay is greater for the dancers that are further away, plus a little randomized component to desynchronize them.
            var historicalStateStartIndex = motionHistory.Count - sourceHandleBuffer.Length * ((int)GetDistanceFromMainDancer(dancerIndex) * 8 + 1 + (HashHelper.Rehash(dancerIndex) & 0xF));
            if (historicalStateStartIndex < 0)
                historicalStateStartIndex = 0;
            var targetHandleBuffer = DancerBodyHandles.AsBuffer((DancerBodyHandles*)Unsafe.AsPointer(ref dancerHandles));
            for (int j = 0; j < sourceHandleBuffer.Length; ++j)
            {
                ref var targetMotionState = ref dancerSimulation.Bodies[targetHandleBuffer[j]].MotionState;
                targetMotionState = motionHistory[historicalStateStartIndex + j];
                targetMotionState.Pose.Position += GetOffsetForDancer(dancerIndex);
            }
            //Update the simulation for the dancer.
            dancerSimulation.Timestep(1 / 60f);
        }

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

        double time;
        double executionTime;
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
            var sourceHandleBuffer = DancerBodyHandles.AsBuffer((DancerBodyHandles*)Unsafe.AsPointer(ref sourceBodyHandles));
            if (motionHistory.Count == historyLength * sourceHandleBuffer.Length)
                for (int i = 0; i < sourceHandleBuffer.Length; ++i)
                    motionHistory.Dequeue();

            for (int i = 0; i < sourceHandleBuffer.Length; ++i)
            {
                motionHistory.EnqueueUnsafely(Simulation.Bodies[sourceHandleBuffer[i]].MotionState);
            }
            var startTime = Stopwatch.GetTimestamp();
            looper.For(0, dancerHandles.Length, ExecuteDancer);
            var endTime = Stopwatch.GetTimestamp();
            executionTime = (endTime - startTime) / (double)Stopwatch.Frequency;
            base.Update(window, camera, input, dt);
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            renderer.Shapes.AddInstances(dancerSimulations, ThreadDispatcher);
            renderer.Lines.Extract(dancerSimulations, ThreadDispatcher);

            var resolution = renderer.Surface.Resolution;
            renderer.TextBatcher.Write(text.Clear().Append("Cosmetic simulations, like cloth, often don't need to be in a game's main simulation."), new Vector2(16, resolution.Y - 144), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Every background dancer in this demo has its own simulation. All dancers can be easily updated in parallel."), new Vector2(16, resolution.Y - 128), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Dancers further from the main dancer use sparser cloth and disable self collision for extra performance."), new Vector2(16, resolution.Y - 112), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Dancer count: ").Append(dancerHandles.Length), new Vector2(16, resolution.Y - 80), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Total cloth body count: ").Append(dancerBodyCount), new Vector2(16, resolution.Y - 64), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Total cloth constraint count: ").Append(dancerConstraintCount), new Vector2(16, resolution.Y - 48), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Total dancer execution time (ms): ").Append(executionTime * 1000, 2), new Vector2(16, resolution.Y - 32), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Amortized execution time per dancer (us): ").Append(executionTime * 1e6 / dancerHandles.Length, 1), new Vector2(16, resolution.Y - 16), 16, Vector3.One, font);

            base.Render(renderer, camera, input, text, font);
        }
        protected override void OnDispose()
        {
            //While the main simulation and pool is disposed by the Demo.cs Dispose function, the dancers have their own pools and need to be cleared.
            //Note that we don't bother disposing the simulation here- all resources in the simulation were taken from the associated pool, and the simulation will not be used anymore.
            //We can just clear the buffer pool and let the GC eat the simulation.
            for (int i = 0; i < dancerSimulations.Length; ++i)
            {
                dancerSimulations[i].BufferPool.Clear();
            }
            base.OnDispose();
        }
    }
}
