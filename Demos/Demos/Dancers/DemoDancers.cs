using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using DemoRenderer;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Demos.Demos.Dancers
{
    //This file contains all the shared helpers used in the DancerDemo and PlumpDancerDemo.
    //It is responsible for creating the main dancer, its constraints, its control mechanisms, and making it dance.
    //The DancerDemo and PlumpDancerDemo simply add whatever attachments they want on top of the dancers created by this infrastructure.

    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct DancerBodyHandles
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

    /// <summary>
    /// Controls one of the main dancer's limbs by yanking it around.
    /// </summary>
    struct DancerControl
    {
        public ConstraintHandle Servo;
        public Vector3 LocalOffset;
        public ServoSettings ServoSettings;
        public SpringSettings SpringSettings;

        public DancerControl(Simulation simulation, BodyHandle body, Vector3 worldControlPoint, ServoSettings servoSettings, SpringSettings springSettings)
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

    /// <summary>
    /// Marks an <see cref="INarrowPhaseCallbacks"/> as usable with the DemoDancers.
    /// </summary>
    /// <typeparam name="TCallbacks">Type of the callbacks to use.</typeparam>
    /// <typeparam name="TFilter">Type of the callback filters to use.</typeparam>
    public interface IDancerNarrowPhaseCallbacks<TCallbacks, TFilter> where TCallbacks : INarrowPhaseCallbacks, IDancerNarrowPhaseCallbacks<TCallbacks, TFilter> where TFilter : unmanaged
    {
        TCallbacks Create(CollidableProperty<TFilter> filters, PairMaterialProperties pairMaterialProperties, int minimumDistanceForSelfCollisions);
    }

    /// <summary>
    /// Used by <see cref="DancerDemo"/> and <see cref="PlumpDancerDemo"/> to coordinate lots of background dancers.
    /// </summary>
    public class DemoDancers
    {
        ParallelLooper looper;
        public DancerBodyHandles SourceBodyHandles;
        //Each dancer has its own simulation. The goal is to show a common use case for cosmetic physics- single threaded simulations that don't interact with the main simulation, running in parallel with other simulations.
        //(The simulations and handles are kept separate just because the simulations are handed over as a group to the renderer to draw stuff.)
        public Simulation[] Simulations;
        //The body handles are cached in each simulation so the source states can be mapped onto each dancer.
        public DancerBodyHandles[] Handles;

        public const float LegOffsetX = 0.135f;
        public const float ArmOffsetX = 0.25f;
        public const int DancerGridWidth = 16;
        public const int DancerGridLength = 16;
        public const int DancerCount = DancerGridWidth * DancerGridLength;
        public const int HistoryLength = 256;


        DancerControl hipsControl;
        DancerControl leftFootControl;
        DancerControl rightFootControl;
        DancerControl leftHandControl;
        DancerControl rightHandControl;

        public delegate void DressUpDancer<TCollisionFilter>(Simulation simulation, CollidableProperty<TCollisionFilter> filters, DancerBodyHandles bodyHandles, int dancerIndex, float levelOfDetail) where TCollisionFilter : unmanaged;
        public DemoDancers Initialize<TNarrowPhaseCallbacks, TCollisionFilter>(Simulation mainSimulation, CollidableProperty<SubgroupCollisionFilter> mainCollisionFilters, IThreadDispatcher threadDispatcher, BufferPool pool,
            DressUpDancer<TCollisionFilter> dressUpDancer, TCollisionFilter filterForDancerBodies)
            where TNarrowPhaseCallbacks : struct, INarrowPhaseCallbacks, IDancerNarrowPhaseCallbacks<TNarrowPhaseCallbacks, TCollisionFilter>
            where TCollisionFilter : unmanaged
        {
            looper = new ParallelLooper() { Dispatcher = threadDispatcher };

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
            var armOffset = new Vector3(ArmOffsetX, 0, 0);
            var legOffset = new Vector3(LegOffsetX, 0, 0);
            const int groupIndex = 0;

            //Build the torso and head bodies.
            RagdollDemo.GetCapsuleForLineSegment(hipsPosition - legOffset, hipsPosition + legOffset, 0.14f, out var hipShape, out _, out var hipOrientation);
            SourceBodyHandles.Hips = mainSimulation.Bodies.Add(BodyDescription.CreateDynamic((hipsPosition, hipOrientation), hipShape.ComputeInertia(1), mainSimulation.Shapes.Add(hipShape), 0.01f));
            ref var hipsFilter = ref mainCollisionFilters.Allocate(SourceBodyHandles.Hips);
            hipsFilter = new SubgroupCollisionFilter(groupIndex, 0);

            RagdollDemo.GetCapsuleForLineSegment(abdomenPosition - legOffset * 0.8f, abdomenPosition + legOffset * 0.8f, 0.13f, out var abdomenShape, out _, out var abdomenOrientation);
            SourceBodyHandles.Abdomen = mainSimulation.Bodies.Add(BodyDescription.CreateDynamic((abdomenPosition, abdomenOrientation), abdomenShape.ComputeInertia(1), mainSimulation.Shapes.Add(abdomenShape), 0.01f));
            ref var abdomenFilter = ref mainCollisionFilters.Allocate(SourceBodyHandles.Abdomen);
            abdomenFilter = new SubgroupCollisionFilter(groupIndex, 1);

            RagdollDemo.GetCapsuleForLineSegment(abdomenPosition - legOffset * 0.8f, abdomenPosition + legOffset * 0.8f, 0.165f, out var chestShape, out _, out var chestOrientation);
            SourceBodyHandles.Chest = mainSimulation.Bodies.Add(BodyDescription.CreateDynamic((chestPosition, chestOrientation), chestShape.ComputeInertia(1), mainSimulation.Shapes.Add(chestShape), 0.01f));
            ref var chestFilter = ref mainCollisionFilters.Allocate(SourceBodyHandles.Chest);
            chestFilter = new SubgroupCollisionFilter(groupIndex, 2);

            var headShape = new Sphere(0.17f);
            SourceBodyHandles.Head = mainSimulation.Bodies.Add(BodyDescription.CreateDynamic(headPosition, headShape.ComputeInertia(1), mainSimulation.Shapes.Add(headShape), 1e-2f));
            ref var headFilter = ref mainCollisionFilters.Allocate(SourceBodyHandles.Head);
            headFilter = new SubgroupCollisionFilter(groupIndex, 3);

            var springSettings = new SpringSettings(30, 1);
            Connect(mainSimulation, SourceBodyHandles.Hips, SourceBodyHandles.Abdomen, 0.5f * (hipsPosition + abdomenPosition), springSettings, ref hipsFilter, ref abdomenFilter);
            ConstrainOrientation(mainSimulation, SourceBodyHandles.Hips, SourceBodyHandles.Abdomen);
            Connect(mainSimulation, SourceBodyHandles.Abdomen, SourceBodyHandles.Chest, 0.5f * (abdomenPosition + chestPosition), springSettings, ref abdomenFilter, ref chestFilter);
            ConstrainOrientation(mainSimulation, SourceBodyHandles.Abdomen, SourceBodyHandles.Chest);
            Connect(mainSimulation, SourceBodyHandles.Chest, SourceBodyHandles.Head, 0.5f * (chestPosition + headPosition), springSettings, ref chestFilter, ref headFilter);
            ConstrainOrientation(mainSimulation, SourceBodyHandles.Chest, SourceBodyHandles.Head);

            //Create the legs.
            RagdollDemo.GetCapsuleForLineSegment(hipsPosition, kneePosition, 0.11f, out var upperLegShape, out var upperLegPosition, out var upperLegOrientation);
            RagdollDemo.GetCapsuleForLineSegment(kneePosition, anklePosition, 0.1f, out var lowerLegShape, out var lowerLegPosition, out var lowerLegOrientation);
            var upperLegDescription = BodyDescription.CreateDynamic((upperLegPosition, upperLegOrientation), upperLegShape.ComputeInertia(1), mainSimulation.Shapes.Add(upperLegShape), 0.01f);
            var lowerLegDescription = BodyDescription.CreateDynamic((lowerLegPosition, lowerLegOrientation), lowerLegShape.ComputeInertia(1), mainSimulation.Shapes.Add(lowerLegShape), 0.01f);

            upperLegDescription.Pose.Position -= legOffset;
            lowerLegDescription.Pose.Position -= legOffset;
            SourceBodyHandles.UpperLeftLeg = mainSimulation.Bodies.Add(upperLegDescription);
            SourceBodyHandles.LowerLeftLeg = mainSimulation.Bodies.Add(lowerLegDescription);
            upperLegDescription.Pose.Position += 2 * legOffset;
            lowerLegDescription.Pose.Position += 2 * legOffset;
            SourceBodyHandles.UpperRightLeg = mainSimulation.Bodies.Add(upperLegDescription);
            SourceBodyHandles.LowerRightLeg = mainSimulation.Bodies.Add(lowerLegDescription);

            CreateLimb(mainSimulation, mainCollisionFilters, SourceBodyHandles.Hips, SourceBodyHandles.UpperLeftLeg, SourceBodyHandles.LowerLeftLeg, hipsPosition - legOffset, kneePosition - legOffset, springSettings, groupIndex, 4);
            CreateLimb(mainSimulation, mainCollisionFilters, SourceBodyHandles.Hips, SourceBodyHandles.UpperRightLeg, SourceBodyHandles.LowerRightLeg, hipsPosition + legOffset, kneePosition + legOffset, springSettings, groupIndex, 6);

            //Create the arms.
            RagdollDemo.GetCapsuleForLineSegment(chestPosition, elbowPosition, 0.08f, out var upperArmShape, out var upperArmPosition, out var upperArmOrientation);
            RagdollDemo.GetCapsuleForLineSegment(elbowPosition, wristPosition, 0.075f, out var lowerArmShape, out var lowerArmPosition, out var lowerArmOrientation);
            var upperArmDescription = BodyDescription.CreateDynamic((upperArmPosition, upperArmOrientation), upperArmShape.ComputeInertia(1), mainSimulation.Shapes.Add(upperArmShape), 0.01f);
            var lowerArmDescription = BodyDescription.CreateDynamic((lowerArmPosition, lowerArmOrientation), lowerArmShape.ComputeInertia(1), mainSimulation.Shapes.Add(lowerArmShape), 0.01f);

            upperArmDescription.Pose.Position -= armOffset;
            lowerArmDescription.Pose.Position -= armOffset;
            SourceBodyHandles.UpperLeftArm = mainSimulation.Bodies.Add(upperArmDescription);
            SourceBodyHandles.LowerLeftArm = mainSimulation.Bodies.Add(lowerArmDescription);
            upperArmDescription.Pose.Position += 2 * armOffset;
            lowerArmDescription.Pose.Position += 2 * armOffset;
            SourceBodyHandles.UpperRightArm = mainSimulation.Bodies.Add(upperArmDescription);
            SourceBodyHandles.LowerRightArm = mainSimulation.Bodies.Add(lowerArmDescription);

            CreateLimb(mainSimulation, mainCollisionFilters, SourceBodyHandles.Chest, SourceBodyHandles.UpperLeftArm, SourceBodyHandles.LowerLeftArm, chestPosition - armOffset, elbowPosition - armOffset, springSettings, groupIndex, 8);
            CreateLimb(mainSimulation, mainCollisionFilters, SourceBodyHandles.Chest, SourceBodyHandles.UpperRightArm, SourceBodyHandles.LowerRightArm, chestPosition + armOffset, elbowPosition + armOffset, springSettings, groupIndex, 10);

            //Create controls.
            hipsControl = new DancerControl(mainSimulation, SourceBodyHandles.Hips, hipsPosition, ServoSettings.Default, new SpringSettings(5, 1));
            mainSimulation.Solver.Add(SourceBodyHandles.Hips, new OneBodyAngularServo { ServoSettings = ServoSettings.Default, SpringSettings = new SpringSettings(30, 1), TargetOrientation = mainSimulation.Bodies[SourceBodyHandles.Hips].Pose.Orientation });

            var limbServoSettings = ServoSettings.Default;
            var limbSpringSettings = new SpringSettings(4, 1);
            leftFootControl = new DancerControl(mainSimulation, SourceBodyHandles.LowerLeftLeg, anklePosition - legOffset, limbServoSettings, limbSpringSettings);
            rightFootControl = new DancerControl(mainSimulation, SourceBodyHandles.LowerRightLeg, anklePosition + legOffset, limbServoSettings, limbSpringSettings);
            leftHandControl = new DancerControl(mainSimulation, SourceBodyHandles.LowerLeftArm, wristPosition - armOffset, limbServoSettings, limbSpringSettings);
            rightHandControl = new DancerControl(mainSimulation, SourceBodyHandles.LowerRightArm, wristPosition + armOffset, limbServoSettings, limbSpringSettings);


            mainSimulation.Statics.Add(new(new Vector3(0, -1.24f, 0), mainSimulation.Shapes.Add(new Box(1000, 1, 1000))));

            //All the background dancers read different historical motion states so that they're not just all doing the exact same thing.
            //Keep the states in a queue. Each batch of 12 motion states is the state for a single frame.
            MotionHistory = new QuickQueue<MotionState>(HistoryLength * 12, pool);

            Handles = new DancerBodyHandles[DancerCount];
            Simulations = new Simulation[DancerCount];
            static BodyHandle CreateCopyForDancer(Simulation sourceSimulation, BodyHandle sourceHandle, TypedIndex shapeIndexInTargetSimulation, Simulation targetSimulation, int dancerIndex, CollidableProperty<TCollisionFilter> filters, TCollisionFilter bodyFilter)
            {
                var description = sourceSimulation.Bodies.GetDescription(sourceHandle);
                description.Pose.Position += GetOffsetForDancer(dancerIndex);
                description.Collidable.Shape = shapeIndexInTargetSimulation;
                description.LocalInertia = default;
                var handle = targetSimulation.Bodies.Add(description);
                filters.Allocate(handle) = bodyFilter;
                return handle;
            }

            for (int i = 0; i < Handles.Length; ++i)
            {
                ref var dancer = ref Handles[i];
                var dancerFilters = new CollidableProperty<TCollisionFilter>();
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

                //If the required detail goes low enough, note that this demo disables cloth self collision to save some extra time.
                //The ClothCallbacks specify a minimum distance required for self collision, and low detail (higher 'level of detail' values) results in MaxValue minimum distance.
                var narrowPhaseCallbacks = default(TNarrowPhaseCallbacks).Create(dancerFilters, new PairMaterialProperties(0.4f, 20, new SpringSettings(120, 1)), levelOfDetail <= 0.5f ? 3 : int.MaxValue);
                var dancerSimulation = Simulation.Create(new BufferPool(16384), narrowPhaseCallbacks,
                    new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(1, 4),
                    //To save some memory, initialize the dancer simulations with smaller starting sizes. For the higher level of detail simulations this could require some resizing. 
                    //More precise estimates could be made without too much work, but the demo will keep it simple.
                    initialAllocationSizes: new SimulationAllocationSizes(128, 1, 1, 8, 512, 64, 8));
                dancer.Hips = CreateCopyForDancer(mainSimulation, SourceBodyHandles.Hips, dancerSimulation.Shapes.Add(hipShape), dancerSimulation, i, dancerFilters, filterForDancerBodies);
                dancer.Abdomen = CreateCopyForDancer(mainSimulation, SourceBodyHandles.Abdomen, dancerSimulation.Shapes.Add(abdomenShape), dancerSimulation, i, dancerFilters, filterForDancerBodies);
                dancer.Chest = CreateCopyForDancer(mainSimulation, SourceBodyHandles.Chest, dancerSimulation.Shapes.Add(chestShape), dancerSimulation, i, dancerFilters, filterForDancerBodies);
                dancer.Head = CreateCopyForDancer(mainSimulation, SourceBodyHandles.Head, dancerSimulation.Shapes.Add(headShape), dancerSimulation, i, dancerFilters, filterForDancerBodies);

                var upperLegShapeInTarget = dancerSimulation.Shapes.Add(upperLegShape);
                var lowerLegShapeInTarget = dancerSimulation.Shapes.Add(lowerLegShape);
                dancer.UpperLeftLeg = CreateCopyForDancer(mainSimulation, SourceBodyHandles.UpperLeftLeg, upperLegShapeInTarget, dancerSimulation, i, dancerFilters, filterForDancerBodies);
                dancer.LowerLeftLeg = CreateCopyForDancer(mainSimulation, SourceBodyHandles.LowerLeftLeg, lowerLegShapeInTarget, dancerSimulation, i, dancerFilters, filterForDancerBodies);
                dancer.UpperRightLeg = CreateCopyForDancer(mainSimulation, SourceBodyHandles.UpperRightLeg, upperLegShapeInTarget, dancerSimulation, i, dancerFilters, filterForDancerBodies);
                dancer.LowerRightLeg = CreateCopyForDancer(mainSimulation, SourceBodyHandles.LowerRightLeg, lowerLegShapeInTarget, dancerSimulation, i, dancerFilters, filterForDancerBodies);

                var upperArmShapeInTarget = dancerSimulation.Shapes.Add(upperArmShape);
                var lowerArmShapeInTarget = dancerSimulation.Shapes.Add(lowerArmShape);
                dancer.UpperLeftArm = CreateCopyForDancer(mainSimulation, SourceBodyHandles.UpperLeftArm, upperArmShapeInTarget, dancerSimulation, i, dancerFilters, filterForDancerBodies);
                dancer.LowerLeftArm = CreateCopyForDancer(mainSimulation, SourceBodyHandles.LowerLeftArm, lowerArmShapeInTarget, dancerSimulation, i, dancerFilters, filterForDancerBodies);
                dancer.UpperRightArm = CreateCopyForDancer(mainSimulation, SourceBodyHandles.UpperRightArm, upperArmShapeInTarget, dancerSimulation, i, dancerFilters, filterForDancerBodies);
                dancer.LowerRightArm = CreateCopyForDancer(mainSimulation, SourceBodyHandles.LowerRightArm, lowerArmShapeInTarget, dancerSimulation, i, dancerFilters, filterForDancerBodies);

                dressUpDancer(dancerSimulation, dancerFilters, dancer, i, levelOfDetail);
                BodyCount += dancerSimulation.Bodies.ActiveSet.Count;
                ConstraintCount += dancerSimulation.Solver.CountConstraints();
                Simulations[i] = dancerSimulation;
            }
            return this;
        }
        public int BodyCount;
        public int ConstraintCount;


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
            var rowIndex = dancerIndex / DancerGridWidth;
            return (dancerIndex - rowIndex * DancerGridWidth, rowIndex);
        }

        public QuickQueue<MotionState> MotionHistory;
        public static Vector3 GetOffsetForDancer(int i)
        {
            const float spacing = 2;
            var (columnIndex, rowIndex) = GetRowAndColumnForDancer(i);
            return new Vector3(DancerGridWidth * spacing / -2 + (columnIndex + 0.5f) * spacing, 0, -2 + rowIndex * -spacing);
        }

        public static float GetDistanceFromMainDancer(int dancerIndex)
        {
            var (columnIndex, rowIndex) = GetRowAndColumnForDancer(dancerIndex);
            var offsetX = columnIndex - (DancerGridWidth / 2 - 0.5f);
            return MathF.Sqrt(offsetX * offsetX + rowIndex * rowIndex);
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
            return new Vector3(-xOffset - LegOffsetX, yOffset, zOffset);
        }
        Vector3 CreateArmTarget(float t)
        {
            var z = MathF.Cos(t * MathF.Tau);
            var zOffset = (Smoothstep(z * 0.5f + 0.5f) * 2 - 1);
            var offset = 0.5f + 0.5f * MathF.Cos(MathF.PI + t * 4 * MathF.PI);
            var xOffset = -0.2f + 0.6f * offset;
            var yOffset = 0.9f - 0.2f * offset;
            return new Vector3(-xOffset - ArmOffsetX, yOffset, zOffset);
        }
        unsafe void ExecuteDancer(int dancerIndex, int workerIndex)
        {
            //Copy historical motion states to the dancers.
            ref var dancerHandles = ref Handles[dancerIndex];
            var dancerSimulation = Simulations[dancerIndex];
            var sourceHandleBuffer = DancerBodyHandles.AsBuffer((DancerBodyHandles*)Unsafe.AsPointer(ref SourceBodyHandles));
            //Delay is greater for the dancers that are further away, plus a little randomized component to desynchronize them.
            var historicalStateStartIndex = MotionHistory.Count - sourceHandleBuffer.Length * ((int)GetDistanceFromMainDancer(dancerIndex) * 8 + 1 + (HashHelper.Rehash(dancerIndex) & 0xF));
            if (historicalStateStartIndex < 0)
                historicalStateStartIndex = 0;
            var targetHandleBuffer = DancerBodyHandles.AsBuffer((DancerBodyHandles*)Unsafe.AsPointer(ref dancerHandles));
            for (int j = 0; j < sourceHandleBuffer.Length; ++j)
            {
                ref var targetMotionState = ref dancerSimulation.Bodies[targetHandleBuffer[j]].MotionState;
                targetMotionState = MotionHistory[historicalStateStartIndex + j];
                targetMotionState.Pose.Position += GetOffsetForDancer(dancerIndex);
            }
            //Update the simulation for the dancer.
            dancerSimulation.Timestep(Demo.TimestepDuration);
        }

        double time;
        public double ExecutionTime;
        public unsafe void UpdateTargets(Simulation mainSimulation)
        {
            //Using a fixed interval here, matching the time used in the Demo.
            time += Demo.TimestepDuration;
            var hipsTarget = new Vector3(0, 0, 3 * (float)Math.Sin(time / 4));
            hipsControl.UpdateTarget(mainSimulation, hipsTarget);
            const float stepDuration = 3.5f;
            var scaledTime = time / stepDuration;
            var t = (float)(scaledTime - Math.Floor(scaledTime));
            var tOffset = (t + 0.5f) % 1;
            var leftFootLocalTarget = CreateLegTarget(t);
            var rightFootLocalTarget = CreateLegTarget(tOffset);
            rightFootLocalTarget.X *= -1;
            leftFootControl.UpdateTarget(mainSimulation, hipsTarget + leftFootLocalTarget);
            rightFootControl.UpdateTarget(mainSimulation, hipsTarget + rightFootLocalTarget);


            var leftArmLocalTarget = CreateArmTarget(tOffset);
            var rightArmLocalTarget = CreateArmTarget(t);
            rightArmLocalTarget.X *= -1;
            leftHandControl.UpdateTarget(mainSimulation, hipsTarget + leftArmLocalTarget);
            rightHandControl.UpdateTarget(mainSimulation, hipsTarget + rightArmLocalTarget);

            //Record the latest motion state from the source dancer.
            var sourceHandleBuffer = DancerBodyHandles.AsBuffer((DancerBodyHandles*)Unsafe.AsPointer(ref SourceBodyHandles));
            if (MotionHistory.Count == HistoryLength * sourceHandleBuffer.Length)
                for (int i = 0; i < sourceHandleBuffer.Length; ++i)
                    MotionHistory.Dequeue();

            for (int i = 0; i < sourceHandleBuffer.Length; ++i)
            {
                MotionHistory.EnqueueUnsafely(mainSimulation.Bodies[sourceHandleBuffer[i]].MotionState);
            }
            var startTime = Stopwatch.GetTimestamp();
            looper.For(0, Handles.Length, ExecuteDancer);
            var endTime = Stopwatch.GetTimestamp();
            ExecutionTime = (endTime - startTime) / (double)Stopwatch.Frequency;
        }

        public void Dispose(BufferPool pool)
        {
            //While the main simulation and pool is disposed by the Demo.cs Dispose function, the dancers have their own pools and need to be cleared.
            //Note that we don't bother disposing the simulation here- all resources in the simulation were taken from the associated pool, and the simulation will not be used anymore.
            //We can just clear the buffer pool and let the GC eat the simulation.
            for (int i = 0; i < Simulations.Length; ++i)
            {
                Simulations[i].BufferPool.Clear();
            }
            MotionHistory.Dispose(pool);
        }
    }
}
