using BenchmarkDotNet.Attributes;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace DemoBenchmarks;

/// <summary>
/// Evaluates performance of a simulation similar to the first timesteps of the RagdollTubeDemo.
/// </summary>
public class RagdollTubeBenchmark
{
    public struct SubgroupCollisionFilter
    {
        public ushort SubgroupMembership;
        public ushort CollidableSubgroups;
        public int GroupId;
        public SubgroupCollisionFilter(int groupId)
        {
            GroupId = groupId;
            SubgroupMembership = ushort.MaxValue;
            CollidableSubgroups = ushort.MaxValue;
        }
        public SubgroupCollisionFilter(int groupId, int subgroupId)
        {
            GroupId = groupId;
            Debug.Assert(subgroupId >= 0 && subgroupId < 16, "The subgroup field is a ushort; it can only hold 16 distinct subgroups.");
            SubgroupMembership = (ushort)(1 << subgroupId);
            CollidableSubgroups = ushort.MaxValue;
        }
        public void DisableCollision(int subgroupId)
        {
            Debug.Assert(subgroupId >= 0 && subgroupId < 16, "The subgroup field is a ushort; it can only hold 16 distinct subgroups.");
            CollidableSubgroups ^= (ushort)(1 << subgroupId);
        }
        public static void DisableCollision(ref SubgroupCollisionFilter filterA, ref SubgroupCollisionFilter filterB)
        {
            filterA.CollidableSubgroups &= (ushort)~filterB.SubgroupMembership;
            filterB.CollidableSubgroups &= (ushort)~filterA.SubgroupMembership;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool AllowCollision(in SubgroupCollisionFilter a, in SubgroupCollisionFilter b)
        {
            return a.GroupId != b.GroupId || (a.CollidableSubgroups & b.SubgroupMembership) > 0;
        }

    }

    struct SubgroupFilteredCallbacks : INarrowPhaseCallbacks
    {
        public CollidableProperty<SubgroupCollisionFilter> CollisionFilters;
        public PairMaterialProperties Material;
        public SubgroupFilteredCallbacks(CollidableProperty<SubgroupCollisionFilter> filters)
        {
            CollisionFilters = filters;
            Material = new PairMaterialProperties(1, 2, new SpringSettings(30, 1));
        }
        public SubgroupFilteredCallbacks(CollidableProperty<SubgroupCollisionFilter> filters, PairMaterialProperties material)
        {
            CollisionFilters = filters;
            Material = material;
        }
        public void Initialize(Simulation simulation)
        {
            CollisionFilters.Initialize(simulation);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
        {
            if (b.Mobility != CollidableMobility.Static)
            {
                return SubgroupCollisionFilter.AllowCollision(CollisionFilters[a.BodyHandle], CollisionFilters[b.BodyHandle]);
            }
            return a.Mobility == CollidableMobility.Dynamic || b.Mobility == CollidableMobility.Dynamic;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
        {
            return true;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
        {
            pairMaterial = Material;
            return true;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
        {
            return true;
        }
        public void Dispose()
        {
            CollisionFilters.Dispose();
        }
    }

    public struct DemoPoseIntegratorCallbacks : IPoseIntegratorCallbacks
    {
        public Vector3 Gravity;
        public float LinearDamping;
        public float AngularDamping;
        public readonly AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;
        public readonly bool AllowSubstepsForUnconstrainedBodies => false;
        public readonly bool IntegrateVelocityForKinematics => false;
        public void Initialize(Simulation simulation) { }
        public DemoPoseIntegratorCallbacks(Vector3 gravity, float linearDamping = .03f, float angularDamping = .03f) : this()
        {
            Gravity = gravity;
            LinearDamping = linearDamping;
            AngularDamping = angularDamping;
        }

        Vector3Wide gravityWideDt;
        Vector<float> linearDampingDt;
        Vector<float> angularDampingDt;

        public void PrepareForIntegration(float dt)
        {
            linearDampingDt = new Vector<float>(MathF.Pow(MathHelper.Clamp(1 - LinearDamping, 0, 1), dt));
            angularDampingDt = new Vector<float>(MathF.Pow(MathHelper.Clamp(1 - AngularDamping, 0, 1), dt));
            gravityWideDt = Vector3Wide.Broadcast(Gravity * dt);
        }
        public void IntegrateVelocity(Vector<int> bodyIndices, Vector3Wide position, QuaternionWide orientation, BodyInertiaWide localInertia, Vector<int> integrationMask, int workerIndex, Vector<float> dt, ref BodyVelocityWide velocity)
        {
            velocity.Linear = (velocity.Linear + gravityWideDt) * linearDampingDt;
            velocity.Angular = velocity.Angular * angularDampingDt;
        }
    }
    static BodyHandle AddBody<TShape>(TShape shape, float mass, in RigidPose pose, Simulation simulation) where TShape : unmanaged, IConvexShape
    {
        //Note that this always registers a new shape instance. You could be more clever/efficient and share shapes, but the goal here is to show the most basic option.
        //Also, the cost of registering different shapes isn't that high for tiny implicit shapes.
        var shapeIndex = simulation.Shapes.Add(shape);
        var description = BodyDescription.CreateDynamic(pose, shape.ComputeInertia(mass), shapeIndex, 0.01f);
        return simulation.Bodies.Add(description);
    }

    static RigidPose GetWorldPose(Vector3 localPosition, Quaternion localOrientation, RigidPose ragdollPose)
    {
        RigidPose worldPose;
        RigidPose.Transform(localPosition, ragdollPose, out worldPose.Position);
        QuaternionEx.ConcatenateWithoutOverlap(localOrientation, ragdollPose.Orientation, out worldPose.Orientation);
        return worldPose;
    }
    public static void GetCapsuleForLineSegment(Vector3 start, Vector3 end, float radius, out Capsule capsule, out Vector3 position, out Quaternion orientation)
    {
        position = 0.5f * (start + end);

        var offset = end - start;
        capsule.HalfLength = 0.5f * offset.Length();
        capsule.Radius = radius;
        //The capsule shape's length is along its local Y axis, so get the shortest rotation from Y to the current orientation.
        var cross = Vector3.Cross(offset / capsule.Length, new Vector3(0, 1, 0));
        var crossLength = cross.Length();
        orientation = crossLength > 1e-8f ? QuaternionEx.CreateFromAxisAngle(cross / crossLength, (float)Math.Asin(crossLength)) : Quaternion.Identity;
    }

    public static Quaternion CreateBasis(Vector3 z, Vector3 x)
    {
        //For ease of use, don't assume that x is perpendicular to z, nor that either input is normalized.
        Matrix3x3 basis;
        basis.Z = Vector3.Normalize(z);
        basis.Y = Vector3.Normalize(Vector3.Cross(basis.Z, x));
        basis.X = Vector3.Cross(basis.Y, basis.Z);
        QuaternionEx.CreateFromRotationMatrix(basis, out var toReturn);
        return toReturn;
    }

    static AngularMotor BuildAngularMotor()
    {
        //By default, these motors use nonzero softness (inverse damping) to damp the relative motion between ragdoll pieces.
        //If you set the damping to 0 and then set the maximum force to some finite value (75 works reasonably well), the ragdolls act more like action figures.
        //You could also replace the AngularMotors with AngularServos and provide actual relative orientation goals for physics-driven animation.
        return new AngularMotor { TargetVelocityLocalA = new Vector3(), Settings = new MotorSettings(float.MaxValue, 0.01f) };
    }

    static RagdollArmHandles AddArm(float sign, Vector3 localShoulder, RigidPose localChestPose, BodyHandle chestHandle, ref SubgroupCollisionFilter chestMask,
        int limbBaseBitIndex, int ragdollIndex, RigidPose ragdollPose, CollidableProperty<SubgroupCollisionFilter> filters, SpringSettings constraintSpringSettings, Simulation simulation)
    {
        RagdollArmHandles handles;
        var localElbow = localShoulder + new Vector3(sign * 0.45f, 0, 0);
        var localWrist = localElbow + new Vector3(sign * 0.45f, 0, 0);
        var handPosition = localWrist + new Vector3(sign * 0.1f, 0, 0);
        GetCapsuleForLineSegment(localShoulder, localElbow, 0.1f, out var upperArmShape, out var upperArmPosition, out var upperArmOrientation);
        handles.UpperArm = AddBody(upperArmShape, 5, GetWorldPose(upperArmPosition, upperArmOrientation, ragdollPose), simulation);
        GetCapsuleForLineSegment(localElbow, localWrist, 0.09f, out var lowerArmShape, out var lowerArmPosition, out var lowerArmOrientation);
        handles.LowerArm = AddBody(lowerArmShape, 5, GetWorldPose(lowerArmPosition, lowerArmOrientation, ragdollPose), simulation);
        handles.Hand = AddBody(new Box(0.2f, 0.1f, 0.2f), 2, GetWorldPose(handPosition, Quaternion.Identity, ragdollPose), simulation);

        //Create joints between limb pieces.
        //Chest-Upper Arm
        simulation.Solver.Add(chestHandle, handles.UpperArm, new BallSocket
        {
            LocalOffsetA = QuaternionEx.Transform(localShoulder - localChestPose.Position, QuaternionEx.Conjugate(localChestPose.Orientation)),
            LocalOffsetB = QuaternionEx.Transform(localShoulder - upperArmPosition, QuaternionEx.Conjugate(upperArmOrientation)),
            SpringSettings = constraintSpringSettings
        });
        simulation.Solver.Add(chestHandle, handles.UpperArm, new SwingLimit
        {
            AxisLocalA = QuaternionEx.Transform(Vector3.Normalize(new Vector3(sign, 0, 1)), QuaternionEx.Conjugate(localChestPose.Orientation)),
            AxisLocalB = QuaternionEx.Transform(new Vector3(sign, 0, 0), QuaternionEx.Conjugate(upperArmOrientation)),
            MaximumSwingAngle = MathHelper.Pi * 0.56f,
            SpringSettings = constraintSpringSettings
        });
        simulation.Solver.Add(chestHandle, handles.UpperArm, new TwistLimit
        {
            LocalBasisA = QuaternionEx.Concatenate(CreateBasis(new Vector3(1, 0, 0), new Vector3(0, 0, -1)), QuaternionEx.Conjugate(localChestPose.Orientation)),
            LocalBasisB = QuaternionEx.Concatenate(CreateBasis(new Vector3(1, 0, 0), new Vector3(0, 0, -1)), QuaternionEx.Conjugate(upperArmOrientation)),
            MinimumAngle = MathHelper.Pi * -0.55f,
            MaximumAngle = MathHelper.Pi * 0.55f,
            SpringSettings = constraintSpringSettings
        });
        simulation.Solver.Add(chestHandle, handles.UpperArm, BuildAngularMotor());

        //Upper Arm-Lower Arm
        simulation.Solver.Add(handles.UpperArm, handles.LowerArm, new SwivelHinge
        {
            LocalOffsetA = QuaternionEx.Transform(localElbow - upperArmPosition, QuaternionEx.Conjugate(upperArmOrientation)),
            LocalSwivelAxisA = new Vector3(1, 0, 0),
            LocalOffsetB = QuaternionEx.Transform(localElbow - lowerArmPosition, QuaternionEx.Conjugate(lowerArmOrientation)),
            LocalHingeAxisB = new Vector3(0, 1, 0),
            SpringSettings = constraintSpringSettings
        });
        simulation.Solver.Add(handles.UpperArm, handles.LowerArm, new SwingLimit
        {
            AxisLocalA = new Vector3(0, 1, 0),
            AxisLocalB = new Vector3(sign, 0, 0),
            MaximumSwingAngle = MathHelper.PiOver2,
            SpringSettings = constraintSpringSettings
        });
        simulation.Solver.Add(handles.UpperArm, handles.LowerArm, new TwistLimit
        {
            LocalBasisA = QuaternionEx.Concatenate(CreateBasis(new Vector3(1, 0, 0), new Vector3(0, 0, -1)), QuaternionEx.Conjugate(upperArmOrientation)),
            LocalBasisB = QuaternionEx.Concatenate(CreateBasis(new Vector3(1, 0, 0), new Vector3(0, 0, -1)), QuaternionEx.Conjugate(lowerArmOrientation)),
            MinimumAngle = MathHelper.Pi * -0.55f,
            MaximumAngle = MathHelper.Pi * 0.55f,
            SpringSettings = constraintSpringSettings
        });
        simulation.Solver.Add(handles.UpperArm, handles.LowerArm, BuildAngularMotor());

        //Lower Arm-Hand
        simulation.Solver.Add(handles.LowerArm, handles.Hand, new BallSocket
        {
            LocalOffsetA = QuaternionEx.Transform(localWrist - lowerArmPosition, QuaternionEx.Conjugate(lowerArmOrientation)),
            LocalOffsetB = localWrist - handPosition,
            SpringSettings = constraintSpringSettings
        });
        simulation.Solver.Add(handles.LowerArm, handles.Hand, new SwingLimit
        {
            AxisLocalA = QuaternionEx.Transform(new Vector3(sign, 0, 0), QuaternionEx.Conjugate(lowerArmOrientation)),
            AxisLocalB = new Vector3(sign, 0, 0),
            MaximumSwingAngle = MathHelper.PiOver2,
            SpringSettings = constraintSpringSettings
        });
        simulation.Solver.Add(handles.LowerArm, handles.Hand, new TwistServo
        {
            LocalBasisA = QuaternionEx.Concatenate(CreateBasis(new Vector3(1, 0, 0), new Vector3(0, 0, 1)), QuaternionEx.Conjugate(lowerArmOrientation)),
            LocalBasisB = CreateBasis(new Vector3(1, 0, 0), new Vector3(0, 0, 1)),
            TargetAngle = 0,
            SpringSettings = constraintSpringSettings,
            ServoSettings = new ServoSettings(float.MaxValue, 0, float.MaxValue)
        });
        simulation.Solver.Add(handles.LowerArm, handles.Hand, BuildAngularMotor());

        //Disable collisions between connected ragdoll pieces.
        var upperArmLocalIndex = limbBaseBitIndex;
        var lowerArmLocalIndex = limbBaseBitIndex + 1;
        var handLocalIndex = limbBaseBitIndex + 2;
        ref var upperArmFilter = ref filters.Allocate(handles.UpperArm);
        ref var lowerArmFilter = ref filters.Allocate(handles.LowerArm);
        ref var handFilter = ref filters.Allocate(handles.Hand);
        upperArmFilter = new SubgroupCollisionFilter(ragdollIndex, upperArmLocalIndex);
        lowerArmFilter = new SubgroupCollisionFilter(ragdollIndex, lowerArmLocalIndex);
        handFilter = new SubgroupCollisionFilter(ragdollIndex, handLocalIndex);
        SubgroupCollisionFilter.DisableCollision(ref chestMask, ref upperArmFilter);
        SubgroupCollisionFilter.DisableCollision(ref upperArmFilter, ref lowerArmFilter);
        SubgroupCollisionFilter.DisableCollision(ref lowerArmFilter, ref handFilter);

        return handles;
    }

    static RagdollLegHandles AddLeg(Vector3 localHip, RigidPose localHipsPose, BodyHandle hipsHandle, ref SubgroupCollisionFilter hipsFilter,
        int limbBaseBitIndex, int ragdollIndex, RigidPose ragdollPose, CollidableProperty<SubgroupCollisionFilter> filters, SpringSettings constraintSpringSettings, Simulation simulation)
    {
        RagdollLegHandles handles;
        var localKnee = localHip - new Vector3(0, 0.5f, 0);
        var localAnkle = localKnee - new Vector3(0, 0.5f, 0);
        var localFoot = localAnkle + new Vector3(0, -0.075f, 0.05f);
        GetCapsuleForLineSegment(localHip, localKnee, 0.12f, out var upperLegShape, out var upperLegPosition, out var upperLegOrientation);
        handles.UpperLeg = AddBody(upperLegShape, 5, GetWorldPose(upperLegPosition, upperLegOrientation, ragdollPose), simulation);
        GetCapsuleForLineSegment(localKnee, localAnkle, 0.11f, out var lowerLegShape, out var lowerLegPosition, out var lowerLegOrientation);
        handles.LowerLeg = AddBody(lowerLegShape, 5, GetWorldPose(lowerLegPosition, lowerLegOrientation, ragdollPose), simulation);
        handles.Foot = AddBody(new Box(0.2f, 0.15f, 0.3f), 2, GetWorldPose(localFoot, Quaternion.Identity, ragdollPose), simulation);

        //Create joints between limb pieces.
        //Hips-Upper Leg
        simulation.Solver.Add(hipsHandle, handles.UpperLeg, new BallSocket
        {
            LocalOffsetA = QuaternionEx.Transform(localHip - localHipsPose.Position, QuaternionEx.Conjugate(localHipsPose.Orientation)),
            LocalOffsetB = QuaternionEx.Transform(localHip - upperLegPosition, QuaternionEx.Conjugate(upperLegOrientation)),
            SpringSettings = constraintSpringSettings
        });
        simulation.Solver.Add(hipsHandle, handles.UpperLeg, new SwingLimit
        {
            AxisLocalA = QuaternionEx.Transform(Vector3.Normalize(new Vector3(Math.Sign(localHip.X), -1, 0)), QuaternionEx.Conjugate(localHipsPose.Orientation)),
            AxisLocalB = QuaternionEx.Transform(new Vector3(0, -1, 0), QuaternionEx.Conjugate(upperLegOrientation)),
            MaximumSwingAngle = MathHelper.PiOver2,
            SpringSettings = constraintSpringSettings
        });
        simulation.Solver.Add(hipsHandle, handles.UpperLeg, new TwistLimit
        {
            LocalBasisA = QuaternionEx.Concatenate(CreateBasis(new Vector3(0, -1, 0), new Vector3(0, 0, 1)), QuaternionEx.Conjugate(localHipsPose.Orientation)),
            LocalBasisB = QuaternionEx.Concatenate(CreateBasis(new Vector3(0, -1, 0), new Vector3(0, 0, 1)), QuaternionEx.Conjugate(upperLegOrientation)),
            MinimumAngle = localHip.X < 0 ? MathHelper.Pi * -0.05f : MathHelper.Pi * -0.55f,
            MaximumAngle = localHip.X < 0 ? MathHelper.Pi * 0.55f : MathHelper.Pi * 0.05f,
            SpringSettings = constraintSpringSettings
        });
        simulation.Solver.Add(hipsHandle, handles.UpperLeg, BuildAngularMotor());

        //Upper Leg-Lower Leg
        simulation.Solver.Add(handles.UpperLeg, handles.LowerLeg, new Hinge
        {
            LocalHingeAxisA = QuaternionEx.Transform(new Vector3(1, 0, 0), QuaternionEx.Conjugate(upperLegOrientation)),
            LocalOffsetA = QuaternionEx.Transform(localKnee - upperLegPosition, QuaternionEx.Conjugate(upperLegOrientation)),
            LocalHingeAxisB = QuaternionEx.Transform(new Vector3(1, 0, 0), QuaternionEx.Conjugate(lowerLegOrientation)),
            LocalOffsetB = QuaternionEx.Transform(localKnee - lowerLegPosition, QuaternionEx.Conjugate(lowerLegOrientation)),
            SpringSettings = constraintSpringSettings
        });
        simulation.Solver.Add(handles.UpperLeg, handles.LowerLeg, new SwingLimit
        {
            AxisLocalA = QuaternionEx.Transform(new Vector3(0, 0, 1), QuaternionEx.Conjugate(upperLegOrientation)),
            AxisLocalB = QuaternionEx.Transform(new Vector3(0, 1, 0), QuaternionEx.Conjugate(lowerLegOrientation)),
            MaximumSwingAngle = MathHelper.PiOver2,
            SpringSettings = constraintSpringSettings
        });
        simulation.Solver.Add(handles.UpperLeg, handles.LowerLeg, BuildAngularMotor());

        //Lower Leg-Foot
        simulation.Solver.Add(handles.LowerLeg, handles.Foot, new BallSocket
        {
            LocalOffsetA = QuaternionEx.Transform(localAnkle - lowerLegPosition, QuaternionEx.Conjugate(lowerLegOrientation)),
            LocalOffsetB = localAnkle - localFoot,
            SpringSettings = constraintSpringSettings
        });
        simulation.Solver.Add(handles.LowerLeg, handles.Foot, new SwingLimit
        {
            AxisLocalA = QuaternionEx.Transform(new Vector3(0, 1, 0), QuaternionEx.Conjugate(lowerLegOrientation)),
            AxisLocalB = new Vector3(0, 1, 0),
            MaximumSwingAngle = 1,
            SpringSettings = constraintSpringSettings
        });
        simulation.Solver.Add(handles.LowerLeg, handles.Foot, new TwistServo
        {
            LocalBasisA = QuaternionEx.Concatenate(CreateBasis(new Vector3(0, 1, 0), new Vector3(0, 0, 1)), QuaternionEx.Conjugate(lowerLegOrientation)),
            LocalBasisB = CreateBasis(new Vector3(0, 1, 0), new Vector3(0, 0, 1)),
            TargetAngle = 0,
            SpringSettings = constraintSpringSettings,
            ServoSettings = new ServoSettings(float.MaxValue, 0, float.MaxValue)
        });
        simulation.Solver.Add(handles.LowerLeg, handles.Foot, BuildAngularMotor());

        //Disable collisions between connected ragdoll pieces.
        var upperLegLocalIndex = limbBaseBitIndex;
        var lowerLegLocalIndex = limbBaseBitIndex + 1;
        var footLocalIndex = limbBaseBitIndex + 2;
        ref var upperLegFilter = ref filters.Allocate(handles.UpperLeg);
        ref var lowerLegFilter = ref filters.Allocate(handles.LowerLeg);
        ref var footFilter = ref filters.Allocate(handles.Foot);
        upperLegFilter = new SubgroupCollisionFilter(ragdollIndex, upperLegLocalIndex);
        lowerLegFilter = new SubgroupCollisionFilter(ragdollIndex, lowerLegLocalIndex);
        footFilter = new SubgroupCollisionFilter(ragdollIndex, footLocalIndex);
        SubgroupCollisionFilter.DisableCollision(ref hipsFilter, ref upperLegFilter);
        SubgroupCollisionFilter.DisableCollision(ref upperLegFilter, ref lowerLegFilter);
        SubgroupCollisionFilter.DisableCollision(ref lowerLegFilter, ref footFilter);
        return handles;
    }

    public struct RagdollArmHandles
    {
        public BodyHandle UpperArm;
        public BodyHandle LowerArm;
        public BodyHandle Hand;
    }
    public struct RagdollLegHandles
    {
        public BodyHandle UpperLeg;
        public BodyHandle LowerLeg;
        public BodyHandle Foot;
    }
    public struct RagdollHandles
    {
        public BodyHandle Head;
        public BodyHandle Chest;
        public BodyHandle Abdomen;
        public BodyHandle Hips;
        public RagdollArmHandles LeftArm;
        public RagdollArmHandles RightArm;
        public RagdollLegHandles LeftLeg;
        public RagdollLegHandles RightLeg;
    }

    public static RagdollHandles AddRagdoll(Vector3 position, Quaternion orientation, int ragdollIndex, CollidableProperty<SubgroupCollisionFilter> collisionFilters, Simulation simulation)
    {
        var ragdollPose = new RigidPose { Position = position, Orientation = orientation };
        var horizontalOrientation = QuaternionEx.CreateFromAxisAngle(new Vector3(0, 0, 1), MathHelper.PiOver2);
        RagdollHandles handles;
        var hipsPose = new RigidPose { Position = new Vector3(0, 1.1f, 0), Orientation = horizontalOrientation };
        handles.Hips = AddBody(new Capsule(0.17f, 0.25f), 8, GetWorldPose(hipsPose.Position, hipsPose.Orientation, ragdollPose), simulation);
        var abdomenPose = new RigidPose { Position = new Vector3(0, 1.3f, 0), Orientation = horizontalOrientation };
        handles.Abdomen = AddBody(new Capsule(0.17f, 0.22f), 7, GetWorldPose(abdomenPose.Position, abdomenPose.Orientation, ragdollPose), simulation);
        var chestPose = new RigidPose { Position = new Vector3(0, 1.6f, 0), Orientation = horizontalOrientation };
        handles.Chest = AddBody(new Capsule(0.21f, 0.3f), 10, GetWorldPose(chestPose.Position, chestPose.Orientation, ragdollPose), simulation);
        var headPose = new RigidPose { Position = new Vector3(0, 2.05f, 0), Orientation = Quaternion.Identity };
        handles.Head = AddBody(new Sphere(0.2f), 5, GetWorldPose(headPose.Position, headPose.Orientation, ragdollPose), simulation);

        //Attach constraints between torso pieces.
        var springSettings = new SpringSettings(15f, 1f);
        var lowerSpine = (hipsPose.Position + abdomenPose.Position) * 0.5f;
        //Hips-Abdomen
        simulation.Solver.Add(handles.Hips, handles.Abdomen, new BallSocket
        {
            LocalOffsetA = QuaternionEx.Transform(lowerSpine - hipsPose.Position, QuaternionEx.Conjugate(hipsPose.Orientation)),
            LocalOffsetB = QuaternionEx.Transform(lowerSpine - abdomenPose.Position, QuaternionEx.Conjugate(abdomenPose.Orientation)),
            SpringSettings = springSettings
        });
        simulation.Solver.Add(handles.Hips, handles.Abdomen, new SwingLimit
        {
            AxisLocalA = QuaternionEx.Transform(new Vector3(0, 1, 0), QuaternionEx.Conjugate(hipsPose.Orientation)),
            AxisLocalB = QuaternionEx.Transform(new Vector3(0, 1, 0), QuaternionEx.Conjugate(abdomenPose.Orientation)),
            MaximumSwingAngle = MathHelper.Pi * 0.27f,
            SpringSettings = springSettings
        });
        simulation.Solver.Add(handles.Hips, handles.Abdomen, new TwistLimit
        {
            LocalBasisA = QuaternionEx.Concatenate(CreateBasis(new Vector3(0, 1, 0), new Vector3(1, 0, 0)), QuaternionEx.Conjugate(hipsPose.Orientation)),
            LocalBasisB = QuaternionEx.Concatenate(CreateBasis(new Vector3(0, 1, 0), new Vector3(1, 0, 0)), QuaternionEx.Conjugate(abdomenPose.Orientation)),
            MinimumAngle = MathHelper.Pi * -0.2f,
            MaximumAngle = MathHelper.Pi * 0.2f,
            SpringSettings = springSettings
        });
        simulation.Solver.Add(handles.Hips, handles.Abdomen, BuildAngularMotor());

        //Abdomen-Chest
        var upperSpine = (abdomenPose.Position + chestPose.Position) * 0.5f;
        simulation.Solver.Add(handles.Abdomen, handles.Chest, new BallSocket
        {
            LocalOffsetA = QuaternionEx.Transform(upperSpine - abdomenPose.Position, QuaternionEx.Conjugate(abdomenPose.Orientation)),
            LocalOffsetB = QuaternionEx.Transform(upperSpine - chestPose.Position, QuaternionEx.Conjugate(chestPose.Orientation)),
            SpringSettings = springSettings
        });
        simulation.Solver.Add(handles.Abdomen, handles.Chest, new SwingLimit
        {
            AxisLocalA = QuaternionEx.Transform(new Vector3(0, 1, 0), QuaternionEx.Conjugate(abdomenPose.Orientation)),
            AxisLocalB = QuaternionEx.Transform(new Vector3(0, 1, 0), QuaternionEx.Conjugate(chestPose.Orientation)),
            MaximumSwingAngle = MathHelper.Pi * 0.27f,
            SpringSettings = springSettings
        });
        simulation.Solver.Add(handles.Abdomen, handles.Chest, new TwistLimit
        {
            LocalBasisA = QuaternionEx.Concatenate(CreateBasis(new Vector3(0, 1, 0), new Vector3(1, 0, 0)), QuaternionEx.Conjugate(abdomenPose.Orientation)),
            LocalBasisB = QuaternionEx.Concatenate(CreateBasis(new Vector3(0, 1, 0), new Vector3(1, 0, 0)), QuaternionEx.Conjugate(chestPose.Orientation)),
            MinimumAngle = MathHelper.Pi * -0.2f,
            MaximumAngle = MathHelper.Pi * 0.2f,
            SpringSettings = springSettings
        });
        simulation.Solver.Add(handles.Abdomen, handles.Chest, BuildAngularMotor());

        //Chest-Head
        var neck = (headPose.Position + chestPose.Position) * 0.5f;
        simulation.Solver.Add(handles.Chest, handles.Head, new BallSocket
        {
            LocalOffsetA = QuaternionEx.Transform(neck - chestPose.Position, QuaternionEx.Conjugate(chestPose.Orientation)),
            LocalOffsetB = neck - headPose.Position,
            SpringSettings = springSettings
        });
        simulation.Solver.Add(handles.Chest, handles.Head, new SwingLimit
        {
            AxisLocalA = QuaternionEx.Transform(new Vector3(0, 1, 0), QuaternionEx.Conjugate(chestPose.Orientation)),
            AxisLocalB = new Vector3(0, 1, 0),
            MaximumSwingAngle = MathHelper.PiOver2 * 0.9f,
            SpringSettings = springSettings
        });
        simulation.Solver.Add(handles.Chest, handles.Head, new TwistLimit
        {
            LocalBasisA = QuaternionEx.Concatenate(CreateBasis(new Vector3(0, 1, 0), new Vector3(1, 0, 0)), QuaternionEx.Conjugate(chestPose.Orientation)),
            LocalBasisB = QuaternionEx.Concatenate(CreateBasis(new Vector3(0, 1, 0), new Vector3(1, 0, 0)), QuaternionEx.Conjugate(headPose.Orientation)),
            MinimumAngle = MathHelper.Pi * -0.5f,
            MaximumAngle = MathHelper.Pi * 0.5f,
            SpringSettings = springSettings
        });
        simulation.Solver.Add(handles.Chest, handles.Head, BuildAngularMotor());

        var hipsLocalIndex = 0;
        var abdomenLocalIndex = 1;
        var chestLocalIndex = 2;
        var headLocalIndex = 3;
        ref var hipsFilter = ref collisionFilters.Allocate(handles.Hips);
        ref var abdomenFilter = ref collisionFilters.Allocate(handles.Abdomen);
        ref var chestFilter = ref collisionFilters.Allocate(handles.Chest);
        ref var headFilter = ref collisionFilters.Allocate(handles.Head);
        hipsFilter = new SubgroupCollisionFilter(ragdollIndex, hipsLocalIndex);
        abdomenFilter = new SubgroupCollisionFilter(ragdollIndex, abdomenLocalIndex);
        chestFilter = new SubgroupCollisionFilter(ragdollIndex, chestLocalIndex);
        headFilter = new SubgroupCollisionFilter(ragdollIndex, headLocalIndex);
        //Disable collisions in the torso and head.
        SubgroupCollisionFilter.DisableCollision(ref hipsFilter, ref abdomenFilter);
        SubgroupCollisionFilter.DisableCollision(ref abdomenFilter, ref chestFilter);
        SubgroupCollisionFilter.DisableCollision(ref chestFilter, ref headFilter);

        //Build all the limbs. Setting the masks is delayed until after the limbs have been created and have disabled collisions with the chest/hips.
        handles.RightArm = AddArm(1, chestPose.Position + new Vector3(0.4f, 0.1f, 0), chestPose, handles.Chest, ref chestFilter, 4, ragdollIndex, ragdollPose, collisionFilters, springSettings, simulation);
        handles.LeftArm = AddArm(-1, chestPose.Position + new Vector3(-0.4f, 0.1f, 0), chestPose, handles.Chest, ref chestFilter, 7, ragdollIndex, ragdollPose, collisionFilters, springSettings, simulation);
        handles.RightLeg = AddLeg(hipsPose.Position + new Vector3(-0.17f, -0.2f, 0), hipsPose, handles.Hips, ref hipsFilter, 10, ragdollIndex, ragdollPose, collisionFilters, springSettings, simulation);
        handles.LeftLeg = AddLeg(hipsPose.Position + new Vector3(0.17f, -0.2f, 0), hipsPose, handles.Hips, ref hipsFilter, 13, ragdollIndex, ragdollPose, collisionFilters, springSettings, simulation);
        return handles;
    }

    const int timestepCount = 384;
    BufferPool BufferPool;
    Simulation Simulation;

    [IterationSetup]
    public unsafe void Initialize()
    {
        var filters = new CollidableProperty<SubgroupCollisionFilter>();
        BufferPool = new BufferPool();
        Simulation = Simulation.Create(BufferPool, new SubgroupFilteredCallbacks(filters, new PairMaterialProperties(2, float.MaxValue, new SpringSettings(10, 1))), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(4, 1));

        int ragdollIndex = 0;
        var spacing = new Vector3(1.7f, 1.8f, 0.5f);
        int width = 4;
        int height = 4;
        int length = 2;
        var origin = -0.5f * spacing * new Vector3(width - 1, 0, length - 1) + new Vector3(0, 5f, 0);
        for (int i = 0; i < width; ++i)
        {
            for (int j = 0; j < height; ++j)
            {
                for (int k = 0; k < length; ++k)
                {
                    AddRagdoll(origin + spacing * new Vector3(i, j, k), QuaternionEx.CreateFromAxisAngle(new Vector3(0, 1, 0), MathHelper.Pi * 0.05f), ragdollIndex++, filters, Simulation);
                }
            }
        }

        var tubeCenter = new Vector3(0, 8, 0);
        const int panelCount = 20;
        const float tubeRadius = 6;
        var panelShape = new Box(MathF.PI * 2 * tubeRadius / panelCount, 1, 80);
        var panelShapeIndex = Simulation.Shapes.Add(panelShape);
        var builder = new CompoundBuilder(BufferPool, Simulation.Shapes, panelCount + 1);
        for (int i = 0; i < panelCount; ++i)
        {
            var rotation = QuaternionEx.CreateFromAxisAngle(Vector3.UnitZ, i * MathHelper.TwoPi / panelCount);
            QuaternionEx.TransformUnitY(rotation, out var localUp);
            var position = localUp * tubeRadius;
            builder.AddForKinematic(panelShapeIndex, (position, rotation), 1);
        }
        builder.AddForKinematic(Simulation.Shapes.Add(new Box(1, 2, panelShape.Length)), new Vector3(0, tubeRadius - 1, 0), 0);
        builder.BuildKinematicCompound(out var children);
        var compound = new BigCompound(children, Simulation.Shapes, BufferPool);
        var tubeHandle = Simulation.Bodies.Add(BodyDescription.CreateKinematic(tubeCenter, (default, new Vector3(0, 0, .25f)), Simulation.Shapes.Add(compound), 0f));
        filters[tubeHandle] = new SubgroupCollisionFilter(int.MaxValue);
        builder.Dispose();

        var staticShape = new Box(300, 1, 300);
        var staticShapeIndex = Simulation.Shapes.Add(staticShape);
        var staticDescription = new StaticDescription(new Vector3(0, -0.5f, 0), staticShapeIndex);
        Simulation.Statics.Add(staticDescription);
    }

    [IterationCleanup]
    public void CleanUp()
    {
        BufferPool.Clear();
        BufferPool = null;
    }

    [Benchmark]
    public unsafe void RagdollTubeBenchmarks()
    {
        for (int i = 0; i < timestepCount; ++i)
        {
            Simulation.Timestep(1 / 60f);
        }
    }
}
