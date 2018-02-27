using BepuUtilities;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Collidables;
using System.Numerics;
using Quaternion = BepuUtilities.Quaternion;
using System;
using BepuPhysics.CollisionDetection;
using System.Runtime.CompilerServices;
using System.Diagnostics;

namespace Demos
{
    //TODO: This would benefit from some convenience work related to storing custom per body data. Callbacks on body add/remove and so on.
    public class BodyCollisionMasks
    {
        ulong[] masks;
        /// <summary>
        /// Gets the mask associated with a body's handle.
        /// </summary>
        /// <param name="bodyHandle">Body handle to retrieve the collision mask of.</param>
        /// <returns>Collision mask associated with a body handle.</returns>
        public ulong this[int bodyHandle]
        {
            get
            {
                Debug.Assert(bodyHandle >= 0 && bodyHandle < masks.Length, "This collection assumes that all bodies are given a handle for simplicity.");
                return masks[bodyHandle];
            }
            set
            {
                if (masks == null || masks.Length <= bodyHandle)
                {
                    Array.Resize(ref masks, bodyHandle * 2);
                }
                masks[bodyHandle] = value;
            }
        }
    }

    //For the purposes of this demo, we have custom collision filtering rules.
    struct RagdollCallbacks : INarrowPhaseCallbacks
    {
        public BodyCollisionMasks Masks;
        public void Initialize(Simulation simulation)
        {
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b)
        {
            if (a.Mobility == CollidableMobility.Dynamic && b.Mobility == CollidableMobility.Dynamic)
            {
                //The upper 32 bits of the mask hold the ragdoll instance id. Different instances are always allowed to collide.
                var maskA = Masks[a.Handle];
                var maskB = Masks[b.Handle];
                const ulong upperMask = ((ulong)uint.MaxValue << 32);
                if ((maskA & upperMask) != (maskB & upperMask))
                    return true;
                //Bits 0 through 15 contain which local collision groups a body belongs to.
                //Bits 16 through 31 contain which local collision groups a given body will collide with. 
                //Note that this only tests a's accepted groups against b's membership, instead of both directions.
                const ulong lower16Mask = ((1 << 16) - 1);
                return (((maskA >> 16) & maskB) & lower16Mask) > 0;

                //This demo will ensure symmetry for simplicity. Optionally, you could make use of the fact that collidable references obey an order;
                //the lower valued handle will always be CollidableReference a. Static collidables will always be in CollidableReference b if they exist.
            }
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
        {
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, ContactManifold* manifold, out PairMaterialProperties pairMaterial)
        {
            pairMaterial.FrictionCoefficient = 1;
            pairMaterial.MaximumRecoveryVelocity = 2f;
            pairMaterial.SpringSettings.NaturalFrequency = MathHelper.Pi * 60;
            pairMaterial.SpringSettings.DampingRatio = 1f;
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ContactManifold* manifold)
        {
            return true;
        }

        public void Dispose()
        {
        }
    }

    public class BasicRagdollDemo : Demo
    {
        static int AddBody<TShape>(TShape shape, float mass, RigidPose pose, Simulation simulation) where TShape : struct, IShape
        {
            BodyInertia inertia;
            inertia.InverseMass = 1f / mass;
            shape.ComputeLocalInverseInertia(inertia.InverseMass, out inertia.InverseInertiaTensor);
            var description = new BodyDescription
            {
                Activity = new BodyActivityDescription { SleepThreshold = 0.01f, MinimumTimestepCountUnderThreshold = 32 },
                Collidable = new CollidableDescription
                {
                    Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                    SpeculativeMargin = .01f,
                    //Note that this always registers a new shape instance. You could be more clever/efficient and share shapes, but the goal here is to show the most basic option.
                    //Also, the cost of registering different shapes isn't that high for tiny implicit shapes.
                    Shape = simulation.Shapes.Add(ref shape)
                },
                LocalInertia = inertia,
                Pose = pose
            };
            return simulation.Bodies.Add(ref description);
        }

        static RigidPose GetWorldPose(Vector3 localPosition, Quaternion localOrientation, RigidPose ragdollPose)
        {
            RigidPose worldPose;
            RigidPose.Transform(ref localPosition, ref ragdollPose, out worldPose.Position);
            Quaternion.ConcatenateWithoutOverlap(ref localOrientation, ref ragdollPose.Orientation, out worldPose.Orientation);
            return worldPose;
        }
        static void GetCapsuleForLineSegment(Vector3 start, Vector3 end, float radius, out Capsule capsule, out Vector3 position, out Quaternion orientation)
        {
            position = 0.5f * (start + end);

            var offset = end - start;
            capsule.HalfLength = 0.5f * offset.Length();
            capsule.Radius = radius;
            //The capsule shape's length is along its local Y axis, so get the shortest rotation from Y to the current orientation.
            var cross = Vector3.Cross(offset / capsule.Length, new Vector3(0, 1, 0));
            var crossLength = cross.Length();
            orientation = crossLength > 1e-8f ? Quaternion.CreateFromAxisAngle(cross / crossLength, (float)Math.Asin(crossLength)) : Quaternion.Identity;
        }

        static ulong BuildCollisionFilteringMask(int ragdollIndex, int localBodyIndex)
        {
            ulong instanceId = (ulong)ragdollIndex << 32;
            //Note that we initialize allowed collisions to all groups.
            ulong acceptedCollisionGroups = ((1ul << 16) - 1) << 16;
            Debug.Assert(localBodyIndex >= 0 && localBodyIndex < 16, "The mask is set up to only handle 16 distinct ragdoll pieces.");
            ulong membership = (ulong)(1 << localBodyIndex);
            return instanceId | acceptedCollisionGroups | membership;
        }

        static void DisableCollision(ref ulong maskA, int localBodyIndexA, ref ulong maskB, int localBodyIndexB)
        {
            maskA ^= 1ul << (localBodyIndexB + 16);
            maskB ^= 1ul << (localBodyIndexA + 16);
        }

        static void AddArm(float sign, Vector3 localShoulder, int chestHandle, int chestLocalIndex, ref ulong chestMask,
            int limbBaseBitIndex, int ragdollIndex, RigidPose ragdollPose, BodyCollisionMasks masks, Simulation simulation)
        {
            var localElbow = localShoulder + new Vector3(sign * 0.6f, 0, 0);
            var localWrist = localElbow + new Vector3(sign * 0.6f, 0, 0);
            var localHand = localWrist + new Vector3(sign * 0.1f, 0, 0);
            GetCapsuleForLineSegment(localShoulder, localElbow, 0.13f, out var upperArmShape, out var upperArmPosition, out var upperArmOrientation);
            var upperArm = AddBody(upperArmShape, 5, GetWorldPose(upperArmPosition, upperArmOrientation, ragdollPose), simulation);
            GetCapsuleForLineSegment(localElbow, localWrist, 0.1f, out var lowerArmShape, out var lowerArmPosition, out var lowerArmOrientation);
            var lowerArm = AddBody(lowerArmShape, 5, GetWorldPose(lowerArmPosition, lowerArmOrientation, ragdollPose), simulation);
            var hand = AddBody(new Box(0.2f, 0.1f, 0.2f), 2, GetWorldPose(localHand, Quaternion.Identity, ragdollPose), simulation);

            //Disable collisions between connected ragdoll pieces.
            var upperArmLocalIndex = limbBaseBitIndex;
            var lowerArmLocalIndex = limbBaseBitIndex + 1;
            var handLocalIndex = limbBaseBitIndex + 2;
            var upperArmMask = BuildCollisionFilteringMask(ragdollIndex, upperArmLocalIndex);
            var lowerArmMask = BuildCollisionFilteringMask(ragdollIndex, lowerArmLocalIndex);
            var handMask = BuildCollisionFilteringMask(ragdollIndex, handLocalIndex);
            DisableCollision(ref chestMask, chestLocalIndex, ref upperArmMask, upperArmLocalIndex);
            DisableCollision(ref upperArmMask, upperArmLocalIndex, ref lowerArmMask, lowerArmLocalIndex);
            DisableCollision(ref lowerArmMask, lowerArmLocalIndex, ref handMask, handLocalIndex);
            masks[upperArm] = upperArmMask;
            masks[lowerArm] = lowerArmMask;
            masks[hand] = handMask;
        }

        static void AddLeg(Vector3 localHip, int hipsHandle, int hipsLocalIndex, ref ulong hipsMask,
            int limbBaseBitIndex, int ragdollIndex, RigidPose ragdollPose, BodyCollisionMasks masks, Simulation simulation)
        {
            var localKnee = localHip - new Vector3(0, 0.6f, 0);
            var localAnkle = localKnee - new Vector3(0, 0.6f, 0);
            var localFoot = localAnkle + new Vector3(0, -0.075f, 0.15f);
            GetCapsuleForLineSegment(localHip, localKnee, 0.17f, out var upperLegShape, out var upperArmPosition, out var upperArmOrientation);
            var upperLeg = AddBody(upperLegShape, 5, GetWorldPose(upperArmPosition, upperArmOrientation, ragdollPose), simulation);
            GetCapsuleForLineSegment(localKnee, localAnkle, 0.13f, out var lowerLegShape, out var lowerArmPosition, out var lowerArmOrientation);
            var lowerLeg = AddBody(lowerLegShape, 5, GetWorldPose(lowerArmPosition, lowerArmOrientation, ragdollPose), simulation);
            var foot = AddBody(new Box(0.2f, 0.15f, 0.3f), 2, GetWorldPose(localFoot, Quaternion.Identity, ragdollPose), simulation);

            //Disable collisions between connected ragdoll pieces.
            var upperLegLocalIndex = limbBaseBitIndex;
            var lowerLegLocalIndex = limbBaseBitIndex + 1;
            var footLocalIndex = limbBaseBitIndex + 2;
            var upperLegMask = BuildCollisionFilteringMask(ragdollIndex, upperLegLocalIndex);
            var lowerLegMask = BuildCollisionFilteringMask(ragdollIndex, lowerLegLocalIndex);
            var footMask = BuildCollisionFilteringMask(ragdollIndex, footLocalIndex);
            DisableCollision(ref hipsMask, hipsLocalIndex, ref upperLegMask, upperLegLocalIndex);
            DisableCollision(ref upperLegMask, upperLegLocalIndex, ref lowerLegMask, lowerLegLocalIndex);
            DisableCollision(ref lowerLegMask, lowerLegLocalIndex, ref footMask, footLocalIndex);
            masks[upperLeg] = upperLegMask;
            masks[lowerLeg] = lowerLegMask;
            masks[foot] = footMask;
        }

        static void AddRagdoll(Vector3 position, Quaternion orientation, int ragdollIndex, BodyCollisionMasks masks, Simulation simulation)
        {
            var ragdollPose = new RigidPose { Position = position, Orientation = orientation };
            var horizontalOrientation = Quaternion.CreateFromAxisAngle(new Vector3(0, 0, 1), MathHelper.PiOver2);
            var hipsPosition = new Vector3(0, 1.1f, 0);
            var hips = AddBody(new Capsule(0.25f, 0.3f), 8, GetWorldPose(hipsPosition, horizontalOrientation, ragdollPose), simulation);
            var abdomen = AddBody(new Capsule(0.25f, 0.25f), 7, GetWorldPose(new Vector3(0, 1.35f, 0), horizontalOrientation, ragdollPose), simulation);
            var chestPosition = new Vector3(0, 1.7f, 0);
            var chest = AddBody(new Capsule(0.3f, 0.35f), 10, GetWorldPose(chestPosition, horizontalOrientation, ragdollPose), simulation);
            var head = AddBody(new Sphere(0.3f), 5, GetWorldPose(new Vector3(0, 2.5f, 0), Quaternion.Identity, ragdollPose), simulation);

            var hipsLocalIndex = 0;
            var abdomenLocalIndex = 1;
            var chestLocalIndex = 2;
            var headLocalIndex = 3;
            var hipsMask = BuildCollisionFilteringMask(ragdollIndex, hipsLocalIndex);
            var abdomenMask = BuildCollisionFilteringMask(ragdollIndex, abdomenLocalIndex);
            var chestMask = BuildCollisionFilteringMask(ragdollIndex, chestLocalIndex);
            var headMask = BuildCollisionFilteringMask(ragdollIndex, headLocalIndex);
            //Disable collisions in the torso and head.
            DisableCollision(ref hipsMask, hipsLocalIndex, ref abdomenMask, abdomenLocalIndex);
            DisableCollision(ref abdomenMask, abdomenLocalIndex, ref chestMask, chestLocalIndex);
            DisableCollision(ref chestMask, chestLocalIndex, ref headMask, headLocalIndex);

            //Build all the limbs. Setting the masks is delayed until after the limbs have been created and have disabled collisions with the chest/hips.
            AddArm(1, chestPosition + new Vector3(0.75f, 0.3f, 0), chest, chestLocalIndex, ref chestMask, 4, ragdollIndex, ragdollPose, masks, simulation);
            AddArm(-1, chestPosition + new Vector3(-0.75f, 0.3f, 0), chest, chestLocalIndex, ref chestMask, 7, ragdollIndex, ragdollPose, masks, simulation);
            AddLeg(hipsPosition + new Vector3(-0.4f, -0.3f, 0), hips, hipsLocalIndex, ref hipsMask, 10, ragdollIndex, ragdollPose, masks, simulation);
            AddLeg(hipsPosition + new Vector3(0.4f, -0.3f, 0), hips, hipsLocalIndex, ref hipsMask, 13, ragdollIndex, ragdollPose, masks, simulation);

            masks[hips] = hipsMask;
            masks[abdomen] = abdomenMask;
            masks[chest] = chestMask;
            masks[head] = headMask;
        }

        public unsafe override void Initialize(Camera camera)
        {
            camera.Position = new Vector3(-20, 10, -20);
            //camera.Yaw = MathHelper.Pi ; 
            camera.Yaw = MathHelper.Pi * 3f / 4;
            //camera.Pitch = MathHelper.PiOver2 * 0.999f;
            var masks = new BodyCollisionMasks();
            var callbacks = new RagdollCallbacks { Masks = masks };
            Simulation = Simulation.Create(BufferPool, callbacks);
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);

            int ragdollIndex = 0;
            AddRagdoll(new Vector3(0, 10, 0), Quaternion.Identity, ragdollIndex, masks, Simulation);


            var staticShape = new Box(100, 1, 100);
            var staticShapeIndex = Simulation.Shapes.Add(ref staticShape);
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
                    Position = new Vector3(0, -0.5f, 0),
                    Orientation = BepuUtilities.Quaternion.Identity
                }
            };
            Simulation.Statics.Add(ref staticDescription);


        }

    }
}


