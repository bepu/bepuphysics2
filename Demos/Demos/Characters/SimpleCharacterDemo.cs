using BepuUtilities;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using System.Numerics;
using System;
using System.Runtime.CompilerServices;
using BepuPhysics.Constraints;
using DemoContentLoader;
using DemoUtilities;
using DemoRenderer.UI;
using OpenTK.Input;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System.Diagnostics;

namespace Demos.Demos.Characters;

// I wanted to see if claude could write a simplified character controller without constraints. 
// Turns out, yes, it can, entirely by itself.
// I've glanced through the code and everything seems reasonable, though it
// does import some of the structural complexity of the full CharacterControllers.
//
// Critically, this simple approach has a timing problem when using solver substepping.
// The velocity modification runs once (in the CollisionsDetected callback) before the solver starts,
// but gravity is integrated at the beginning of *each* substep. So even if we perfectly set the velocity
// to stick to a ramp, the first substep immediately integrates gravity and adds downward velocity.
// With no character constraint participating in the solver, there's nothing to fight back, and the
// character slides down slopes.
// 
// The constraint-based CharacterControllers doesn't have this problem because the CharacterMotionConstraint
// participates in every solver substep iteration, continuously counteracting integrated gravity.
// A future update may change the timestepper to integrate velocity only once per timestep (rather than
// per-substep), which would allow this simple approach to work correctly. For now, this demo is best
// understood as an educational tool showing the underlying concept.


/// <summary>
/// A simplified version of the character controller that demonstrates the same concepts as the full CharacterController,
/// but without using explicit constraint infrastructure, multithreading, or AoSoA data layouts.
/// This is intended purely for educational purposes to show the underlying math in a more accessible form.
/// </summary>
/// <remarks>
/// <para>
/// The full CharacterController uses custom constraints that run during the solver phase. This allows the character
/// to respond robustly to interactions with dynamic objects- if a heavy crate pushes against the character,
/// the constraint solver will find a balanced solution that respects both the character's movement goals and the physics.
/// </para>
/// <para>
/// This simplified version instead modifies velocities directly between timesteps. It applies the same conceptual process,
/// but happens outside the solver. This means it won't handle resistance as robustly- if something pushes back,
/// this simple approach just overwrites the velocity without properly accounting for momentum exchange.
/// </para>
/// <para>
/// The key simplification is replacing the constraint math (which uses jacobians, effective mass, and iterative solving)
/// with direct velocity modification. We're essentially doing one iteration of a constraint solve, but outside the solver
/// and without the infrastructure that allows convergence to a balanced solution.
/// </para>
/// </remarks>
public struct SimpleCharacter
{
    /// <summary>
    /// Handle of the body associated with the character.
    /// </summary>
    public BodyHandle BodyHandle;

    /// <summary>
    /// Direction the character is looking in world space. Defines the forward direction for movement.
    /// </summary>
    public Vector3 ViewDirection;

    /// <summary>
    /// Target horizontal velocity.
    /// X component is velocity along the strafing direction (perpendicular to the view direction projected onto the surface).
    /// Y component is velocity along the forward direction (aligned with the view direction projected onto the surface).
    /// </summary>
    public Vector2 TargetVelocity;

    /// <summary>
    /// If true, the character will try to jump on the next time step. Will be reset to false after being processed.
    /// </summary>
    public bool TryJump;

    /// <summary>
    /// Character's up direction in the local space of the character's body.
    /// </summary>
    public Vector3 LocalUp;

    /// <summary>
    /// Velocity at which the character pushes off the support during a jump.
    /// </summary>
    public float JumpVelocity;

    /// <summary>
    /// Maximum force the character can apply tangent to the supporting surface to move.
    /// </summary>
    public float MaximumHorizontalForce;

    /// <summary>
    /// Maximum force the character can apply to glue itself to the supporting surface.
    /// </summary>
    public float MaximumVerticalForce;

    /// <summary>
    /// Cosine of the maximum slope angle that the character can treat as a support.
    /// </summary>
    public float CosMaximumSlope;

    /// <summary>
    /// Depth threshold beyond which a contact is considered a support if the normal allows it.
    /// </summary>
    public float MinimumSupportDepth;

    /// <summary>
    /// Depth threshold beyond which a contact is considered a support if the previous frame had support.
    /// </summary>
    public float MinimumSupportContinuationDepth;

    /// <summary>
    /// Whether the character is currently supported.
    /// </summary>
    public bool Supported;

    /// <summary>
    /// Collidable currently supporting the character, if any. Only valid if Supported is true.
    /// </summary>
    public CollidableReference Support;
}

/// <summary>
/// Stores information about a potential support contact for the simple character controller.
/// </summary>
public struct SimpleSupportCandidate
{
    public Vector3 OffsetFromCharacter;
    public Vector3 OffsetFromSupport;
    public Vector3 Normal;
    public float Depth;
    public CollidableReference Support;
}

/// <summary>
/// Simplified character controller system that uses direct velocity modification rather than constraints.
/// </summary>
/// <remarks>
/// <para>
/// This controller demonstrates the same motion concepts as the full CharacterControllers, but in a simpler form.
/// The key insight is that character motion on a surface can be broken into two parts:
/// </para>
/// <para>
/// 1. Horizontal motion: The character tries to reach a target velocity along the surface.
///    We compute the velocity error (target - current) and apply an acceleration to reduce it,
///    limited by the maximum horizontal force.
/// </para>
/// <para>
/// 2. Vertical motion: The character tries to stick to the support surface.
///    If the character is separating from the support, we apply an acceleration to reduce that separation velocity,
///    limited by the maximum vertical force. Unlike horizontal motion, this only pulls (never pushes away).
/// </para>
/// </remarks>
public class SimpleCharacterControllers : IDisposable
{
    public Simulation Simulation { get; private set; }
    BufferPool pool;

    Buffer<int> bodyHandleToCharacterIndex;
    QuickList<SimpleCharacter> characters;
    Buffer<SimpleSupportCandidate> supportCandidates;

    public int CharacterCount => characters.Count;

    public SimpleCharacterControllers(BufferPool pool, int initialCharacterCapacity = 64, int initialBodyHandleCapacity = 64)
    {
        this.pool = pool;
        characters = new QuickList<SimpleCharacter>(initialCharacterCapacity, pool);
        pool.Take(initialCharacterCapacity, out supportCandidates);
        ResizeBodyHandleCapacity(initialBodyHandleCapacity);
    }

    public void Initialize(Simulation simulation)
    {
        Simulation = simulation;
        // We hook into the timestepper events to:
        // 1. Prepare for contact collection before collision detection
        // 2. Apply velocity modifications after collision detection but before solving
        simulation.Timestepper.BeforeCollisionDetection += PrepareForContacts;
        simulation.Timestepper.CollisionsDetected += ApplyCharacterMotion;
    }

    void ResizeBodyHandleCapacity(int bodyHandleCapacity)
    {
        var oldCapacity = bodyHandleToCharacterIndex.Length;
        pool.ResizeToAtLeast(ref bodyHandleToCharacterIndex, bodyHandleCapacity, bodyHandleToCharacterIndex.Length);
        if (bodyHandleToCharacterIndex.Length > oldCapacity)
        {
            Unsafe.InitBlockUnaligned(ref Unsafe.As<int, byte>(ref bodyHandleToCharacterIndex[oldCapacity]),
                0xFF, (uint)((bodyHandleToCharacterIndex.Length - oldCapacity) * sizeof(int)));
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public ref SimpleCharacter GetCharacterByBodyHandle(BodyHandle bodyHandle)
    {
        Debug.Assert(bodyHandle.Value >= 0 && bodyHandle.Value < bodyHandleToCharacterIndex.Length &&
            bodyHandleToCharacterIndex[bodyHandle.Value] >= 0);
        return ref characters[bodyHandleToCharacterIndex[bodyHandle.Value]];
    }

    public ref SimpleCharacter AllocateCharacter(BodyHandle bodyHandle)
    {
        if (bodyHandle.Value >= bodyHandleToCharacterIndex.Length)
            ResizeBodyHandleCapacity(Math.Max(bodyHandle.Value + 1, bodyHandleToCharacterIndex.Length * 2));

        var characterIndex = characters.Count;

        // Ensure support candidates buffer is large enough
        if (characterIndex >= supportCandidates.Length)
        {
            pool.ResizeToAtLeast(ref supportCandidates, characters.Span.Length * 2, supportCandidates.Length);
        }

        ref var character = ref characters.Allocate(pool);
        character = default;
        character.BodyHandle = bodyHandle;
        bodyHandleToCharacterIndex[bodyHandle.Value] = characterIndex;
        return ref character;
    }

    public void RemoveCharacterByBodyHandle(BodyHandle bodyHandle)
    {
        var characterIndex = bodyHandleToCharacterIndex[bodyHandle.Value];
        bodyHandleToCharacterIndex[bodyHandle.Value] = -1;
        characters.FastRemoveAt(characterIndex);
        if (characters.Count > characterIndex)
        {
            bodyHandleToCharacterIndex[characters[characterIndex].BodyHandle.Value] = characterIndex;
        }
    }

    /// <summary>
    /// Called before collision detection to reset support candidate data.
    /// </summary>
    void PrepareForContacts(float dt, IThreadDispatcher threadDispatcher)
    {
        // Reset all support candidates to indicate no support found yet
        for (int i = 0; i < characters.Count; ++i)
        {
            supportCandidates[i].Depth = float.MinValue;
        }
    }

    /// <summary>
    /// Tries to report a contact as a potential support for a character.
    /// This is called from the narrow phase callbacks during collision detection.
    /// </summary>
    /// <remarks>
    /// <para>
    /// For educational simplicity, this implementation is single-threaded. The full CharacterControllers
    /// uses per-worker caches to enable multithreaded contact processing, then merges results afterward.
    /// </para>
    /// <para>
    /// The support selection heuristic is simple: pick the deepest contact that has a valid support normal.
    /// A valid support normal is one that points mostly upward relative to the character (within the maximum slope angle).
    /// </para>
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryReportContact<TManifold>(CollidableReference characterCollidable, CollidableReference otherCollidable,
        CollidablePair pair, ref TManifold manifold) where TManifold : struct, IContactManifold<TManifold>
    {
        if (characterCollidable.Mobility != CollidableMobility.Dynamic ||
            characterCollidable.BodyHandle.Value >= bodyHandleToCharacterIndex.Length)
            return false;

        var characterIndex = bodyHandleToCharacterIndex[characterCollidable.BodyHandle.Value];
        if (characterIndex < 0)
            return false;

        ref var character = ref characters[characterIndex];

        // Get the character's current up direction in world space
        ref var bodyLocation = ref Simulation.Bodies.HandleToLocation[character.BodyHandle.Value];
        ref var set = ref Simulation.Bodies.Sets[bodyLocation.SetIndex];
        ref var pose = ref set.DynamicsState[bodyLocation.Index].Motion.Pose;
        QuaternionEx.Transform(character.LocalUp, pose.Orientation, out var up);

        // Process the manifold to find the best support candidate
        // For convex manifolds, all contacts share the same normal
        if (manifold.Convex)
        {
            ref var convexManifold = ref Unsafe.As<TManifold, ConvexContactManifold>(ref manifold);
            var normalUpDot = Vector3.Dot(convexManifold.Normal, up);

            // The narrow phase generates normals pointing from B to A.
            // If the character is collidable B, we need to negate the comparison.
            if ((pair.B.Packed == characterCollidable.Packed ? -normalUpDot : normalUpDot) > character.CosMaximumSlope)
            {
                // This manifold has a valid support angle. Find the deepest contact.
                var maxDepth = convexManifold.Contact0.Depth;
                var maxDepthIndex = 0;
                for (int i = 1; i < convexManifold.Count; ++i)
                {
                    ref var depth = ref Unsafe.Add(ref convexManifold.Contact0, i).Depth;
                    if (depth > maxDepth)
                    {
                        maxDepth = depth;
                        maxDepthIndex = i;
                    }
                }

                // Check if this contact qualifies as a support based on depth
                if (maxDepth >= character.MinimumSupportDepth ||
                    (character.Supported && maxDepth > character.MinimumSupportContinuationDepth))
                {
                    ref var candidate = ref supportCandidates[characterIndex];
                    if (candidate.Depth < maxDepth)
                    {
                        candidate.Depth = maxDepth;
                        ref var contact = ref Unsafe.Add(ref convexManifold.Contact0, maxDepthIndex);
                        var offsetFromB = contact.Offset - convexManifold.OffsetB;

                        if (pair.B.Packed == characterCollidable.Packed)
                        {
                            candidate.Normal = -convexManifold.Normal;
                            candidate.OffsetFromCharacter = offsetFromB;
                            candidate.OffsetFromSupport = contact.Offset;
                        }
                        else
                        {
                            candidate.Normal = convexManifold.Normal;
                            candidate.OffsetFromCharacter = contact.Offset;
                            candidate.OffsetFromSupport = offsetFromB;
                        }
                        candidate.Support = otherCollidable;
                    }
                }
            }
        }
        else
        {
            // Nonconvex manifolds can have per-contact normals
            ref var nonconvexManifold = ref Unsafe.As<TManifold, NonconvexContactManifold>(ref manifold);
            var maxDepth = float.MinValue;
            var maxDepthIndex = -1;

            for (int i = 0; i < nonconvexManifold.Count; ++i)
            {
                ref var contact = ref Unsafe.Add(ref nonconvexManifold.Contact0, i);
                if (contact.Depth > maxDepth)
                {
                    var upDot = Vector3.Dot(contact.Normal, up);
                    if ((pair.B.Packed == characterCollidable.Packed ? -upDot : upDot) > character.CosMaximumSlope)
                    {
                        maxDepth = contact.Depth;
                        maxDepthIndex = i;
                    }
                }
            }

            if (maxDepthIndex >= 0 && (maxDepth >= character.MinimumSupportDepth ||
                (character.Supported && maxDepth > character.MinimumSupportContinuationDepth)))
            {
                ref var candidate = ref supportCandidates[characterIndex];
                if (candidate.Depth < maxDepth)
                {
                    ref var contact = ref Unsafe.Add(ref nonconvexManifold.Contact0, maxDepthIndex);
                    candidate.Depth = maxDepth;
                    var offsetFromB = contact.Offset - nonconvexManifold.OffsetB;

                    if (pair.B.Packed == characterCollidable.Packed)
                    {
                        candidate.Normal = -contact.Normal;
                        candidate.OffsetFromCharacter = offsetFromB;
                        candidate.OffsetFromSupport = contact.Offset;
                    }
                    else
                    {
                        candidate.Normal = contact.Normal;
                        candidate.OffsetFromCharacter = contact.Offset;
                        candidate.OffsetFromSupport = offsetFromB;
                    }
                    candidate.Support = otherCollidable;
                }
            }
        }

        return true;
    }

    /// <summary>
    /// Main entry point for the narrow phase to report contacts.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryReportContacts<TManifold>(in CollidablePair pair, ref TManifold manifold,
        ref PairMaterialProperties materialProperties) where TManifold : struct, IContactManifold<TManifold>
    {
        if (manifold.Count == 0)
            return false;

        var aIsCharacter = TryReportContact(pair.A, pair.B, pair, ref manifold);
        var bIsCharacter = TryReportContact(pair.B, pair.A, pair, ref manifold);

        if (aIsCharacter || bIsCharacter)
        {
            // Zero friction - the character's motion is controlled by our velocity modification, not contact friction
            materialProperties.FrictionCoefficient = 0;
            return true;
        }
        return false;
    }

    /// <summary>
    /// Applies character motion by directly modifying velocities.
    /// This is the simplified equivalent of what the CharacterMotionConstraint does in the solver.
    /// </summary>
    /// <remarks>
    /// <para>
    /// The full constraint-based approach solves motion iteratively during the solver phase.
    /// Each iteration considers the current velocity state and applies corrective impulses to move
    /// toward the goal. Multiple iterations allow the solver to find a balanced solution when
    /// multiple forces/constraints are competing.
    /// </para>
    /// <para>
    /// This simplified approach just does the equivalent of one iteration, directly on the velocity.
    /// The math is essentially the same, but:
    /// 1. We're not using jacobians explicitly (they're baked into the projection)
    /// 2. We're not using effective mass (we're directly computing acceleration from force/mass)
    /// 3. We're not iterating (one pass only)
    /// </para>
    /// <para>
    /// The result is less robust when facing resistance, but much easier to understand.
    /// </para>
    /// </remarks>
    void ApplyCharacterMotion(float dt, IThreadDispatcher threadDispatcher)
    {
        // Process each character
        for (int i = 0; i < characters.Count; ++i)
        {
            ref var character = ref characters[i];
            ref var candidate = ref supportCandidates[i];

            ref var bodyLocation = ref Simulation.Bodies.HandleToLocation[character.BodyHandle.Value];

            // Skip inactive bodies
            if (bodyLocation.SetIndex != 0)
            {
                character.TryJump = false;
                continue;
            }

            ref var activeSet = ref Simulation.Bodies.ActiveSet;
            ref var velocity = ref activeSet.DynamicsState[bodyLocation.Index].Motion.Velocity;
            ref var pose = ref activeSet.DynamicsState[bodyLocation.Index].Motion.Pose;
            var inverseMass = activeSet.DynamicsState[bodyLocation.Index].Inertia.Local.InverseMass;

            QuaternionEx.Transform(character.LocalUp, pose.Orientation, out var up);

            bool hasValidSupport = candidate.Depth > float.MinValue;

            // Handle jumping
            if (hasValidSupport && character.TryJump)
            {
                // Compute jump velocity change
                var characterUpVelocity = Vector3.Dot(velocity.Linear, up);

                // If supported by a dynamic body, consider its velocity
                if (candidate.Support.Mobility == CollidableMobility.Dynamic)
                {
                    ref var supportLocation = ref Simulation.Bodies.HandleToLocation[candidate.Support.BodyHandle.Value];
                    if (supportLocation.SetIndex == 0)
                    {
                        ref var supportVelocity = ref activeSet.DynamicsState[supportLocation.Index].Motion.Velocity;
                        var wxr = Vector3.Cross(supportVelocity.Angular, candidate.OffsetFromSupport);
                        var jumpSupportContactVelocity = supportVelocity.Linear + wxr;
                        var supportUpVelocity = Vector3.Dot(jumpSupportContactVelocity, up);

                        var jumpChange = up * MathF.Max(0, character.JumpVelocity - (characterUpVelocity - supportUpVelocity));
                        velocity.Linear += jumpChange;

                        // Apply equal and opposite impulse to support
                        var impulse = -jumpChange / inverseMass;
                        BodyReference.ApplyImpulse(activeSet, supportLocation.Index, impulse, candidate.OffsetFromSupport);
                    }
                }
                else
                {
                    // Static or kinematic support - just apply jump velocity
                    velocity.Linear += up * MathF.Max(0, character.JumpVelocity - characterUpVelocity);
                }

                character.Supported = false;
                character.TryJump = false;
                continue;
            }

            character.TryJump = false;

            if (!hasValidSupport)
            {
                character.Supported = false;
                continue;
            }

            // We have a valid support - apply motion control
            character.Supported = true;
            character.Support = candidate.Support;

            // Build the surface basis for motion
            // Y axis = surface normal (support direction)
            // Z axis = backward direction (opposite of view direction, projected onto surface)
            // X axis = right direction (perpendicular to forward and up)
            Matrix3x3 surfaceBasis;
            surfaceBasis.Y = candidate.Normal;

            // Project view direction onto the surface plane
            var rayDistance = Vector3.Dot(character.ViewDirection, surfaceBasis.Y);
            var rayVelocity = Vector3.Dot(up, surfaceBasis.Y);
            surfaceBasis.Z = up * (rayDistance / rayVelocity) - character.ViewDirection;
            var zLengthSquared = surfaceBasis.Z.LengthSquared();
            if (zLengthSquared > 1e-12f)
            {
                surfaceBasis.Z /= MathF.Sqrt(zLengthSquared);
            }
            else
            {
                // View is nearly aligned with surface normal; use a default forward
                QuaternionEx.GetQuaternionBetweenNormalizedVectors(Vector3.UnitY, surfaceBasis.Y, out var rotation);
                QuaternionEx.TransformUnitZ(rotation, out surfaceBasis.Z);
            }
            surfaceBasis.X = Vector3.Cross(surfaceBasis.Y, surfaceBasis.Z);

            // Get the support's velocity at the contact point (if it's a body)
            Vector3 supportContactVelocity = Vector3.Zero;
            float supportInverseMass = 0;
            if (candidate.Support.Mobility != CollidableMobility.Static)
            {
                ref var supportLocation = ref Simulation.Bodies.HandleToLocation[candidate.Support.BodyHandle.Value];
                if (supportLocation.SetIndex == 0)
                {
                    ref var supportVelocity = ref activeSet.DynamicsState[supportLocation.Index].Motion.Velocity;
                    var wxr = Vector3.Cross(supportVelocity.Angular, candidate.OffsetFromSupport);
                    supportContactVelocity = supportVelocity.Linear + wxr;
                    if (candidate.Support.Mobility == CollidableMobility.Dynamic)
                    {
                        supportInverseMass = activeSet.DynamicsState[supportLocation.Index].Inertia.Local.InverseMass;
                    }
                }
            }

            // Compute the character's velocity at the contact point
            var wxrCharacter = Vector3.Cross(velocity.Angular, candidate.OffsetFromCharacter);
            var characterContactVelocity = velocity.Linear + wxrCharacter;

            // Relative velocity of the character with respect to the support
            var relativeVelocity = characterContactVelocity - supportContactVelocity;

            // === HORIZONTAL MOTION ===
            // Project the relative velocity onto the horizontal plane (X and Z axes of surface basis)
            var currentHorizontalX = Vector3.Dot(relativeVelocity, surfaceBasis.X);
            var currentHorizontalZ = Vector3.Dot(relativeVelocity, surfaceBasis.Z);

            // The target velocity is specified as (strafe, forward) = (X, -Z) in the surface basis
            // Note: Z points backward, so forward motion is in the -Z direction
            var targetHorizontalX = character.TargetVelocity.X;
            var targetHorizontalZ = -character.TargetVelocity.Y;  // Negate because +Y target = -Z direction

            // Compute the velocity error
            var errorX = targetHorizontalX - currentHorizontalX;
            var errorZ = targetHorizontalZ - currentHorizontalZ;

            // Compute the acceleration needed to reach the target velocity in one timestep
            // But clamp it to the maximum force the character can apply
            var desiredAccelX = errorX / dt;
            var desiredAccelZ = errorZ / dt;

            // Total inverse mass felt by the horizontal constraint (character + support if dynamic)
            var horizontalEffectiveInverseMass = inverseMass + supportInverseMass;

            // Convert to force and clamp by maximum horizontal force
            var desiredForceX = desiredAccelX / horizontalEffectiveInverseMass;
            var desiredForceZ = desiredAccelZ / horizontalEffectiveInverseMass;
            var forceMagnitude = MathF.Sqrt(desiredForceX * desiredForceX + desiredForceZ * desiredForceZ);

            if (forceMagnitude > character.MaximumHorizontalForce)
            {
                var scale = character.MaximumHorizontalForce / forceMagnitude;
                desiredForceX *= scale;
                desiredForceZ *= scale;
            }

            // Convert back to impulse and apply
            var impulseX = desiredForceX * dt;
            var impulseZ = desiredForceZ * dt;
            var horizontalImpulse = surfaceBasis.X * impulseX + surfaceBasis.Z * impulseZ;

            velocity.Linear += horizontalImpulse * inverseMass;

            // Apply opposite impulse to support if it's dynamic
            if (candidate.Support.Mobility == CollidableMobility.Dynamic)
            {
                ref var supportLocation = ref Simulation.Bodies.HandleToLocation[candidate.Support.BodyHandle.Value];
                if (supportLocation.SetIndex == 0)
                {
                    BodyReference.ApplyImpulse(activeSet, supportLocation.Index, -horizontalImpulse, candidate.OffsetFromSupport);
                }
            }

            // === VERTICAL MOTION ===
            // The vertical constraint tries to prevent the character from separating from the support.
            // Unlike horizontal motion, it only pulls (never pushes).

            // Current separation velocity along the surface normal
            var separationVelocity = Vector3.Dot(relativeVelocity, candidate.Normal);

            // Allow some separation velocity if deeply penetrating (to resolve penetration)
            var allowedSeparationVelocity = MathF.Max(0, candidate.Depth / dt);

            // Only apply vertical force if we're separating faster than allowed
            if (separationVelocity > allowedSeparationVelocity)
            {
                var verticalError = allowedSeparationVelocity - separationVelocity;
                var desiredVerticalAccel = verticalError / dt;
                var verticalEffectiveInverseMass = inverseMass + supportInverseMass;
                var desiredVerticalForce = desiredVerticalAccel / verticalEffectiveInverseMass;

                // Clamp to maximum vertical force
                desiredVerticalForce = MathF.Max(desiredVerticalForce, -character.MaximumVerticalForce);

                var verticalImpulse = candidate.Normal * (desiredVerticalForce * dt);
                velocity.Linear += verticalImpulse * inverseMass;

                // Apply opposite impulse to support if it's dynamic
                if (candidate.Support.Mobility == CollidableMobility.Dynamic)
                {
                    ref var supportLocation = ref Simulation.Bodies.HandleToLocation[candidate.Support.BodyHandle.Value];
                    if (supportLocation.SetIndex == 0)
                    {
                        BodyReference.ApplyImpulse(activeSet, supportLocation.Index, -verticalImpulse, candidate.OffsetFromSupport);
                    }
                }
            }
        }
    }

    bool disposed;
    public void Dispose()
    {
        if (!disposed)
        {
            disposed = true;
            Simulation.Timestepper.BeforeCollisionDetection -= PrepareForContacts;
            Simulation.Timestepper.CollisionsDetected -= ApplyCharacterMotion;
            characters.Dispose(pool);
            pool.Return(ref bodyHandleToCharacterIndex);
            pool.Return(ref supportCandidates);
        }
    }
}

/// <summary>
/// Simplified narrow phase callbacks for the simple character controller.
/// </summary>
struct SimpleCharacterNarrowphaseCallbacks : INarrowPhaseCallbacks
{
    public SimpleCharacterControllers Characters;

    public SimpleCharacterNarrowphaseCallbacks(SimpleCharacterControllers characters)
    {
        Characters = characters;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
    {
        return a.Mobility == CollidableMobility.Dynamic || b.Mobility == CollidableMobility.Dynamic;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
    {
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold,
        out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
    {
        pairMaterial = new PairMaterialProperties
        {
            FrictionCoefficient = 1,
            MaximumRecoveryVelocity = 2,
            SpringSettings = new SpringSettings(30, 1)
        };
        Characters.TryReportContacts(pair, ref manifold, ref pairMaterial);
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB,
        ref ConvexContactManifold manifold)
    {
        return true;
    }

    public void Dispose()
    {
        Characters.Dispose();
    }

    public void Initialize(Simulation simulation)
    {
        Characters.Initialize(simulation);
    }
}

/// <summary>
/// Convenience wrapper for a simple character, similar to CharacterInput but for SimpleCharacterControllers.
/// </summary>
public struct SimpleCharacterInput
{
    BodyHandle bodyHandle;
    SimpleCharacterControllers characters;
    float speed;
    Capsule shape;

    public BodyHandle BodyHandle => bodyHandle;

    public SimpleCharacterInput(SimpleCharacterControllers characters, Vector3 initialPosition, Capsule shape,
        float minimumSpeculativeMargin, float mass, float maximumHorizontalForce, float maximumVerticalGlueForce,
        float jumpVelocity, float speed, float maximumSlope = MathF.PI * 0.25f)
    {
        this.characters = characters;
        var shapeIndex = characters.Simulation.Shapes.Add(shape);

        // Create the character body with infinite inertia tensor (no rotation)
        bodyHandle = characters.Simulation.Bodies.Add(
            BodyDescription.CreateDynamic(initialPosition, new BodyInertia { InverseMass = 1f / mass },
            new(shapeIndex, minimumSpeculativeMargin, float.MaxValue, ContinuousDetection.Passive), shape.Radius * 0.02f));

        ref var character = ref characters.AllocateCharacter(bodyHandle);
        character.LocalUp = new Vector3(0, 1, 0);
        character.CosMaximumSlope = MathF.Cos(maximumSlope);
        character.JumpVelocity = jumpVelocity;
        character.MaximumVerticalForce = maximumVerticalGlueForce;
        character.MaximumHorizontalForce = maximumHorizontalForce;
        character.MinimumSupportDepth = shape.Radius * -0.01f;
        character.MinimumSupportContinuationDepth = -minimumSpeculativeMargin;
        this.speed = speed;
        this.shape = shape;
    }

    static Key MoveForward = Key.W;
    static Key MoveBackward = Key.S;
    static Key MoveRight = Key.D;
    static Key MoveLeft = Key.A;
    static Key Sprint = Key.LShift;
    static Key Jump = Key.Space;
    static Key JumpAlternate = Key.BackSpace;

    public void UpdateCharacterGoals(Input input, Camera camera, float simulationTimestepDuration)
    {
        Vector2 movementDirection = default;
        if (input.IsDown(MoveForward))
            movementDirection = new Vector2(0, 1);
        if (input.IsDown(MoveBackward))
            movementDirection += new Vector2(0, -1);
        if (input.IsDown(MoveLeft))
            movementDirection += new Vector2(-1, 0);
        if (input.IsDown(MoveRight))
            movementDirection += new Vector2(1, 0);

        var movementDirectionLengthSquared = movementDirection.LengthSquared();
        if (movementDirectionLengthSquared > 0)
            movementDirection /= MathF.Sqrt(movementDirectionLengthSquared);

        ref var character = ref characters.GetCharacterByBodyHandle(bodyHandle);
        character.TryJump = input.WasPushed(Jump) || input.WasPushed(JumpAlternate);
        var characterBody = new BodyReference(bodyHandle, characters.Simulation.Bodies);
        var effectiveSpeed = input.IsDown(Sprint) ? speed * 1.75f : speed;
        var newTargetVelocity = movementDirection * effectiveSpeed;
        var viewDirection = camera.Forward;

        // Wake the character if goals have changed
        if (!characterBody.Awake &&
            ((character.TryJump && character.Supported) ||
            newTargetVelocity != character.TargetVelocity ||
            (newTargetVelocity != Vector2.Zero && character.ViewDirection != viewDirection)))
        {
            characters.Simulation.Awakener.AwakenBody(character.BodyHandle);
        }

        character.TargetVelocity = newTargetVelocity;
        character.ViewDirection = viewDirection;

        // Air control (same as the full implementation)
        if (!character.Supported && movementDirectionLengthSquared > 0)
        {
            QuaternionEx.Transform(character.LocalUp, characterBody.Pose.Orientation, out var characterUp);
            var characterRight = Vector3.Cross(character.ViewDirection, characterUp);
            var rightLengthSquared = characterRight.LengthSquared();
            if (rightLengthSquared > 1e-10f)
            {
                characterRight /= MathF.Sqrt(rightLengthSquared);
                var characterForward = Vector3.Cross(characterUp, characterRight);
                var worldMovementDirection = characterRight * movementDirection.X + characterForward * movementDirection.Y;
                var currentVelocity = Vector3.Dot(characterBody.Velocity.Linear, worldMovementDirection);

                const float airControlForceScale = .2f;
                const float airControlSpeedScale = .2f;
                var airAccelerationDt = characterBody.LocalInertia.InverseMass * character.MaximumHorizontalForce * airControlForceScale * simulationTimestepDuration;
                var maximumAirSpeed = effectiveSpeed * airControlSpeedScale;
                var targetVelocity = MathF.Min(currentVelocity + airAccelerationDt, maximumAirSpeed);
                var velocityChangeAlongMovementDirection = MathF.Max(0, targetVelocity - currentVelocity);
                characterBody.Velocity.Linear += worldMovementDirection * velocityChangeAlongMovementDirection;
            }
        }
    }

    public void UpdateCameraPosition(Camera camera, float cameraBackwardOffsetScale = 4)
    {
        ref var character = ref characters.GetCharacterByBodyHandle(bodyHandle);
        var characterBody = new BodyReference(bodyHandle, characters.Simulation.Bodies);
        camera.Position = characterBody.Pose.Position + new Vector3(0, shape.HalfLength, 0) +
            camera.Up * (shape.Radius * 1.2f) -
            camera.Forward * (shape.HalfLength + shape.Radius) * cameraBackwardOffsetScale;
    }

    void RenderControl(ref Vector2 position, float textHeight, string controlName, string controlValue,
        TextBuilder text, TextBatcher textBatcher, Font font)
    {
        text.Clear().Append(controlName).Append(": ").Append(controlValue);
        textBatcher.Write(text, position, textHeight, new Vector3(1), font);
        position.Y += textHeight * 1.1f;
    }

    public void RenderControls(Vector2 position, float textHeight, TextBatcher textBatcher, TextBuilder text, Font font)
    {
        RenderControl(ref position, textHeight, nameof(MoveForward), ControlStrings.GetName(MoveForward), text, textBatcher, font);
        RenderControl(ref position, textHeight, nameof(MoveBackward), ControlStrings.GetName(MoveBackward), text, textBatcher, font);
        RenderControl(ref position, textHeight, nameof(MoveRight), ControlStrings.GetName(MoveRight), text, textBatcher, font);
        RenderControl(ref position, textHeight, nameof(MoveLeft), ControlStrings.GetName(MoveLeft), text, textBatcher, font);
        RenderControl(ref position, textHeight, nameof(Sprint), ControlStrings.GetName(Sprint), text, textBatcher, font);
        RenderControl(ref position, textHeight, nameof(Jump), ControlStrings.GetName(Jump), text, textBatcher, font);
    }

    public void Dispose()
    {
        characters.Simulation.Shapes.Remove(new BodyReference(bodyHandle, characters.Simulation.Bodies).Collidable.Shape);
        characters.Simulation.Bodies.Remove(bodyHandle);
        characters.RemoveCharacterByBodyHandle(bodyHandle);
    }
}

/// <summary>
/// Demonstrates a simplified character controller that uses direct velocity modification
/// instead of the full constraint-based approach.
/// </summary>
/// <remarks>
/// <para>
/// This demo is functionally similar to CharacterDemo, but uses <see cref="SimpleCharacterControllers"/> instead
/// of the full <see cref="CharacterControllers"/>. The simplified approach is easier to understand because:
/// </para>
/// <para>
/// 1. No custom constraint types - motion is achieved through direct velocity changes
/// 2. No SIMD/AoSoA complexity - all math uses simple scalar/Vector3 types
/// 3. No multithreading concerns - everything runs single-threaded
/// </para>
/// <para>
/// The trade-off is robustness: this simple approach doesn't handle resistance as well.
/// If the character is pushed by a heavy object, the constraint-based approach would find
/// a balanced solution, while this approach just applies the velocity change without
/// considering the opposing forces.
/// </para>
/// </remarks>
public class SimpleCharacterDemo : Demo
{
    SimpleCharacterControllers characters;

    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Position = new Vector3(20, 10, 20);
        camera.Yaw = MathF.PI;
        camera.Pitch = 0;

        characters = new SimpleCharacterControllers(BufferPool);
        Simulation = Simulation.Create(BufferPool,
            new SimpleCharacterNarrowphaseCallbacks(characters),
            new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)),
            new SolveDescription(8, 1));

        CreateCharacter(new Vector3(0, 2, -4));

        // Create a bunch of small boxes to walk on
        var random = new Random(5);
        var origin = new Vector3(-3f, 0.5f, 0);
        var spacing = new Vector3(0.5f, 0, -0.5f);
        for (int i = 0; i < 12; ++i)
        {
            for (int j = 0; j < 12; ++j)
            {
                var position = origin + new Vector3(i, 0, j) * spacing;
                var orientation = QuaternionEx.CreateFromAxisAngle(
                    Vector3.Normalize(new Vector3(0.0001f) + new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle())),
                    10 * random.NextSingle());
                var shape = new Box(0.1f + 0.3f * random.NextSingle(), 0.1f + 0.3f * random.NextSingle(), 0.1f + 0.3f * random.NextSingle());
                var shapeIndex = Simulation.Shapes.Add(shape);
                var choice = (i + j) % 3;
                switch (choice)
                {
                    case 0:
                        Simulation.Bodies.Add(BodyDescription.CreateDynamic((position, orientation), shape.ComputeInertia(1), shapeIndex, 0.01f));
                        break;
                    case 1:
                        Simulation.Bodies.Add(BodyDescription.CreateKinematic((position, orientation), shapeIndex, 0.01f));
                        break;
                    case 2:
                        Simulation.Statics.Add(new StaticDescription(position, orientation, shapeIndex));
                        break;
                }
            }
        }

        // Add spinning fans
        var bladeDescription = BodyDescription.CreateConvexDynamic(new Vector3(), 3, Simulation.Shapes, new Box(10, 0.2f, 2));
        var bladeBaseDescription = BodyDescription.CreateConvexKinematic(new Vector3(), Simulation.Shapes, new Box(0.2f, 1, 0.2f));
        for (int i = 0; i < 3; ++i)
        {
            bladeBaseDescription.Pose.Position = new Vector3(-22, 1, i * 11);
            bladeDescription.Pose.Position = new Vector3(-22, 1.7f, i * 11);
            var baseHandle = Simulation.Bodies.Add(bladeBaseDescription);
            var bladeHandle = Simulation.Bodies.Add(bladeDescription);
            Simulation.Solver.Add(baseHandle, bladeHandle,
                new Hinge
                {
                    LocalHingeAxisA = Vector3.UnitY,
                    LocalHingeAxisB = Vector3.UnitY,
                    LocalOffsetA = new Vector3(0, 0.7f, 0),
                    LocalOffsetB = new Vector3(0, 0, 0),
                    SpringSettings = new SpringSettings(30, 1)
                });
            Simulation.Solver.Add(baseHandle, bladeHandle,
                new AngularAxisMotor
                {
                    LocalAxisA = Vector3.UnitY,
                    TargetVelocity = (i + 1) * (i + 1) * (i + 1) * (i + 1) * 0.2f,
                    Settings = new MotorSettings(5 * (i + 1), 0.0001f)
                });
        }

        // Include a giant newt
        var newtMesh = DemoMeshHelper.LoadModel(content, BufferPool, @"Content\newt.obj", new Vector3(15, 15, 15));
        Simulation.Statics.Add(new StaticDescription(new Vector3(0, 0.5f, 0), Simulation.Shapes.Add(newtMesh)));

        // Newt tongue
        var tongueBase = Simulation.Bodies.Add(BodyDescription.CreateKinematic(new Vector3(0, 8.4f, 24), default, default));
        var tongue = Simulation.Bodies.Add(BodyDescription.CreateConvexDynamic(new Vector3(0, 8.4f, 27.5f), 1, Simulation.Shapes, new Box(1, 0.1f, 6f)));
        Simulation.Solver.Add(tongueBase, tongue, new Hinge
        {
            LocalHingeAxisA = Vector3.UnitX,
            LocalHingeAxisB = Vector3.UnitX,
            LocalOffsetB = new Vector3(0, 0, -3f),
            SpringSettings = new SpringSettings(30, 1)
        });
        Simulation.Solver.Add(tongueBase, tongue, new AngularServo
        {
            TargetRelativeRotationLocalA = Quaternion.Identity,
            ServoSettings = ServoSettings.Default,
            SpringSettings = new SpringSettings(2, 0)
        });

        // Seesaw
        var seesawBase = Simulation.Bodies.Add(BodyDescription.CreateKinematic(new Vector3(0, 1f, 34f),
            Simulation.Shapes.Add(new Box(0.2f, 1, 0.2f)), 0.01f));
        var seesaw = Simulation.Bodies.Add(BodyDescription.CreateConvexDynamic(new Vector3(0, 1.7f, 34f), 1, Simulation.Shapes, new Box(1, 0.1f, 6f)));
        Simulation.Solver.Add(seesawBase, seesaw, new Hinge
        {
            LocalHingeAxisA = Vector3.UnitX,
            LocalHingeAxisB = Vector3.UnitX,
            LocalOffsetA = new Vector3(0, 0.7f, 0),
            LocalOffsetB = new Vector3(0, 0, 0),
            SpringSettings = new SpringSettings(30, 1)
        });

        Simulation.Bodies.Add(BodyDescription.CreateConvexDynamic(new Vector3(0, 2.25f, 35.5f), 0.5f, Simulation.Shapes, new Box(1f, 1f, 1f)));

        // Moving platforms
        movingPlatforms = new MovingPlatform[16];
        Func<double, RigidPose> poseCreator = time =>
        {
            RigidPose pose;
            var horizontalScale = (float)(45 + 10 * Math.Sin(time * 0.015));
            pose.Position = new Vector3(0.7f * horizontalScale * (float)Math.Sin(time * 0.1),
                10 + 4 * (float)Math.Sin((time + Math.PI * 0.5f) * 0.25),
                horizontalScale * (float)Math.Cos(time * 0.1));
            var x = MathF.Max(0f, MathF.Min(1f, 1f - (pose.Position.Z + 20f) / -20f));
            var smoothStepped = 3 * x * x - 2 * x * x * x;
            pose.Position.Y = smoothStepped * (pose.Position.Y - 0.025f) + 0.025f;
            pose.Orientation = Quaternion.Identity;
            return pose;
        };
        var platformShapeIndex = Simulation.Shapes.Add(new Box(5, 1, 5));
        for (int i = 0; i < movingPlatforms.Length; ++i)
        {
            movingPlatforms[i] = new MovingPlatform(platformShapeIndex, i * 3559, 1f / 60f, Simulation, poseCreator);
        }

        // Static platform grid
        var box = new Box(4, 1, 4);
        var boxShapeIndex = Simulation.Shapes.Add(box);
        const int width = 8;
        for (int i = 0; i < width; ++i)
        {
            for (int j = 0; j < width; ++j)
            {
                Simulation.Statics.Add(new StaticDescription(new Vector3(box.Width, 0, box.Length) * new Vector3(i, 0, j) + new Vector3(32f, 1, 0), boxShapeIndex));
            }
        }

        // Ground plane
        Simulation.Statics.Add(new StaticDescription(new Vector3(0, 0, 0), Simulation.Shapes.Add(new Box(200, 1, 200))));
    }

    struct MovingPlatform
    {
        public BodyHandle BodyHandle;
        public float InverseGoalSatisfactionTime;
        public double TimeOffset;
        public Func<double, RigidPose> PoseCreator;

        public MovingPlatform(CollidableDescription collidable, double timeOffset, float goalSatisfactionTime,
            Simulation simulation, Func<double, RigidPose> poseCreator)
        {
            PoseCreator = poseCreator;
            BodyHandle = simulation.Bodies.Add(BodyDescription.CreateKinematic(poseCreator(timeOffset), collidable, -1));
            InverseGoalSatisfactionTime = 1f / goalSatisfactionTime;
            TimeOffset = timeOffset;
        }

        public void Update(Simulation simulation, double time)
        {
            var body = simulation.Bodies[BodyHandle];
            ref var pose = ref body.Pose;
            ref var velocity = ref body.Velocity;
            var targetPose = PoseCreator(time + TimeOffset);
            velocity.Linear = (targetPose.Position - pose.Position) * InverseGoalSatisfactionTime;
            QuaternionEx.GetRelativeRotationWithoutOverlap(pose.Orientation, targetPose.Orientation, out var rotation);
            QuaternionEx.GetAxisAngleFromQuaternion(rotation, out var axis, out var angle);
            velocity.Angular = axis * (angle * InverseGoalSatisfactionTime);
        }
    }
    MovingPlatform[] movingPlatforms;

    bool characterActive;
    SimpleCharacterInput character;
    double time;

    void CreateCharacter(Vector3 position)
    {
        characterActive = true;
        character = new SimpleCharacterInput(characters, position, new Capsule(0.5f, 1), 0.1f, 1, 20, 100, 6, 4, MathF.PI * 0.4f);
    }

    public override void Update(Window window, Camera camera, Input input, float dt)
    {
        if (input.WasPushed(Key.C))
        {
            if (characterActive)
            {
                character.Dispose();
                characterActive = false;
            }
            else
            {
                CreateCharacter(camera.Position);
            }
        }

        if (characterActive)
        {
            character.UpdateCharacterGoals(input, camera, TimestepDuration);
        }

        time += TimestepDuration;
        for (int i = 0; i < movingPlatforms.Length; ++i)
        {
            movingPlatforms[i].Update(Simulation, time);
        }

        base.Update(window, camera, input, dt);
    }

    public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
    {
        float textHeight = 16;
        var position = new Vector2(32, renderer.Surface.Resolution.Y - textHeight * 10);
        renderer.TextBatcher.Write(text.Clear().Append("Simple Character Demo - Direct Velocity Modification"), position, textHeight, new Vector3(1, 1, 0), font);
        position.Y += textHeight * 1.2f;
        renderer.TextBatcher.Write(text.Clear().Append("Toggle character: C"), position, textHeight, new Vector3(1), font);
        position.Y += textHeight * 1.2f;
        character.RenderControls(position, textHeight, renderer.TextBatcher, text, font);

        if (characterActive)
        {
            character.UpdateCameraPosition(camera);
        }

        base.Render(renderer, camera, input, text, font);
    }
}
