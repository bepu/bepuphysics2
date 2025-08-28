﻿using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuPhysics.Trees;
using BepuUtilities;
using BepuUtilities.Memory;
using DemoRenderer;
using DemoRenderer.Constraints;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace Demos;

struct Grabber
{
    bool active;
    BodyReference body;
    float t;
    Vector3 localGrabPoint;
    Quaternion targetOrientation;
    ConstraintHandle linearMotorHandle;
    ConstraintHandle angularMotorHandle;

    struct RayHitHandler : IRayHitHandler
    {
        public float T;
        public CollidableReference HitCollidable;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowTest(CollidableReference collidable)
        {
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowTest(CollidableReference collidable, int childIndex)
        {
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void OnRayHit(in RayData ray, ref float maximumT, float t, Vector3 normal, CollidableReference collidable, int childIndex)
        {
            //We are only interested in the earliest hit. This callback is executing within the traversal, so modifying maximumT informs the traversal
            //that it can skip any AABBs which are more distant than the new maximumT.
            maximumT = t;
            //Cache the earliest impact.
            T = t;
            HitCollidable = collidable;
        }
    }

    readonly void CreateMotorDescription(Vector3 target, float inverseMass, out OneBodyLinearServo linearDescription, out OneBodyAngularServo angularDescription)
    {
        linearDescription = new OneBodyLinearServo
        {
            LocalOffset = localGrabPoint,
            Target = target,
            ServoSettings = new ServoSettings(float.MaxValue, 0, 360 / inverseMass),
            SpringSettings = new SpringSettings(5, 2),
        };
        angularDescription = new OneBodyAngularServo
        {
            TargetOrientation = targetOrientation,
            ServoSettings = new ServoSettings(float.MaxValue, 0, localGrabPoint.Length() * 180 / inverseMass),
            SpringSettings = new SpringSettings(5, 2),
        };
    }

    public void Update(Simulation simulation, Camera camera, bool mouseLocked, bool shouldGrab, Quaternion rotation, in Vector2 normalizedMousePosition, BufferPool pool)
    {
        //On the off chance some demo modifies the kinematic state, treat that as a grab terminator.
        var bodyExists = body.Exists && !body.Kinematic;
        if (active && (!shouldGrab || !bodyExists))
        {
            active = false;
            if (bodyExists)
            {
                //If the body wasn't removed, then the constraint should be removed.
                //(Body removal forces connected constraints to removed, so in that case we wouldn't have to worry about it.)
                simulation.Solver.Remove(linearMotorHandle);
                if (!Bodies.HasLockedInertia(body.LocalInertia.InverseInertiaTensor))
                    simulation.Solver.Remove(angularMotorHandle);
            }
            body = new BodyReference();
        }
        else if (shouldGrab && !active)
        {
            var rayDirection = camera.GetRayDirection(mouseLocked, normalizedMousePosition);
            var hitHandler = default(RayHitHandler);
            hitHandler.T = float.MaxValue;
            simulation.RayCast(camera.Position, rayDirection, float.MaxValue, pool, ref hitHandler);
            if (hitHandler.T < float.MaxValue && hitHandler.HitCollidable.Mobility == CollidableMobility.Dynamic)
            {
                //Found something to grab!
                t = hitHandler.T;
                body = simulation.Bodies[hitHandler.HitCollidable.BodyHandle];
                var hitLocation = camera.Position + rayDirection * t;
                RigidPose.TransformByInverse(hitLocation, body.Pose, out localGrabPoint);
                targetOrientation = body.Pose.Orientation;
                active = true;
                CreateMotorDescription(hitLocation, body.LocalInertia.InverseMass, out var linearDescription, out var angularDescription);
                linearMotorHandle = simulation.Solver.Add(body.Handle, linearDescription);
                if (!Bodies.HasLockedInertia(body.LocalInertia.InverseInertiaTensor))
                    angularMotorHandle = simulation.Solver.Add(body.Handle, angularDescription);
            }
        }
        else if (active)
        {
            var rayDirection = camera.GetRayDirection(mouseLocked, normalizedMousePosition);
            var targetPoint = camera.Position + rayDirection * t;
            targetOrientation = QuaternionEx.Normalize(QuaternionEx.Concatenate(targetOrientation, rotation));

            CreateMotorDescription(targetPoint, body.LocalInertia.InverseMass, out var linearDescription, out var angularDescription);
            simulation.Solver.ApplyDescription(linearMotorHandle, linearDescription);
            if (!Bodies.HasLockedInertia(body.LocalInertia.InverseInertiaTensor))
                simulation.Solver.ApplyDescription(angularMotorHandle, angularDescription);
            body.Activity.TimestepsUnderThresholdCount = 0;
        }
    }

    public void Draw(LineExtractor lines, Camera camera, bool mouseLocked, bool shouldGrab, in Vector2 normalizedMousePosition)
    {
        if (shouldGrab && !active && mouseLocked)
        {
            //Draw a crosshair if there is no mouse cursor.
            var center = camera.Position + camera.Forward * (camera.NearClip * 10);
            var crosshairLength = 0.1f * camera.NearClip * MathF.Tan(camera.FieldOfView * 0.5f);
            var rightOffset = camera.Right * crosshairLength;
            var upOffset = camera.Up * crosshairLength;
            lines.Allocate() = new LineInstance(center - rightOffset, center + rightOffset, new Vector3(1, 0, 0), new Vector3());
            lines.Allocate() = new LineInstance(center - upOffset, center + upOffset, new Vector3(1, 0, 0), new Vector3());
        }
    }
}
