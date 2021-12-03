using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using Helpers = DemoRenderer.Helpers;

namespace Demos.Demos.Sponsors
{
    public struct SponsorNewt
    {
        public int SponsorIndex;
        public BodyHandle BodyHandle;
        public SponsorNewt(Simulation simulation, TypedIndex shape, float height, in Vector2 arenaMin, in Vector2 arenaMax, Random random, int sponsorIndex) : this()
        {
            var arenaSpan = arenaMax - arenaMin;
            var position = arenaMin + arenaSpan * new Vector2(random.NextSingle(), random.NextSingle());
            var angle = MathF.PI * 2 * random.NextSingle();
            BodyHandle = simulation.Bodies.Add(BodyDescription.CreateKinematic((new Vector3(position.X, height, position.Y), QuaternionEx.CreateFromAxisAngle(Vector3.UnitY, angle)), shape, -1));
            SponsorIndex = sponsorIndex;
        }

        //The newt hops between predetermined points unstoppably, waiting a moment in between jumps.
        double nextAllowedJump;
        Vector2 jumpStart, jumpEnd;
        //The orientation will interpolate while jumping.
        Vector2 forwardAtJumpStart, forwardAtJumpEnd;
        double jumpStartTime, jumpEndTime;

        public void Update(Simulation simulation, double time, float height, in Vector2 arenaMin, in Vector2 arenaMax, Random random, float inverseDt)
        {
            const float jumpDuration = 1;
            var body = simulation.Bodies[BodyHandle];
            if (time >= nextAllowedJump)
            {
                //Choose a jump location. It should be within the arena, and generally somewhere ahead of the newt.
                QuaternionEx.TransformUnitZ(body.Pose.Orientation, out var backward);
                jumpStart = new Vector2(body.Pose.Position.X, body.Pose.Position.Z);
                jumpEnd = jumpStart + new Vector2(backward.X + random.NextSingle() * 1.4f - 0.7f, backward.Z + random.NextSingle() * 1.4f - 0.7f) * -20;
                jumpEnd -= jumpStart * 0.05f;
                jumpEnd = Vector2.Max(arenaMin, Vector2.Min(arenaMax, jumpEnd));
                jumpStartTime = time;
                jumpEndTime = time + jumpDuration;
                forwardAtJumpStart = -new Vector2(backward.X, backward.Z);
                forwardAtJumpEnd = jumpEnd - jumpStart;
                var newForwardLengthSquared = forwardAtJumpEnd.LengthSquared();
                forwardAtJumpEnd = newForwardLengthSquared < 1e-10f ? forwardAtJumpStart : forwardAtJumpEnd / MathF.Sqrt(newForwardLengthSquared);
                nextAllowedJump = jumpEndTime + (1 + random.NextDouble() * 2.5f);
            }
            Vector3 targetPosition;
            Vector2 targetForward;
            if (time >= jumpStartTime && time <= jumpEndTime)
            {
                //The newt's in the middle of a jump. Choose a target position/orientation by interpolation.
                const float maximumJumpHeight = 5;
                var jumpProgress = (float)(time - jumpStartTime) / jumpDuration;
                var targetPosition2D = (float)jumpProgress * (jumpEnd - jumpStart) + jumpStart;
                var parabolaTerm = (2 * jumpProgress - 1);
                var currentHeight = height + (1 - parabolaTerm * parabolaTerm) * maximumJumpHeight;
                targetPosition = new Vector3(targetPosition2D.X, currentHeight, targetPosition2D.Y);
                targetForward = jumpProgress * (forwardAtJumpEnd - forwardAtJumpStart) + forwardAtJumpStart;
                var targetForwardLengthSquared = targetForward.LengthSquared();
                if (targetForwardLengthSquared < 1e-10f)
                {
                    QuaternionEx.TransformUnitZ(body.Pose.Orientation, out var backward);
                    targetForward = -new Vector2(backward.X, backward.Z);
                }
                else
                {
                    targetForward /= MathF.Sqrt(targetForwardLengthSquared);
                }
            }
            else
            {
                //The target pose is just wherever the previous jump ended.
                targetPosition = new Vector3(jumpEnd.X, height, jumpEnd.Y);
                targetForward = forwardAtJumpEnd;
            }

            //Since it's a kinematic body, we'll compute the current pose error, and then the velocity to correct that error within a single frame.
            body.Velocity.Linear = (targetPosition - body.Pose.Position) * inverseDt;
            Matrix3x3 targetOrientationBasis;
            targetOrientationBasis.X = new Vector3(-targetForward.Y, 0, targetForward.X);
            targetOrientationBasis.Y = Vector3.UnitY;
            targetOrientationBasis.Z = -new Vector3(targetForward.X, 0, targetForward.Y);
            QuaternionEx.CreateFromRotationMatrix(targetOrientationBasis, out var targetOrientation);
            QuaternionEx.GetRelativeRotationWithoutOverlap(body.Pose.Orientation, targetOrientation, out var orientationError);
            QuaternionEx.GetAxisAngleFromQuaternion(orientationError, out var errorAxis, out var errorAngle);
            body.Velocity.Angular = errorAxis * (errorAngle * inverseDt);
        }

        public readonly void Render(Simulation simulation, List<Sponsor> sponsors, Renderer renderer, in Matrix viewProjection, in Vector2 resolution, TextBuilder text, Font font)
        {
            var name = sponsors[SponsorIndex].Name;
            var body = simulation.Bodies[BodyHandle];
            Helpers.GetScreenLocation(body.Pose.Position + new Vector3(0, 8, 0), viewProjection, resolution, out var screenspacePosition);
            const float nameHeight = 14;
            var nameLength = GlyphBatch.MeasureLength(name, font, nameHeight);
            screenspacePosition.X -= nameLength * 0.5f;
            renderer.TextBatcher.Write(text.Clear().Append(name), screenspacePosition, nameHeight, new Vector3(0.3f, 0f, 0f), font);
        }
    }
}
