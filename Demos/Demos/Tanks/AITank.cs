using System;
using System.Numerics;
using BepuPhysics;
using BepuUtilities;
using BepuUtilities.Collections;

namespace Demos.Demos.Tanks
{
    public struct AITank
    {
        /// <summary>
        /// Controller used by the AI.
        /// </summary>
        public TankController Controller;
        /// <summary>
        /// Index of the current target of this AI.
        /// </summary>
        public int Target;
        /// <summary>
        /// Current movement target location.
        /// </summary>
        public Vector2 MovementTarget;
        /// <summary>
        /// Index of the last frame that the tank fired on.
        /// </summary>
        public long LastShotFrame;

        /// <summary>
        /// Remaining hit points of the AI. When it hits zero, the tank will fall apart.
        /// </summary>
        internal int HitPoints;

        public void Update(Simulation simulation, BodyProperty<TankDemoBodyProperties> bodyProperties, Random random, long frameIndex, in Vector2 playAreaMin, in Vector2 playAreaMax, int aiIndex, ref QuickList<AITank> aiTanks, ref int projectileCount)
        {
            ref var currentPose = ref simulation.Bodies.GetBodyReference(Controller.Tank.Body).Pose;
            //tankBodyPose = localTankBodyPose * tankPose
            //tankPose = inverse(localTankBodyPose) * tankBodyPose
            QuaternionEx.TransformUnitY(QuaternionEx.Concatenate(QuaternionEx.Conjugate(Controller.Tank.BodyLocalOrientation), currentPose.Orientation), out var tankUp);
            if (tankUp.Y < -0.5f)
            {
                //The tank is upside down. Don't bother doing anything.
                Controller.UpdateMovementAndAim(simulation, 0, 0, false, false, false, new Vector3(1, 0, 0));
                return;
            }
            if (Target >= aiTanks.Count || Target == aiIndex || random.NextDouble() < 1f / 250f)
            {
                //Change target.
                if (aiTanks.Count > 1)
                {
                    do { Target = random.Next(0, aiTanks.Count); } while (Target == aiIndex);
                }
            }
            ref var targetTank = ref aiTanks[Target];
            var targetTankBody = simulation.Bodies.GetBodyReference(targetTank.Controller.Tank.Body);
            ref var targetTankPosition = ref targetTankBody.Pose.Position;
            var currentTankPosition2D = new Vector2(currentPose.Position.X, currentPose.Position.Z);
            var targetTankPosition2D = new Vector2(targetTankPosition.X, targetTankPosition.Z);
            var currentToTargetTank2D = targetTankPosition2D - currentTankPosition2D;
            if (random.NextDouble() < 1f / 250f)
            {
                //Change movement target. Pick a random point around the target's current location.
                //Try to avoid being excessively close, though- that tends to make all the tanks bunch up in the middle.
                Vector2 movementTargetOffset;
                float offsetLengthSquared;
                do
                {
                    movementTargetOffset = new Vector2((float)(random.NextDouble() - 0.5) * 150, (float)(random.NextDouble() - 0.5) * 150);
                    offsetLengthSquared = movementTargetOffset.LengthSquared();

                }
                while (offsetLengthSquared > 150 * 150 || offsetLengthSquared < 50 * 50);
                MovementTarget = Vector2.Min(playAreaMax, Vector2.Max(playAreaMin, targetTankPosition2D + movementTargetOffset));
            }

            var offset = new Vector3(MovementTarget.X - currentTankPosition2D.X, 0, MovementTarget.Y - currentTankPosition2D.Y);
            QuaternionEx.Transform(offset, QuaternionEx.Concatenate(QuaternionEx.Conjugate(currentPose.Orientation), Controller.Tank.BodyLocalOrientation), out var localMovementOffset);

            var targetHorizontalMovementDirection = new Vector2(localMovementOffset.X, -localMovementOffset.Z);
            var targetDirectionLength = targetHorizontalMovementDirection.Length();
            targetHorizontalMovementDirection = targetDirectionLength > 1e-10f ? targetHorizontalMovementDirection / targetDirectionLength : new Vector2(0, 1);
            var turnWeight = targetHorizontalMovementDirection.Y > 0 ? targetHorizontalMovementDirection.X : targetHorizontalMovementDirection.X > 0 ? 1f : -1f;
            //Set the leftTrack to 1 at turnWeight >= 0. At turnWeight -1, leftTrack should be -1.
            var leftTrack = MathF.Min(1f, 2f * turnWeight + 1f);
            //rightTrack = 1 at turnWeight <= 0, rightTrack = -1 at turnWeight = 1.
            var rightTrack = MathF.Min(1f, -2f * turnWeight + 1f);

            //If we're close to the target, slow down a bit.
            var speedMultiplier = Math.Min(targetDirectionLength * 0.05f, 1f);
            leftTrack *= speedMultiplier;
            rightTrack *= speedMultiplier;
            //If we're far away from the target but are pointing in the right direction, zoom.
            var zoom = targetDirectionLength > 50 && targetHorizontalMovementDirection.Y > 0.8f;
            //We're not going to compute an optimal firing solution- just aim directly at the middle of the other tank. Pretty poor choice, but that's fine.
            ref var barrelPosition = ref simulation.Bodies.GetBodyReference(Controller.Tank.Barrel).Pose.Position;
            var barrelToTarget = targetTankPosition - barrelPosition;
            var barrelToTargetLength = barrelToTarget.Length();
            barrelToTarget = barrelToTargetLength > 1e-10f ? barrelToTarget / barrelToTargetLength : new Vector3(0, 1, 0);
            Controller.UpdateMovementAndAim(simulation, leftTrack, rightTrack, zoom, false, false, barrelToTarget);
            if (frameIndex > LastShotFrame + 60)
            {
                //Are we aiming reasonably close to the target?
                if (barrelToTargetLength > 1e-10f && barrelToTargetLength < 100)
                {
                    Controller.Tank.ComputeBarrelDirection(simulation, out var barrelDirection);
                    var dot = Vector3.Dot(barrelDirection, barrelToTarget);
                    if (dot > 0.98f)
                    {
                        Controller.Tank.Fire(simulation, bodyProperties);
                        ++projectileCount;
                    }
                }
                LastShotFrame = frameIndex;
            }
        }

    }
}