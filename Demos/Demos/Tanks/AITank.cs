using System;
using System.Numerics;
using BepuPhysics;
using BepuUtilities.Collections;

namespace Demos.Demos.Tanks
{
    public struct AITank
    {
        public TankController Controller;
        /// <summary>
        /// Index of the current target of this AI.
        /// </summary>
        public int Target;
        /// <summary>
        /// Current movement target location.
        /// </summary>
        public Vector2 MovementTarget;
        public long LastShotFrame;

        public void Update(Simulation simulation, BodyProperty<TankBodyProperties> bodyProperties, Random random, long frameIndex, in Vector2 playAreaMin, in Vector2 playAreaMax, ref QuickList<AITank> aiTanks)
        {
            if (random.NextDouble() < 1f / 1000f)
            {
                //Change target.
                Target = random.Next(0, aiTanks.Count);
            }
            ref var targetTank = ref aiTanks[Target];
            var targetTankBody = simulation.Bodies.GetBodyReference(targetTank.Controller.Tank.Body);
            ref var targetTankPosition = ref targetTankBody.Pose.Position;
            if (random.NextDouble() < 1f / 1000f)
            {
                //Change movement target. Pick a random point around the target's current location.
                var unclampedMovementTarget = new Vector2(targetTankPosition.X + (float)(random.NextDouble() - 0.5) * 100, targetTankPosition.X + (float)(random.NextDouble() - 0.5) * 100);
                MovementTarget = Vector2.Min(playAreaMax, Vector2.Max(playAreaMin, unclampedMovementTarget));
            }
            if (frameIndex > LastShotFrame + 60)
            {
                //Are we aiming reasonably close to the target?
                //We're not going to compute an optimal firing solution- just aim directly at the middle of the other tank. Pretty poor choice, but that's fine.
                ref var barrelPosition = ref simulation.Bodies.GetBodyReference(targetTank.Controller.Tank.Barrel).Pose.Position;
                var barrelToTarget = barrelPosition - targetTankPosition;
                var length = barrelToTarget.Length();
                if (length > 1e-10f)
                {
                    targetTank.Controller.Tank.ComputeBarrelDirection(simulation, out var barrelDirection);
                    var dot = Vector3.Dot(barrelDirection, barrelToTarget) / length;
                    if (dot > 0.98f)
                    {
                        Controller.Tank.Fire(simulation, bodyProperties);
                    }
                }
                LastShotFrame = frameIndex;
            }
        }

    }
}