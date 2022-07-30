using BepuPhysics;
using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace Demos.SpecializedTests
{
    public static class TestHelpers
    {
        /// <summary>
        /// Gets a value roughly representing the amount of energy in the simulation. This is occasionally handy for debug purposes.
        /// </summary>
        public static float GetBodyEnergyHeuristic(Bodies bodies)
        {
            float accumulated = 0;
            for (int index = 0; index < bodies.ActiveSet.Count; ++index)
            {
                ref var velocity = ref bodies.ActiveSet.DynamicsState[index].Motion.Velocity;
                accumulated += Vector3.Dot(velocity.Linear, velocity.Linear);
                accumulated += Vector3.Dot(velocity.Angular, velocity.Angular);
            }
            return accumulated;
        }
        public static RigidPose CreateRandomPose(Random random, BoundingBox positionBounds)
        {
            RigidPose pose;
            var span = positionBounds.Max - positionBounds.Min;

            pose.Position = positionBounds.Min + span * new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle());
            var axis = new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle());
            var length = axis.Length();
            if (length > 0)
                axis /= length;
            else
                axis = new Vector3(0, 1, 0);
            pose.Orientation = BepuUtilities.QuaternionEx.CreateFromAxisAngle(axis, 1203f * random.NextSingle());
            return pose;
        }

    }
}
