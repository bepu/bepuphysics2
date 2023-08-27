using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities.Collections;
using Demos.Demos.Characters;
using System;
using System.Numerics;

namespace Demos.Demos.Sponsors
{
    public struct SponsorCharacterAI
    {
        BodyHandle bodyHandle;
        Vector2 targetLocation;
        public SponsorCharacterAI(CharacterControllers characters, in CollidableDescription characterCollidable, Vector3 initialPosition, in Vector2 targetLocation)
        {
            bodyHandle = characters.Simulation.Bodies.Add(BodyDescription.CreateDynamic(initialPosition, new BodyInertia { InverseMass = 1f }, characterCollidable, -1f));

            ref var character = ref characters.AllocateCharacter(bodyHandle);
            character.LocalUp = new Vector3(0, 1, 0);
            character.CosMaximumSlope = MathF.Cos(MathF.PI * 0.48f);
            character.JumpVelocity = 4;
            character.MaximumVerticalForce = 10f;
            character.MaximumHorizontalForce = 5f;
            character.MinimumSupportDepth = -0.01f;
            character.MinimumSupportContinuationDepth = -0.1f;
            character.ViewDirection = new Vector3(0, 0, -1);
            this.targetLocation = targetLocation;
        }

        public void Update(CharacterControllers characters, Simulation simulation, ref QuickList<SponsorNewt> newts, Random random)
        {
            var body = simulation.Bodies[bodyHandle];
            Vector2 influenceSum = default;
            bool spooked = false;
            for (int i = 0; i < newts.Count; ++i)
            {
                ref var newtPosition = ref simulation.Bodies[newts[i].BodyHandle].Pose.Position;
                var offset = newtPosition - body.Pose.Position;
                var distance = offset.Length();
                if (distance > 1e-10f)
                {
                    var influenceMagnitude = 1f / (distance * 0.1f + .1f);
                    influenceSum -= new Vector2(offset.X, offset.Z) * influenceMagnitude / distance;
                }
                if (distance < 20)
                {
                    spooked = true;
                }
            }
            //We want target position relative influence to be consistent regardless of newt count.
            influenceSum /= newts.Count;
            ref var character = ref characters.GetCharacterByBodyHandle(bodyHandle);
            influenceSum -= (new Vector2(body.Pose.Position.X, body.Pose.Position.Z) - targetLocation) * 0.001f;
            //Newts should do a good job at avoiding a division by zero here, but just in case, guard against it.
            var influenceSumLength = influenceSum.Length();
            var targetWorldVelocity = influenceSumLength > 1e-6f ? influenceSum * (6f / influenceSumLength) : new Vector2();
            //Rephrase the target velocity in terms of the character's control basis. 
            character.TargetVelocity = new Vector2(targetWorldVelocity.X, -targetWorldVelocity.Y);
            if (spooked && random.NextDouble() < 0.015f)
            {
                //oh no oh no oh no he gonna get me
                character.TryJump = true;
            }
        }
    }
}
