using BepuUtilities;
using BepuPhysics;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

using Quaternion = BepuUtilities.Quaternion;

namespace Demos
{
    public struct BallSocketConstraintBuilder : IConstraintBuilder
    {
        static void CreateBallSocket(ref RigidPose a, ref RigidPose b, out BallSocket description)
        {
            var midpoint = 0.5f * (a.Position + b.Position);
            description.LocalOffsetA = Quaternion.Transform(midpoint - a.Position, Quaternion.Conjugate(a.Orientation));
            description.LocalOffsetB = Quaternion.Transform(midpoint - b.Position, Quaternion.Conjugate(b.Orientation));
            description.SpringSettings = new SpringSettings(15, 0.1f);
        }
        static void TryConnectTo(int sliceIndex, int rowIndex, int columnIndex,
            ref BodyDescription bodyDescription,
            ref LatticeBodyGetter ids, 
            ref ConstraintAdder constraintAdder)
        {
            if (ids.GetBody(columnIndex, rowIndex, sliceIndex, out var otherHandle, out var otherDescription) &&
                (bodyDescription.LocalInertia.InverseMass > 0 || otherDescription.LocalInertia.InverseMass > 0))
            {
                CreateBallSocket(ref bodyDescription.Pose, ref otherDescription.Pose, out var description);
                constraintAdder.Add(ref description, otherHandle);
            }
        }
        public void BuildConstraintsForBody(int sliceIndex, int rowIndex, int columnIndex,
            ref BodyDescription bodyDescription,
            ref LatticeBodyGetter ids,
            ref ConstraintAdder constraintAdder)
        {
            //For each lesser neighbor along each main axis, create a connection.
            TryConnectTo(sliceIndex - 1, rowIndex, columnIndex, ref bodyDescription, ref ids, ref constraintAdder);
            TryConnectTo(sliceIndex, rowIndex - 1, columnIndex, ref bodyDescription, ref ids, ref constraintAdder);
            TryConnectTo(sliceIndex, rowIndex, columnIndex - 1, ref bodyDescription, ref ids, ref constraintAdder);
            //Create the four diagonals downward.
            TryConnectTo(sliceIndex - 1, rowIndex - 1, columnIndex - 1, ref bodyDescription, ref ids, ref constraintAdder);
            TryConnectTo(sliceIndex + 1, rowIndex - 1, columnIndex - 1, ref bodyDescription, ref ids, ref constraintAdder);
            TryConnectTo(sliceIndex - 1, rowIndex - 1, columnIndex + 1, ref bodyDescription, ref ids, ref constraintAdder);
            TryConnectTo(sliceIndex + 1, rowIndex - 1, columnIndex + 1, ref bodyDescription, ref ids, ref constraintAdder);
        }
    }


}

