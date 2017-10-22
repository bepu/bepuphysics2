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
    public struct ConstraintlessLatticeBuilder : IConstraintBuilder
    {
        public void BuildConstraintsForBody(int sliceIndex, int rowIndex, int columnIndex,
            ref BodyDescription bodyDescription,
            ref LatticeBodyGetter ids,
            ref ConstraintAdder constraintAdder)
        {
        }

        public void RegisterConstraintTypes()
        {
        }
    }


}

