using BepuPhysics;

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

