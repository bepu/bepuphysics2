using BepuPhysics;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuPhysics.Constraints.Contact;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace Demos
{
    public struct ContactManifoldConstraintBuilder : IConstraintBuilder
    {
        static void CreateManifoldConstraint(ref Vector3 unitX, ref Vector3 unitY, ref Vector3 unitZ, ref Vector3 offsetB, out Contact4 description)
        {
            description = new Contact4
            {
                //By convention, normal faces from B to A.
                SpringSettings = new SpringSettings(60, 100),
                MaximumRecoveryVelocity = 1f,
                FrictionCoefficient = 1,
                Normal = unitY,
                OffsetB = offsetB
            };

            for (int contactIndex = 0; contactIndex < 4; ++contactIndex)
            {
                ref var contact = ref Unsafe.Add(ref description.Contact0, contactIndex);

                var x = (contactIndex & 1) - 0.5f;
                var z = ((contactIndex & 2) >> 1) - 0.5f;
                var localOffsetA = new Vector3(x, 0.5f, z);
                var localOffsetB = new Vector3(x, -0.5f, z);
                var worldOffsetA = localOffsetA.X * unitX + localOffsetA.Y * unitY + localOffsetA.Z * unitZ;
                var worldOffsetB = localOffsetB.X * unitX + localOffsetB.Y * unitY + localOffsetB.Z * unitZ;
                contact.OffsetA = worldOffsetA;
                contact.PenetrationDepth = 0.00f;
            }
        }

        public void BuildConstraintsForBody(int sliceIndex, int rowIndex, int columnIndex, ref BodyDescription bodyDescription,
            ref LatticeBodyGetter ids, ref ConstraintAdder constraintAdder)
        {
            //For each lower neighbor, create a connection.
            if (ids.GetBody(columnIndex - 1, rowIndex, sliceIndex, out var previousColumnHandle, out var previousColumnDescription) &&
                (bodyDescription.LocalInertia.InverseMass != 0 || previousColumnDescription.LocalInertia.InverseMass != 0))
            {
                SimulationSetup.BuildBasis(ref bodyDescription.Pose, ref previousColumnDescription.Pose, out var offsetB, out var x, out var y, out var z);
                CreateManifoldConstraint(ref x, ref y, ref z, ref offsetB, out var description);
                constraintAdder.Add(ref description, previousColumnHandle);
            }
            if (ids.GetBody(columnIndex, rowIndex - 1, sliceIndex, out var previousRowHandle, out var previousRowDescription) &&
                (bodyDescription.LocalInertia.InverseMass != 0 || previousRowDescription.LocalInertia.InverseMass != 0))
            {
                SimulationSetup.BuildBasis(ref bodyDescription.Pose, ref previousRowDescription.Pose, out var offsetB, out var x, out var y, out var z);
                CreateManifoldConstraint(ref x, ref y, ref z, ref offsetB, out var description);
                constraintAdder.Add(ref description, previousRowHandle);
            }
            if (ids.GetBody(columnIndex, rowIndex, sliceIndex - 1, out var previousSliceHandle, out var previousSliceDescription) &&
                (bodyDescription.LocalInertia.InverseMass != 0 || previousSliceDescription.LocalInertia.InverseMass != 0))
            {
                SimulationSetup.BuildBasis(ref bodyDescription.Pose, ref previousSliceDescription.Pose, out var offsetB, out var x, out var y, out var z);
                CreateManifoldConstraint(ref x, ref y, ref z, ref offsetB, out var description);
                constraintAdder.Add(ref description, previousSliceHandle);
            }
        }

    }


}

