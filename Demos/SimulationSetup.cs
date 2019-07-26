using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using Demos.SpecializedTests;
using System.Collections.Generic;
using System.Numerics;

using Quaternion = BepuUtilities.Quaternion;

namespace Demos
{
    public interface IBodyBuilder
    {
        void Build(int columnIndex, int rowIndex, int sliceIndex, out BodyDescription bodyDescription);
    }
    public interface IConstraintBuilder
    {
        void BuildConstraintsForBody(int sliceIndex, int rowIndex, int columnIndex,
            ref BodyDescription bodyDescription, ref LatticeBodyGetter ids, ref ConstraintAdder constraintAdder);
    }
    public struct LatticeBodyGetter
    {
        int width, height, length;
        int[] bodyHandles;
        Bodies bodies;
        public LatticeBodyGetter(int width, int height, int length, int[] bodyHandles, Bodies bodies)
        {
            this.width = width;
            this.height = height;
            this.length = length;
            this.bodyHandles = bodyHandles;
            this.bodies = bodies;
        }
        public bool TryGetId(int columnIndex, int rowIndex, int sliceIndex, out int id)
        {
            if (columnIndex < 0 || columnIndex >= width || rowIndex < 0 || rowIndex >= height || sliceIndex < 0 || sliceIndex >= length)
            {
                id = -1;
                return false;
            }
            id = sliceIndex * (height * width) + rowIndex * width + columnIndex;
            return true;
        }
        public bool GetBody(int columnIndex, int rowIndex, int sliceIndex, out int handle, out BodyDescription bodyDescription)
        {
            if (!TryGetId(columnIndex, rowIndex, sliceIndex, out var id))
            {
                handle = -1;
                bodyDescription = new BodyDescription();
                return false;
            }
            handle = bodyHandles[id];
            bodies.GetDescription(handle, out bodyDescription);
            return true;
        }
    }

    public struct ConstraintAdder
    {
        public int LocalBodyHandle;
        Simulation simulation;
        public List<int> ConstraintHandles;
        public ConstraintAdder(Simulation simulation, List<int> constraintHandles)
        {
            this.simulation = simulation;
            this.ConstraintHandles = constraintHandles;
            LocalBodyHandle = 0;
        }

        public void Add<T>(ref T description, int otherBodyHandle) where T : ITwoBodyConstraintDescription<T>
        {
            var constraintHandle = simulation.Solver.Add(LocalBodyHandle, otherBodyHandle, ref description);
            ConstraintHandles.Add(constraintHandle);
        }
    }

    public static class SimulationSetup
    {
        public static void BuildBasis(ref RigidPose a, ref RigidPose b, out Vector3 offsetB, out Vector3 x, out Vector3 y, out Vector3 z)
        {
            offsetB = b.Position - a.Position;
            y = Vector3.Normalize(-offsetB);
            Quaternion.TransformUnitZ(a.Orientation, out var ax);
            x = Vector3.Cross(ax, y);
            var xLength = x.Length();
            if (xLength > 1e-7)
            {
                x /= xLength;
            }
            else
            {
                Quaternion.TransformUnitX(a.Orientation, out var az);
                x = Vector3.Normalize(Vector3.Cross(az, y));
            }
            z = Vector3.Cross(x, y);
        }

        public static void BuildLattice<TBodyBuilder, TConstraintBuilder>(TBodyBuilder bodyBuilder, TConstraintBuilder constraintBuilder, int width, int height, int length, Simulation simulation,
            out int[] bodyHandles, out int[] constraintHandles) where TBodyBuilder : IBodyBuilder where TConstraintBuilder : IConstraintBuilder
        {
            var bodyCount = width * height * length;
            bodyHandles = new int[bodyCount];

            var bodyGetter = new LatticeBodyGetter(width, height, length, bodyHandles, simulation.Bodies);

            for (int sliceIndex = 0; sliceIndex < length; ++sliceIndex)
            {
                for (int rowIndex = 0; rowIndex < height; ++rowIndex)
                {
                    for (int columnIndex = 0; columnIndex < width; ++columnIndex)
                    {
                        bodyBuilder.Build(columnIndex, rowIndex, sliceIndex, out var bodyDescription);
                        bodyGetter.TryGetId(columnIndex, rowIndex, sliceIndex, out var id);
                        bodyHandles[id] = simulation.Bodies.Add(bodyDescription);
                    }
                }
            }
            
            var constraintAdder = new ConstraintAdder(simulation, new List<int>(width * height * length * 3));
            for (int sliceIndex = 0; sliceIndex < length; ++sliceIndex)
            {
                //The bottom rows are all kinematic, so don't create connections between them.
                for (int rowIndex = 0; rowIndex < height; ++rowIndex)
                {
                    for (int columnIndex = 0; columnIndex < width; ++columnIndex)
                    {
                        bodyGetter.GetBody(columnIndex, rowIndex, sliceIndex, out constraintAdder.LocalBodyHandle, out var bodyDescription);
                        constraintBuilder.BuildConstraintsForBody(sliceIndex, rowIndex, columnIndex, ref bodyDescription, ref bodyGetter, ref constraintAdder);
                    }
                }
            }
            constraintHandles = constraintAdder.ConstraintHandles.ToArray();


        }


        public static void BuildLattice<TBodyBuilder, TConstraintBuilder>(TBodyBuilder bodyBuilder, TConstraintBuilder constraintBuilder, int width, int height, int length,
            out Simulation simulation,
            out int[] bodyHandles, out int[] constraintHandles) where TBodyBuilder : IBodyBuilder where TConstraintBuilder : IConstraintBuilder
        {
            var bodyCount = width * height * length;
            simulation = Simulation.Create(
                new BufferPool(),
                new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)),
                initialAllocationSizes: new SimulationAllocationSizes
                {
                    Bodies = bodyCount,
                    ShapesPerType = 128,
                    Constraints = bodyCount * 3,
                    ConstraintsPerTypeBatch = (bodyCount * 3) / 6,
                    ConstraintCountPerBodyEstimate = 6
                });
            BuildLattice(bodyBuilder, constraintBuilder, width, height, length, simulation, out bodyHandles, out constraintHandles);

        }




    }
}

