using BepuUtilities.Memory;
using System.Numerics;

namespace BepuPhysics.Collidables
{
    /// <summary>
    /// Defines a type usable as a shape by collidables.
    /// </summary>
    public interface IShape
    {
        int TypeId { get; }

        ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapeBatches);
    }

    //Note that the following bounds functions require only an orientation because the effect of the position on the bounding box is the same for all shapes.
    //By isolating the shape from the position, we can more easily swap out the position representation for higher precision modes while only modifying the stuff that actually
    //deals with positions directly.

    //Note that we also support one-off bounds calculations. They are used even in the engine sometimes. Adding individual bodies to the simulation, for example.
    //Note, however, that we do not bother supporting velocity expansion on the one-off variant. For the purposes of adding objects to the simulation, that is basically irrelevant.
    //I don't predict ever needing it, but such an implementation could be added...
    public interface IConvexShape : IShape
    {
        void GetBounds(ref BepuUtilities.Quaternion orientation, out Vector3 min, out Vector3 max);

        void ComputeLocalInverseInertia(float inverseMass, out Triangular3x3 localInverseInertia);

        bool RayTest(ref RigidPose pose, ref Vector3 origin, ref Vector3 direction, out float t, out Vector3 normal);
    }

    public interface ICompoundShape : IShape
    {
        //Note that compound shapes have no wide GetBounds function. Compounds, by virtue of containing shapes of different types, cannot be usefully vectorized over.
        //Instead, their children are added to other computation batches.
        void GetBounds(ref BepuUtilities.Quaternion orientation, Shapes shapeBatches, out Vector3 min, out Vector3 max);
        
        //Compound shapes may require indirections into other shape batches. This isn't wonderfully fast, but this scalar path is designed more for convenience than performance anyway.
        //For performance, a batched and vectorized codepath should be used.
        bool RayTest(ref RigidPose pose, ref Vector3 origin, ref Vector3 direction, Shapes shapeBatches, out float t, out Vector3 normal);

        void AddChildBoundsToBatcher(ref BoundingBoxBatcher batcher, ref RigidPose pose, ref BodyVelocity velocity, int bodyIndex);
    }


    public interface IShapeWide<TShape> where TShape : IShape
    {
        /// <summary>
        /// Places the specified AOS-formatted shape into the first lane of the wide 'this' reference.
        /// </summary>
        /// <remarks>Note that we are effectively using the TShapeWide as a stride.
        /// The base address is offset by the user of this function, so the implementation only ever considers the first slot.</remarks>
        /// <param name="source">AOS-formatted shape to gather from.</param>
        void Gather(ref TShape source);

        void GetBounds(ref QuaternionWide orientations, out Vector<float> maximumRadius, out Vector<float> maximumAngularExpansion, out Vector3Wide min, out Vector3Wide max);
    }

}
