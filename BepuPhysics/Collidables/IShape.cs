using BepuUtilities.Memory;
using System.Numerics;

namespace BepuPhysics.Collidables
{
    /// <summary>
    /// Defines a type usable as a shape by collidables.
    /// </summary>
    public interface IShape
    {

        //Note that the shape gathering required for get bounds is also useful for narrow phase calculations.
        //However, exposing it in a type-safe way isn't trivial. So instead, we just choose at the API level to bundle the gather and AABB calculation together.
        //Analogously to the bounds calculation, narrow phase pairs will have the type information to directly call the underlying type's gather function.
        void GetBounds<TShape>(ref Buffer<TShape> shapes, ref Vector<int> shapeIndices, int count, ref QuaternionWide orientations,
           out Vector<float> maximumRadius, out Vector<float> maximumAngularExpansion, out Vector3Wide min, out Vector3Wide max)
           where TShape : struct, IShape;
        //One-off bounds calculations are useful sometimes, even within the engine. Adding individual bodies to the simulation, for example.
        //(Of course, if you wanted to add a lot of bodies, you'd want to batch everything up and use either cached values or the above bundle bounds calculator, but 
        //for most use cases, body-adding isn't the bottleneck.)
        //Note, however, that we do not bother supporting velocity expansion on the one-off variant. For the purposes of adding objects to the simulation, that is basically irrelevant.
        //I don't predict ever needing it, but such an implementation could be added...
        void GetBounds(ref BepuUtilities.Quaternion orientation, out Vector3 min, out Vector3 max);

        bool RayTest(ref RigidPose pose, ref Vector3 origin, ref Vector3 direction, out float t, out Vector3 normal);

        void ComputeLocalInverseInertia(float inverseMass, out Triangular3x3 localInverseInertia);

        //These functions require only an orientation because the effect of the position on the bounding box is the same for all shapes.
        //By isolating the shape from the position, we can more easily swap out the position representation for higher precision modes while only modifying the stuff that actually
        //deals with positions directly.

        int TypeId { get; }
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
    }

}
