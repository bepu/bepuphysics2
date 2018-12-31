using BepuUtilities;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Collidables;
using System.Numerics;
using Quaternion = BepuUtilities.Quaternion;
using System;
using BepuPhysics.CollisionDetection;
using System.Runtime.CompilerServices;
using System.Diagnostics;
using BepuPhysics.Constraints;
using DemoContentLoader;
using DemoUtilities;
using BepuUtilities.Memory;
using static BepuUtilities.GatherScatter;

namespace Demos.Demos
{
    //Constraint descriptions provide an explicit mapping from the array-of-structures format to the internal array-of-structures-of-arrays format used by the solver.
    /// <summary>
    /// Description of a character motion constraint.
    /// </summary>
    public struct CharacterMotionConstraint : IConstraintDescription<CharacterMotionConstraint>
    {
        /// <summary>
        /// Stores the quaternion-packed orthonormal basis for the motion constraint. When expanded into a matrix, X and Z will represent the Right and Backward directions respectively. Y will represent Up.
        /// In other words, a target tangential velocity of (4, 2) will result in a goal velocity of 4 along the (1, 0, 0) * Basis direction and a goal velocity of 2 along the (0, 0, -1) * Basis direction.
        /// All motion moving along the (0, 1, 0) * Basis axis will be fought against by the vertical motion constraint.
        /// </summary>
        public Quaternion SurfaceBasis;
        /// <summary>
        /// Point at which to apply impulses to the supporting body and character. Stored as a world space offset from the character body center to the support point.
        /// </summary>
        public Vector3 SupportPoint;
        /// <summary>
        /// Maximum force that the horizontal motion constraint can apply to reach the current velocity goal.
        /// </summary>
        public float MaximumHorizontalForce;
        /// <summary>
        /// Maximum force that the vertical motion constraint can apply to fight separation.
        /// </summary>
        public float MaximumVerticalForce;
        /// <summary>
        /// Target horizontal velocity in terms of the basis X and -Z axes.
        /// </summary>
        public Vector2 TargetVelocity;
        
        //It's possible to create multiple descriptions for the same underlying constraint type id which can update different parts of the constraint data.
        //This functionality isn't used very often, though- you'll notice that the engine has a 1:1 mapping (at least at the time of this writing).
        //But in principle, it doesn't have to be that way. So, the description must provide information about the type and type id.
        /// <summary>
        /// Gets the constraint type id that this description is associated with. 
        /// </summary>
        public int ConstraintTypeId => CharacterMotionTypeProcessor.BatchTypeId;

        /// <summary>
        /// Gets the TypeProcessor type that is associated with this description.
        /// </summary>
        public Type BatchType => typeof(CharacterMotionTypeProcessor);

        //Note that these mapping functions use a "GetOffsetInstance" function. Each CharacterMotionPrestep is a bundle of multiple constraints;
        //by grabbing an offset instance, we're selecting a specific slot in the bundle to modify. For simplicity and to guarantee consistency of field strides,
        //we refer to that slot using the same struct and then write only to the first slot.
        //(Note that accessing slots after the first may result in access violations; the 'offset instance' is not guaranteed to refer to valid data beyond the first slot!)
        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<CharacterMotionPrestep>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            QuaternionWide.WriteFirst(SurfaceBasis, ref target.SurfaceBasis);
            Vector3Wide.WriteFirst(SupportPoint, ref target.SupportPoint);
            GetFirst(ref target.MaximumHorizontalForce) = MaximumHorizontalForce;
            GetFirst(ref target.MaximumVerticalForce) = MaximumVerticalForce;
            Vector2Wide.WriteFirst(TargetVelocity, ref target.TargetVelocity);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out CharacterMotionConstraint description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<CharacterMotionPrestep>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            QuaternionWide.ReadFirst(source.SurfaceBasis, out description.SurfaceBasis);
            Vector3Wide.ReadFirst(source.SupportPoint, out description.SupportPoint);
            description.MaximumHorizontalForce = GetFirst(ref source.MaximumHorizontalForce);
            description.MaximumVerticalForce = GetFirst(ref source.MaximumVerticalForce);
            Vector2Wide.ReadFirst(source.TargetVelocity, out description.TargetVelocity);
        }
    }
        
    //Note that all the solver-side data is in terms of 'Wide' data types- the solver never works on just one constraint at a time. Instead,
    //it executes them in bundles of width equal to the runtime/hardware exposed SIMD unit width. This lets the solver scale with wider compute units.
    //(This is important for machines that can perform 8 or more operations per instruction- there's no good way to map a single constraint instance's 
    //computation onto such a wide instruction, so if the solver tried to do such a thing, it would leave a huge amount of performance on the table.)

    //"Prestep" data can be thought of as the input to the solver. It describes everything the solver needs to know about.
    /// <summary>
    /// AOSOA formatted bundle of prestep data for multiple character motion constraints.
    /// </summary>
    public struct CharacterMotionPrestep
    {
        //Note that the prestep data layout is important. The solver tends to be severely memory bandwidth bound, so using a minimal representation is valuable.
        //That's why the Basis is stored as a quaternion and not a full Matrix- the cost of the arithmetic operations to expand it back into the original matrix form is far less
        //than the cost of loading all the extra lanes of data when scaled up to many cores.
        public QuaternionWide SurfaceBasis;
        public Vector3Wide SupportPoint;
        public Vector<float> MaximumHorizontalForce;
        public Vector<float> MaximumVerticalForce;
        public Vector2Wide TargetVelocity;
    }
    
    //Using the prestep data plus some current body state, the solver computes the information required to execute velocity iterations. The main purpose of this intermediate data
    //is to describe the projection from body velocities into constraint space impulses, and from constraint space impulses to body velocities again.
    public struct CharacterMotionProjection
    {

    }

    public struct CharacterMotionFunctions : IConstraintFunctions<CharacterMotionPrestep, CharacterMotionProjection, Vector3Wide>
    {
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count, float dt, float inverseDt, ref BodyInertias inertiaA, ref BodyInertias inertiaB, ref CharacterMotionPrestep prestepData, out CharacterMotionProjection projection)
        {
            throw new NotImplementedException();
        }

        public void Solve(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref CharacterMotionProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            throw new NotImplementedException();
        }

        public void WarmStart(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref CharacterMotionProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            throw new NotImplementedException();
        }
    }

    //Each constraint type has its own 'type processor'- it acts as the outer loop that handles all the common logic across batches of constraints and invokes
    //the per-constraint logic as needed. The CharacterMotionFunctions type provides the actual implementation.
    public class CharacterMotionTypeProcessor : TwoBodyTypeProcessor<CharacterMotionPrestep, CharacterMotionProjection, Vector3Wide, CharacterMotionFunctions>
    {
        /// <summary>
        /// Simulation-wide unique id for the character motion constraint. Every type has needs a unique compile time id; this is a little bit annoying to guarantee given that there is no central
        /// registry of all types that can exist (custom ones, like this one, can always be created), but having it be constant helps simplify and optimize its internal usage.
        /// </summary>
        public const int BatchTypeId = 40;
    }

    /// <summary>
    /// Shows how to use a simple character controller.
    /// </summary>
    public class CharacterDemo : Demo
    {
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-20, 10, -20);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = MathHelper.Pi * 0.05f;
            var masks = new BodyProperty<ulong>();
            Simulation = Simulation.Create(BufferPool, new RagdollCallbacks { Masks = masks }, new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));
                       
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -0.5f, 0), new CollidableDescription(Simulation.Shapes.Add(new Box(300, 1, 300)), 0.1f)));
        }

    }
}


