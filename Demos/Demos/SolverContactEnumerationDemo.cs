using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuPhysics.Constraints.Contact;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Numerics;

namespace Demos.Demos
{
    /// <summary>
    /// Stores the easy to read data associated with a single contact extracted from solver contact constraints.
    /// </summary>
    struct ExtractedContact
    {
        public Vector3 OffsetA;
        public float Depth;
        //For the purposes of the demo, we'll store a normal for every single contact, even though the original manifold might have been convex (and so shared one normal across all its contacts).
        public Vector3 Normal;
        public float PenetrationImpulse;
        //We'll also derive friction impulse from different sources depending on convexity- convex manifolds have a single twist/tangent friction constraint, while nonconvex manifolds have a per-contact tangent friction.
        public float FrictionImpulseMagnitude;
    }

    /// <summary>
    /// Stores the connected bodies and contacts for a contact manifold constraint extracted from the solver.
    /// </summary>
    struct ExtractedManifold
    {
        public QuickList<ExtractedContact> Contacts;
        public BodyHandle BodyA;
        //For one body constraints, this will be -1.
        public BodyHandle BodyB;

        public ExtractedManifold(BufferPool pool, BodyHandle a, BodyHandle b)
        {
            //Nonconvex manifolds will never have less than the convex count, so we'll preallocate enough space for a nonconvex manifold.
            Contacts = new QuickList<ExtractedContact>(NonconvexContactManifold.MaximumContactCount, pool);
            BodyA = a;
            BodyB = b;
        }

        public ExtractedManifold(BufferPool pool, BodyHandle a) : this(pool, a, new BodyHandle(-1)) { }

        public void Dispose(BufferPool pool)
        {
            Contacts.Dispose(pool);
        }
    }

    /// <summary>
    /// Example implementation of a <see cref="ISolverContactDataExtractor"/> that pulls contact data into an easier to read format.
    /// </summary>
    struct SolverContactDataExtractor : ISolverContactDataExtractor
    {
        //We'll pull the solver data into a different form- just a list of contacts with basic data about each one.
        //What you store depends on your application's needs- there's nothing saying you have to use this layout.
        //The ISolverContactDataExtractor can be thought of as the foundation on which to build other (more convenient) abstractions.
        public QuickList<ExtractedManifold> Constraints;
        public BufferPool Pool;

        public SolverContactDataExtractor(BufferPool pool, int initialCapacity)
        {
            Pool = pool;
            Constraints = new QuickList<ExtractedManifold>(initialCapacity, pool);
        }

        //The callbacks distinguish between convex and nonconvex because the underlying data is different.
        //Nonconvex manifolds can have different normals at each contact, while convex manifolds share one normal across all contacts.
        //This also affects the accumulated impulses- nonconvex impulses need to include per-contact tangent friction impulses due to the potentially differing normals.

        //In all of the following, you might notice an unusual pattern- things like prestep.GetNormal(ref prestep...). 
        //This is a bit of a hack to work around the fact that 
        //1) struct member functions cannot return a reference to the 'this' instance, and 
        //2) there is no way in the current version of C# for an interface to require a static function. (This will change with static abstracts!)
        //It's another weird little tradeoff for minimal overhead.

        void ExtractConvexData<TPrestep, TAccumulatedImpulses>(ref ExtractedManifold constraintContacts, ref TPrestep prestep, ref TAccumulatedImpulses impulses)
            where TPrestep : struct, IConvexContactPrestep<TPrestep>
            where TAccumulatedImpulses : struct, IConvexContactAccumulatedImpulses<TAccumulatedImpulses>
        {
            //Note that we narrow from the raw vectorized reference into a more application-convenient AOS representation.
            Vector3Wide.ReadFirst(prestep.GetNormal(ref prestep), out var normal);

            //We'll approximate the per-contact friction by allocating the shared friction impulses to contacts weighted by their penetration impulse.
            float totalPenetrationImpulse = 0;
            for (int i = 0; i < prestep.ContactCount; ++i)
            {
                ref var sourceContact = ref prestep.GetContact(ref prestep, i);
                ref var targetContact = ref constraintContacts.Contacts.AllocateUnsafely();
                Vector3Wide.ReadFirst(sourceContact.OffsetA, out targetContact.OffsetA);
                //We can use [0] to access the slot because the prestep bundle memory reference was already offset for us.
                targetContact.Depth = sourceContact.Depth[0];
                targetContact.Normal = normal;
                targetContact.PenetrationImpulse = impulses.GetPenetrationImpulseForContact(ref impulses, i)[0];
                totalPenetrationImpulse += targetContact.PenetrationImpulse;
            }
            Vector2Wide.ReadFirst(impulses.GetTangentFriction(ref impulses), out var tangentFriction);
            var twistFriction = impulses.GetTwistFriction(ref impulses)[0];
            //This isn't a 'correct' allocation of impulses, we just want a rough sense.
            var frictionMagnitudeApproximation = MathF.Sqrt(tangentFriction.LengthSquared() + twistFriction * twistFriction);
            var impulseScale = totalPenetrationImpulse > 0 ? frictionMagnitudeApproximation / totalPenetrationImpulse : 0;
            for (int i = 0; i < prestep.ContactCount; ++i)
            {
                ref var contact = ref constraintContacts.Contacts[i];
                contact.FrictionImpulseMagnitude = contact.PenetrationImpulse * impulseScale;
            }
        }

        public void ConvexOneBody<TPrestep, TAccumulatedImpulses>(BodyHandle bodyHandle, ref TPrestep prestep, ref TAccumulatedImpulses impulses)
            where TPrestep : struct, IConvexContactPrestep<TPrestep>
            where TAccumulatedImpulses : struct, IConvexContactAccumulatedImpulses<TAccumulatedImpulses>
        {
            ref var constraintContacts = ref Constraints.Allocate(Pool);
            constraintContacts = new ExtractedManifold(Pool, bodyHandle);
            ExtractConvexData(ref constraintContacts, ref prestep, ref impulses);
        }

        public void ConvexTwoBody<TPrestep, TAccumulatedImpulses>(BodyHandle bodyHandleA, BodyHandle bodyHandleB, ref TPrestep prestep, ref TAccumulatedImpulses impulses)
            where TPrestep : struct, ITwoBodyConvexContactPrestep<TPrestep>
            where TAccumulatedImpulses : struct, IConvexContactAccumulatedImpulses<TAccumulatedImpulses>
        {
            ref var constraintContacts = ref Constraints.Allocate(Pool);
            constraintContacts = new ExtractedManifold(Pool, bodyHandleA, bodyHandleB);
            ExtractConvexData(ref constraintContacts, ref prestep, ref impulses);
        }

        void ExtractNonconvexData<TPrestep, TAccumulatedImpulses>(ref ExtractedManifold constraintContacts, ref TPrestep prestep, ref TAccumulatedImpulses impulses)
            where TPrestep : struct, INonconvexContactPrestep<TPrestep>
            where TAccumulatedImpulses : struct, INonconvexContactAccumulatedImpulses<TAccumulatedImpulses>
        {
            //Nonconvex types require no approximation of friction; we can pull it directly from the solved results.
            for (int i = 0; i < prestep.ContactCount; ++i)
            {
                ref var sourceContact = ref prestep.GetContact(ref prestep, i);
                ref var targetContact = ref constraintContacts.Contacts.AllocateUnsafely();
                Vector3Wide.ReadFirst(sourceContact.Offset, out targetContact.OffsetA);
                targetContact.Depth = sourceContact.Depth[0];
                Vector3Wide.ReadFirst(sourceContact.Normal, out targetContact.Normal);

                ref var contactImpulses = ref impulses.GetImpulsesForContact(ref impulses, i);
                targetContact.PenetrationImpulse = contactImpulses.Penetration[0];
                Vector2Wide.ReadFirst(contactImpulses.Tangent, out var tangentImpulses);
                targetContact.FrictionImpulseMagnitude = tangentImpulses.Length();
            }
        }

        public void NonconvexOneBody<TPrestep, TAccumulatedImpulses>(BodyHandle bodyHandle, ref TPrestep prestep, ref TAccumulatedImpulses impulses)
            where TPrestep : struct, INonconvexContactPrestep<TPrestep>
            where TAccumulatedImpulses : struct, INonconvexContactAccumulatedImpulses<TAccumulatedImpulses>
        {
            ref var constraintContacts = ref Constraints.Allocate(Pool);
            constraintContacts = new ExtractedManifold(Pool, bodyHandle);
            ExtractNonconvexData(ref constraintContacts, ref prestep, ref impulses);
        }

        public void NonconvexTwoBody<TPrestep, TAccumulatedImpulses>(BodyHandle bodyHandleA, BodyHandle bodyHandleB, ref TPrestep prestep, ref TAccumulatedImpulses impulses)
            where TPrestep : struct, ITwoBodyNonconvexContactPrestep<TPrestep>
            where TAccumulatedImpulses : struct, INonconvexContactAccumulatedImpulses<TAccumulatedImpulses>
        {
            ref var constraintContacts = ref Constraints.Allocate(Pool);
            constraintContacts = new ExtractedManifold(Pool, bodyHandleA, bodyHandleB);
            ExtractNonconvexData(ref constraintContacts, ref prestep, ref impulses);
        }

        public void Reset()
        {
            for (int i = 0; i < Constraints.Count; ++i)
            {
                Constraints[i].Dispose(Pool);
            }
            Constraints.Count = 0;
        }

        public void Dispose()
        {
            Reset();
            Constraints.Dispose(Pool);
        }
    }

    /// <summary>
    /// Shows how to enumerate contact constraints in the solver using contact accessors and ISolverContactDataExtractor.
    /// </summary>
    public class SolverContactEnumerationDemo : Demo
    {
        BodyHandle sensorBodyHandle;
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 5, 25);
            camera.Yaw = 0;
            camera.Pitch = 0;

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));

            //Drop a pyramid on top of the sensor so there are more contacts to look at.
            var boxShape = new Box(1, 1, 1);
            var boxInertia = boxShape.ComputeInertia(1);
            var boxIndex = Simulation.Shapes.Add(boxShape);

            const int rowCount = 20;
            for (int rowIndex = 0; rowIndex < rowCount; ++rowIndex)
            {
                int columnCount = rowCount - rowIndex;
                for (int columnIndex = 0; columnIndex < columnCount; ++columnIndex)
                {
                    Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(
                        (-columnCount * 0.5f + columnIndex) * boxShape.Width,
                        (rowIndex + 0.5f) * boxShape.Height + 10, 0),
                        boxInertia, boxIndex, 0.01f));
                }
            }


            sensorBodyHandle = Simulation.Bodies.Add(BodyDescription.CreateConvexDynamic(new Vector3(0, 0, 1), 10, Simulation.Shapes, new Box(4, 2, 6)));

            //Put a mesh under the sensor so that nonconvex contacts are shown.
            const int planeWidth = 128;
            const int planeHeight = 128;
            DemoMeshHelper.CreateDeformedPlane(planeWidth, planeHeight,
                (int x, int y) =>
                {
                    return new Vector3(x - planeWidth / 2, 1 * MathF.Cos(x / 2f) * MathF.Sin(y / 2f), y - planeHeight / 2);
                }, new Vector3(2, 1, 2), BufferPool, out var planeMesh);
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -2, 0), QuaternionEx.CreateFromAxisAngle(new Vector3(0, 1, 0), MathF.PI / 2), Simulation.Shapes.Add(planeMesh)));
        }


        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            var sensorBody = Simulation.Bodies[sensorBodyHandle];
            var extractor = new SolverContactDataExtractor(BufferPool, sensorBody.Constraints.Count);
            //The basic idea behind the contact extractor is to submit it to a narrow phase contact accessor that is able to understand the solver's layout,
            //which will then call the contact extractor's relevant callbacks for the type of constraint encountered.
            //Here, we'll enumerate over all the constraints currently affecting the sensor body, attempting to extract contact data from each one.
            //If there are constraints that aren't contact constraints, they'll just get skipped.
            for (int i = 0; i < sensorBody.Constraints.Count; ++i)
            {
                Simulation.NarrowPhase.TryExtractSolverContactData(sensorBody.Constraints[i].ConnectingConstraintHandle, ref extractor);
            }
            //We now have extracted contact data. Let's analyze it!
            //For the purposes of the demo, we'll draw debug shapes at the contacts to represent the different properties.
            int sensorContactCount = 0;
            for (int manifoldIndex = 0; manifoldIndex < extractor.Constraints.Count; ++manifoldIndex)
            {
                ref var constraintContacts = ref extractor.Constraints[manifoldIndex];
                var bodyA = Simulation.Bodies[constraintContacts.BodyA];
                sensorContactCount += constraintContacts.Contacts.Count;
                for (int contactIndex = 0; contactIndex < constraintContacts.Contacts.Count; ++contactIndex)
                {
                    ref var contact = ref constraintContacts.Contacts[contactIndex];
                    var contactPosition = contact.OffsetA + bodyA.Pose.Position;
                    Matrix3x3 basisPose;
                    //We want to visualize both friction and penetration impulses, so a cylinder is a good choice- radius and length.
                    //The length will be oriented along the contact normal, while the radius will expand along the tangent directions.
                    BepuPhysics.Helpers.BuildOrthonormalBasis(contact.Normal, out basisPose.X, out basisPose.Z);
                    basisPose.Y = contact.Normal;
                    var baseLength = 0.2f;
                    var baseRadius = 0.1f;
                    //We'll make purely speculative contacts (negative depth) a fixed minimum size and a different color.
                    var contactVisualShape = contact.Depth < 0 ?
                        new Cylinder(baseRadius, baseLength) :
                        new Cylinder(baseRadius + MathF.Min(5, contact.FrictionImpulseMagnitude * 0.1f), baseLength + MathF.Min(5, contact.PenetrationImpulse * 0.3f));
                    RigidPose contactVisualPose;
                    contactVisualPose.Position = contactPosition + contact.Normal * contactVisualShape.HalfLength;
                    QuaternionEx.CreateFromRotationMatrix(basisPose, out contactVisualPose.Orientation);
                    renderer.Shapes.AddShape(contactVisualShape, Simulation.Shapes, contactVisualPose, contact.Depth < 0 ? new Vector3(0, 0, 1) : new Vector3(0, 1, 0));
                }
            }

            extractor.Dispose();

            var resolution = renderer.Surface.Resolution;
            renderer.TextBatcher.Write(text.Clear().Append("The solver stores data in an optimized array-of-structures-of-arrays format that makes it difficult to directly read."), new Vector2(16, resolution.Y - 96), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("This demo implements an ISolverContactDataExtractor that pulls data out of the solver and puts it into a simpler format."), new Vector2(16, resolution.Y - 80), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Pulling data this way can sometimes be more convenient than tracking contacts from INarrowPhaseCallbacks."), new Vector2(16, resolution.Y - 64), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Contacts associated with the large box are visualized. The size of a contact corresponds to the contact's force."), new Vector2(16, resolution.Y - 48), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Sensor manifold constraint count: ").Append(extractor.Constraints.Count).Append(", contact count: ").Append(sensorContactCount), new Vector2(16, resolution.Y - 20), 20, Vector3.One, font);
            base.Render(renderer, camera, input, text, font);
        }
    }
}
