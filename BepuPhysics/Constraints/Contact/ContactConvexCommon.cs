using BepuPhysics.CollisionDetection;
using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace BepuPhysics.Constraints.Contact
{

    /// <summary>
    /// Pulls direct references to the solver's constraint data.
    /// </summary>
    public interface IDirectContactDataExtractor
    {
        /// <summary>
        /// Gets whether the extractor wants to retrieve a reference to the constraint's prestep data.
        /// </summary>
        bool RequestPrestep { get; }
        /// <summary>
        /// Gets whether the extractor wants to retrieve a reference to the constraint's accumulated impulses.
        /// </summary>
        bool RequestAccumulatedImpulses { get; }
        /// <summary>
        /// Gets whether the extractor wants to retrieve the constraint's body references.
        /// </summary>
        bool RequestBodyReferences { get; }
        /// <summary>
        /// Provides a reference to the requested prestep data when the constraint was a convex one body constraint. Constraint data is in the first lane of the direct reference (slot 0 of vectors).
        /// </summary>
        /// <typeparam name="TPrestep">Type of the prestep data returned.</typeparam>
        /// <param name="prestep">Prestep data associated with the constraint.</param>
        void ConvexOneBodyPrestep<TPrestep>(ref TPrestep prestep) where TPrestep : struct, IConvexContactPrestep<TPrestep>;
        /// <summary>
        /// Provides a reference to the requested prestep data when the constraint was a convex two body constraint. Constraint data is in the first lane of the direct reference (slot 0 of vectors).
        /// </summary>
        /// <typeparam name="TPrestep">Type of the prestep data returned.</typeparam>
        /// <param name="prestep">Prestep data associated with the constraint.</param>
        void ConvexTwoBodyPrestep<TPrestep>(ref TPrestep prestep) where TPrestep : struct, ITwoBodyConvexContactPrestep<TPrestep>;
        /// <summary>
        /// Provides a reference to the requested prestep data when the constraint was a nonconvex one body constraint. Constraint data is in the first lane of the direct reference (slot 0 of vectors).
        /// </summary>
        /// <typeparam name="TPrestep">Type of the prestep data returned.</typeparam>
        /// <param name="prestep">Prestep data associated with the constraint.</param>
        void NonconvexOneBodyPrestep<TPrestep>(ref TPrestep prestep) where TPrestep : struct, INonconvexContactPrestep<TPrestep>;
        /// <summary>
        /// Provides a reference to the requested prestep data when the constraint was a nonconvex two body constraint. Constraint data is in the first lane of the direct reference (slot 0 of vectors).
        /// </summary>
        /// <typeparam name="TPrestep">Type of the prestep data returned.</typeparam>
        /// <param name="prestep">Prestep data associated with the constraint.</param>
        void NonconvexTwoBodyPrestep<TPrestep>(ref TPrestep prestep) where TPrestep : struct, ITwoBodyNonconvexContactPrestep<TPrestep>;
        /// <summary>
        /// Provides a reference to the constraint's accumulated impulses when the constraint was convex. Constraint data is in the first lane of the direct reference (slot 0 of vectors).
        /// </summary>
        /// <typeparam name="TAccumulatedImpulses">Type of the accumulated impulses data returned.</typeparam>
        /// <param name="impulses">Accumulated impulses associated with the constraint.</param>
        void ConvexAccumulatedImpulses<TAccumulatedImpulses>(ref TAccumulatedImpulses impulses) where TAccumulatedImpulses : struct, IConvexContactAccumulatedImpulses<TAccumulatedImpulses>;
        /// <summary>
        /// Provides a reference to the constraint's accumulated impulses when the constraint was nonconvex. Constraint data is in the first lane of the direct reference (slot 0 of vectors).
        /// </summary>
        /// <typeparam name="TAccumulatedImpulses">Type of the accumulated impulses data returned.</typeparam>
        /// <param name="impulses">Accumulated impulses associated with the constraint.</param>
        void NonconvexAccumulatedImpulses<TAccumulatedImpulses>(ref TAccumulatedImpulses impulses) where TAccumulatedImpulses : struct, INonconvexContactAccumulatedImpulses<TAccumulatedImpulses>;

        /// <summary>
        /// Provides the constraint's body reference if it was a one body constraint.
        /// </summary>
        /// <param name="bodyHandle">Body handle referenced by the constraint.</param>
        void OneBodyHandle(int bodyHandle);
        /// <summary>
        /// Provides the constraint's two body references if it was a two body constraint. If the constraint was in the active set, the body references are indices in the active set. If the constraint was in a sleeping set, the body references are body handles.
        /// </summary>
        /// <param name="bodyHandleA">First body handle referenced by the constraint.</param>
        /// <param name="bodyHandleB">Second body handle referenced by the constraint.</param>
        void TwoBodyHandles(int bodyHandleA, int bodyHandleB);
    }

    public struct ConvexContactWide
    {
        public Vector3Wide OffsetA;
        public Vector<float> Depth;
    }

    public struct MaterialPropertiesWide
    {
        public Vector<float> FrictionCoefficient;
        public SpringSettingsWide SpringSettings;
        public Vector<float> MaximumRecoveryVelocity;
    }

    public interface IContactPrestep<TPrestep> where TPrestep : struct, IContactPrestep<TPrestep>
    {
        ref MaterialPropertiesWide GetMaterialProperties(ref TPrestep prestep);
        int ContactCount { get; }
        int BodyCount { get; }
    }


    public interface IConvexContactPrestep<TPrestep> : IContactPrestep<TPrestep> where TPrestep : struct, IConvexContactPrestep<TPrestep>
    {
        ref Vector3Wide GetNormal(ref TPrestep prestep);
        ref ConvexContactWide GetContact(ref TPrestep prestep, int index);

    }

    public interface ITwoBodyConvexContactPrestep<TPrestep> : IConvexContactPrestep<TPrestep> where TPrestep : struct, ITwoBodyConvexContactPrestep<TPrestep>
    {
        ref Vector3Wide GetOffsetB(ref TPrestep prestep);
    }

    public interface IContactAccumulatedImpulses<TAccumulatedImpulses> where TAccumulatedImpulses : struct, IContactAccumulatedImpulses<TAccumulatedImpulses>
    {
        int ContactCount { get; }
    }

    public interface IConvexContactAccumulatedImpulses<TAccumulatedImpulses> : IContactAccumulatedImpulses<TAccumulatedImpulses> where TAccumulatedImpulses : struct, IConvexContactAccumulatedImpulses<TAccumulatedImpulses>
    {
        ref Vector2Wide GetTangentFriction(ref TAccumulatedImpulses impulses);
        ref Vector<float> GetTwistFriction(ref TAccumulatedImpulses impulses);
        ref Vector<float> GetPenetrationImpulseForContact(ref TAccumulatedImpulses impulses, int index);
    }

}
