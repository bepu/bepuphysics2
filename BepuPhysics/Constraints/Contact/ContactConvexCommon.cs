using BepuPhysics.CollisionDetection;
using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace BepuPhysics.Constraints.Contact
{
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
        static abstract ref MaterialPropertiesWide GetMaterialProperties(ref TPrestep prestep);
        static abstract int ContactCount { get; }
        static abstract int BodyCount { get; }
    }


    public interface IConvexContactPrestep<TPrestep> : IContactPrestep<TPrestep> where TPrestep : struct, IConvexContactPrestep<TPrestep>
    {
        static abstract ref Vector3Wide GetNormal(ref TPrestep prestep);
        static abstract ref ConvexContactWide GetContact(ref TPrestep prestep, int index);

    }

    public interface ITwoBodyConvexContactPrestep<TPrestep> : IConvexContactPrestep<TPrestep> where TPrestep : struct, ITwoBodyConvexContactPrestep<TPrestep>
    {
        static abstract ref Vector3Wide GetOffsetB(ref TPrestep prestep);
    }

    public interface IContactAccumulatedImpulses<TAccumulatedImpulses> where TAccumulatedImpulses : struct, IContactAccumulatedImpulses<TAccumulatedImpulses>
    {
        static abstract int ContactCount { get; }
    }

    public interface IConvexContactAccumulatedImpulses<TAccumulatedImpulses> : IContactAccumulatedImpulses<TAccumulatedImpulses> where TAccumulatedImpulses : struct, IConvexContactAccumulatedImpulses<TAccumulatedImpulses>
    {
        static abstract ref Vector2Wide GetTangentFriction(ref TAccumulatedImpulses impulses);
        static abstract ref Vector<float> GetTwistFriction(ref TAccumulatedImpulses impulses);
        static abstract ref Vector<float> GetPenetrationImpulseForContact(ref TAccumulatedImpulses impulses, int index);
    }

}
