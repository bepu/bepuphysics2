using BepuPhysics.CollisionDetection;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace BepuPhysics.Constraints.Contact
{
    public interface IConvexOneBodyContactConstraintDescription<TDescription> : IConstraintDescription<TDescription> 
        where TDescription : IConvexOneBodyContactConstraintDescription<TDescription>
    {
        void CopyManifoldWideProperties(ref Vector3 normal, ref PairMaterialProperties material);
    }
    public interface IConvexTwoBodyContactConstraintDescription<TDescription> : IConstraintDescription<TDescription> 
        where TDescription : IConvexTwoBodyContactConstraintDescription<TDescription>
    {
        void CopyManifoldWideProperties(ref Vector3 offsetB, ref Vector3 normal, ref PairMaterialProperties material);
    }

    public interface INonconvexOneBodyContactConstraintDescription<TDescription> : IConstraintDescription<TDescription> 
        where TDescription : INonconvexOneBodyContactConstraintDescription<TDescription>
    {
        void CopyManifoldWideProperties(ref PairMaterialProperties material);
    }
    public interface INonconvexTwoBodyContactConstraintDescription<TDescription> : IConstraintDescription<TDescription> 
        where TDescription : INonconvexTwoBodyContactConstraintDescription<TDescription>
    {
        void CopyManifoldWideProperties(ref Vector3 offsetB, ref PairMaterialProperties material);
    }
    public struct ConstraintContactData
    {
        public Vector3 OffsetA;
        public float PenetrationDepth;
    }
    public struct NonconvexConstraintContactData
    {
        public Vector3 OffsetA;
        public Vector3 Normal;
        public float PenetrationDepth;
    }

}
