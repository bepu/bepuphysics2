using BepuPhysics.Constraints;
using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.CollisionDetection
{
    /// <summary>
    /// Data associated with a single contact in a contact manifold.
    /// </summary>
    public struct ContactData
    {
        /// <summary>
        /// Offset from the center of body A to the contact.
        /// </summary>
        public Vector3 OffsetA;
        /// <summary>
        /// Normal of the contact surface.
        /// </summary>
        public Vector3 Normal;
        /// <summary>
        /// Penetration depth at the contact location.
        /// </summary>
        public float Depth;
    }


    /// <summary>
    /// Data shared across an entire contact manifold.
    /// </summary>
    public struct ManifoldData
    {
        public float FrictionCoefficient;
        public SpringSettings SpringSettings;
        public float MaximumRecoveryVelocity;
    }

    /// <summary>
    /// Raw data bundle shared across an entire contact manifold.
    /// </summary>
    public struct ManifoldDataWide
    {
        public Vector<float> FrictionCoefficient;
        public SpringSettingsWide SpringSettings;
        public Vector<float> MaximumRecoveryVelocity;
    }


    /// <summary>
    /// Accesses existing contact data in the solver. 
    /// Movement of memory in the solver may invalidate this view and result in access violations; do not continue to use an instance if anything changes in the solver during its lifespan.
    /// </summary>
    public unsafe struct UnsafeManifoldViewer
    {
        byte* offsetABase;
        byte* normalBase;
        byte* depthBase;
        byte* common;
        int offsetDepthStride;
        int normalStride;

        int contactCount;
        bool convex;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal UnsafeManifoldViewer(int contactCount, bool convex, void* common, void* offsetABase, int offsetDepthStride, void* normalBase, int normalStride, void* depthBase)
        {
            this.offsetABase = (byte*)offsetABase;
            this.normalBase = (byte*)normalBase;
            this.depthBase = (byte*)depthBase;
            this.common = (byte*)common;
            this.offsetDepthStride = offsetDepthStride;
            this.normalStride = normalStride;

            this.contactCount = contactCount;
            this.convex = convex;
        }

        /// <summary>
        /// Gets the number of contacts in the manifold.
        /// </summary>
        public int ContactCount => contactCount;

        /// <summary>
        /// Gets whether this manifold is convex. Convex manifolds share the same normal for every contact; nonconvex manifolds do not.
        /// </summary>
        public bool Convex => convex;

        /// <summary>
        /// Gets or sets the contact data at the given index in the manifold.
        /// </summary>
        /// <param name="index">Index of the contact to access.</param>
        /// <returns>Contact data at the given index in the manifold.</returns>
        public ContactData this[int index]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                Debug.Assert(index >= 0 && index < contactCount, "Index must be within the contact count.");
                ContactData toReturn;
                GetOffsetA(index, out toReturn.OffsetA);
                GetNormal(index, out toReturn.Normal);
                toReturn.Depth = GetDepth(index);
                return toReturn;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set
            {
                Debug.Assert(index >= 0 && index < contactCount, "Index must be within the contact count.");
                SetOffsetA(index, value.OffsetA);
                SetNormal(index, value.Normal);
                GetDepth(index) = value.Depth;
            }
        }

        /// <summary>
        /// Gets a reference to the offset bundle in the solver prestep data.
        /// </summary>
        /// <param name="index">Contact index to access.</param>
        /// <returns>Reference to a contact's offset bundle in the solver prestep data.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector3Wide GetOffsetAWide(int index)
        {
            Debug.Assert(index >= 0 && index < contactCount, "Index must be within the contact count.");
            return ref Unsafe.AsRef<Vector3Wide>(offsetABase + offsetDepthStride * index);
        }

        /// <summary>
        /// Gets the offset from body A to the contact for the given contact index.
        /// </summary>
        /// <param name="index">Index of the contact to access.</param>
        /// <param name="offsetA">Offset from body A to the contact location.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetOffsetA(int index, out Vector3 offsetA)
        {
            Vector3Wide.ReadFirst(GetOffsetAWide(index), out offsetA);
        }

        /// <summary>
        /// Sets the offset from body A to the contact for the given contact index.
        /// </summary>
        /// <param name="index">Index of the contact to access.</param>
        /// <param name="offsetA">New offset from body A to the contact location.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetOffsetA(int index, in Vector3 offsetA)
        {
            Vector3Wide.WriteFirst(offsetA, ref GetOffsetAWide(index));
        }

        /// <summary>
        /// Gets a reference to the normal bundle in the solver prestep data.
        /// </summary>
        /// <param name="index">Contact index to access.</param>
        /// <returns>Reference to a contact's normal bundle in the solver prestep data.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector3Wide GetNormalWide(int index)
        {
            Debug.Assert(index >= 0 && index < contactCount, "Index must be within the contact count.");
            return ref Unsafe.AsRef<Vector3Wide>(normalBase + normalStride * index);
        }

        /// <summary>
        /// Gets the normal for the given contact index.
        /// </summary>
        /// <param name="index">Index of the contact to access.</param>
        /// <param name="normal">Normal for the accessed contact.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetNormal(int index, out Vector3 normal)
        {
            Vector3Wide.ReadFirst(GetNormalWide(index), out normal);
        }

        /// <summary>
        /// Sets the normal for the given contact index.
        /// </summary>
        /// <param name="index">Index of the contact to access.</param>
        /// <param name="normal">New normal for the accessed contact.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetNormal(int index, in Vector3 normal)
        {
            Vector3Wide.WriteFirst(normal, ref GetNormalWide(index));
        }

        /// <summary>
        /// Gets a reference to the depth in the solver prestep data.
        /// </summary>
        /// <param name="index">Contact index to access.</param>
        /// <returns>Reference to a contact's depth in the solver prestep data.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref float GetDepth(int index)
        {
            Debug.Assert(index >= 0 && index < contactCount, "Index must be within the contact count.");
            return ref Unsafe.AsRef<float>(depthBase + offsetDepthStride * index);
        }

        /// <summary>
        /// Gets a reference to the depth bundle in the solver prestep data.
        /// </summary>
        /// <returns>Reference to a contact's depth bundle in the solver prestep data.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ManifoldDataWide GetManifoldDataWide()
        {
            //Note that this assumes the memory layout matches the ManifoldDataWide struct.
            return ref Unsafe.AsRef<ManifoldDataWide>(common);
        }

        /// <summary>
        /// Gets the data shared across the entire manifold.
        /// </summary>
        /// <param name="depth">Depth for the accessed contact.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetManifoldData(out ManifoldData manifoldData)
        {
            ref var wide = ref GetManifoldDataWide();
            manifoldData.FrictionCoefficient = wide.FrictionCoefficient[0];
            SpringSettingsWide.ReadFirst(wide.SpringSettings, out manifoldData.SpringSettings);
            manifoldData.MaximumRecoveryVelocity = wide.MaximumRecoveryVelocity[0];
        }

        /// <summary>
        /// Sets the data shared across the entire manifold.
        /// </summary>
        /// <param name="manifoldData">New manifold data.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetManifoldData(in ManifoldData manifoldData)
        {
            ref var wide = ref GetManifoldDataWide();
            GatherScatter.GetFirst(ref wide.FrictionCoefficient) = manifoldData.FrictionCoefficient;
            SpringSettingsWide.WriteFirst(manifoldData.SpringSettings, ref wide.SpringSettings);
            GatherScatter.GetFirst(ref wide.MaximumRecoveryVelocity) = manifoldData.MaximumRecoveryVelocity;
        }

        /// <summary>
        /// Gets a reference to the friction coefficient for the manifold.
        /// </summary>
        public ref float FrictionCoefficient
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return ref GatherScatter.GetFirst(ref GetManifoldDataWide().FrictionCoefficient); }
        }

        /// <summary>
        /// Gets a reference to the maximum recovery velocity for the manifold.
        /// </summary>
        public ref float MaximumRecoveryVelocity
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return ref GatherScatter.GetFirst(ref GetManifoldDataWide().MaximumRecoveryVelocity); }
        }

        /// <summary>
        /// Gets the spring settings for the manifold.
        /// </summary>
        /// <param name="springSettings">Spring settings for the manifold.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetSpringSettings(out SpringSettings springSettings)
        {
            SpringSettingsWide.ReadFirst(GetManifoldDataWide().SpringSettings, out springSettings);
        }

        /// <summary>
        /// Sets the spring settings for the manifold.
        /// </summary>
        /// <param name="springSettings">Spring settings for the manifold.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetSpringSettings(in SpringSettings springSettings)
        {
            SpringSettingsWide.WriteFirst(springSettings, ref GetManifoldDataWide().SpringSettings);
        }

    }
}
