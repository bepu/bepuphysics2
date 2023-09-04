using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BepuPhysics.CollisionDetection
{
    /// <summary>
    /// Information about a single contact.
    /// </summary>
    /// <remarks>This type contains a field for the normal; it can be used to represent contacts within nonconvex contact manifolds or convex manifolds.</remarks>
    [StructLayout(LayoutKind.Explicit, Size = 32)]
    public struct Contact
    {
        /// <summary>
        /// Offset from the position of collidable A to the contact position. 
        /// </summary>
        [FieldOffset(0)]
        public Vector3 Offset;
        /// <summary>
        /// Penetration depth between the two collidables at this contact. Negative values represent separation.
        /// </summary>
        [FieldOffset(12)]
        public float Depth;
        /// <summary>
        /// Surface basis of the contact. If transformed into a rotation matrix, X and Z represent tangent directions and Y represents the contact normal. Points from collidable B to collidable A.
        /// </summary>
        [FieldOffset(16)]
        public Vector3 Normal;
        /// <summary>
        /// Id of the features involved in the collision that generated this contact. If a contact has the same feature id as in a previous frame, it is an indication that the
        /// same parts of the shape contributed to its creation. This is useful for carrying information from frame to frame.
        /// </summary>
        [FieldOffset(28)]
        public int FeatureId;
    }
    /// <summary>
    /// Information about a single contact in a convex collidable pair. Convex collidable pairs share one surface basis across the manifold, since the contact surface is guaranteed to be a plane.
    /// </summary>    
    [StructLayout(LayoutKind.Explicit, Size = 20)]
    public struct ConvexContact
    {
        /// <summary>
        /// Offset from the position of collidable A to the contact position. 
        /// </summary>
        [FieldOffset(0)]
        public Vector3 Offset;
        /// <summary>
        /// Penetration depth between the two collidables at this contact. Negative values represent separation.
        /// </summary>
        [FieldOffset(12)]
        public float Depth;
        /// <summary>
        /// Id of the features involved in the collision that generated this contact. If a contact has the same feature id as in a previous frame, it is an indication that the
        /// same parts of the shape contributed to its creation. This is useful for carrying information from frame to frame.
        /// </summary>
        [FieldOffset(16)]
        public int FeatureId;
    }

    public interface IContactManifold<TManifold> where TManifold : struct, IContactManifold<TManifold>
    {
        /// <summary>
        /// Gets the number of contacts in the manifold.
        /// </summary>
        int Count { get; }

        /// <summary>
        /// Gets whether the contact manifold was created by a pair of convex objects or not. True if convex, false if nonconvex.
        /// </summary>
        bool Convex { get; }

        /// <summary>
        /// Gets or sets the contact at the given index in the manifold.
        /// </summary>
        /// <param name="contactIndex">Index of the contact to get or set.</param>
        /// <returns>Contact at the specified index.</returns>
        /// <remarks>Note that contact normals are shared across a <see cref="ConvexContactManifold"/>. Setting one contact in a convex manifold will change the entire convex manifold's normal.</remarks>
        public Contact this[int contactIndex] { get; set; }

        /// <summary>
        /// Gets the feature id associated with a requested contact.
        /// </summary>
        /// <param name="contactIndex">Index of the contact to grab the feature id of.</param>
        /// <returns>Feature id of the requested contact.</returns>
        int GetFeatureId(int contactIndex);

        /// <summary>
        /// Gets the depth associated with a requested contact.
        /// </summary>
        /// <param name="contactIndex">Index of the contact to grab the depth of.</param>
        /// <returns>Depth of the requested contact.</returns>
        float GetDepth(int contactIndex);

        /// <summary>
        /// Gets a contact's normal.
        /// </summary>
        /// <param name="contactIndex">Index of the contact to grab the normal of.</param>
        /// <returns>Normal of the requested contact.</returns>
        /// <remarks>Points from collidable B to collidable A. In convex manifolds, all contacts share a normal and will return the same value.</remarks>
        Vector3 GetNormal(int contactIndex);

        /// <summary>
        /// Gets the offset from collidable A to the requested contact.
        /// </summary>
        /// <param name="contactIndex">Index of the contact to grab the offset of.</param>
        /// <returns>Offset to a contact's offset.</returns>
        Vector3 GetOffset(int contactIndex);

        /// <summary>
        /// Gets a copy of a contact's data.
        /// </summary>
        /// <param name="contactIndex">Index of the contact to copy data from.</param>
        /// <param name="offset">Offset from the first collidable's position to the contact position.</param>
        /// <param name="normal">Normal of the contact surface at the requested contact. Points from collidable B to collidable A.</param>
        /// <param name="depth">Penetration depth at the requested contact.</param>
        /// <param name="featureId">Feature id of the requested contact.
        /// Feature ids represent which parts of the collidables formed the contact and can be used to track unique contacts across frames.</param>
        void GetContact(int contactIndex, out Vector3 offset, out Vector3 normal, out float depth, out int featureId);

        /// <summary>
        /// Gets a copy of a contact's data.
        /// </summary>
        /// <param name="contactIndex">Index of the contact to copy data from.</param>
        /// <param name="contactData">Data associated with the contact.</param>
        void GetContact(int contactIndex, out Contact contactData);

        /// <summary>
        /// Gets a reference to a contact's depth.
        /// </summary>
        /// <param name="manifold">Manifold to pull a reference from.</param>
        /// <param name="contactIndex">Contact to pull data from.</param>
        /// <returns>Reference to a contact's depth.</returns>
        static abstract ref float GetDepthReference(ref TManifold manifold, int contactIndex);

        /// <summary>
        /// Gets a reference to a contact's normal. Points from collidable B to collidable A. For convex manifolds that share a normal, all contact indices will simply return a reference to the manifold-wide normal.
        /// </summary>
        /// <param name="manifold">Manifold to pull a reference from.</param>
        /// <param name="contactIndex">Contact to pull data from.</param>
        /// <returns>Reference to a contact's normal (or the manifold-wide normal in a convex manifold).</returns>
        static abstract ref Vector3 GetNormalReference(ref TManifold manifold, int contactIndex);

        /// <summary>
        /// Gets a reference to the offset from collidable A to the requested contact.
        /// </summary>
        /// <param name="manifold">Manifold to pull a reference from.</param>
        /// <param name="contactIndex">Contact to pull data from.</param>
        /// <returns>Reference to a contact's offset.</returns>
        static abstract ref Vector3 GetOffsetReference(ref TManifold manifold, int contactIndex);

        /// <summary>
        /// Gets a reference to a contact's feature id.
        /// </summary>
        /// <param name="manifold">Manifold to pull a reference from.</param>
        /// <param name="contactIndex">Contact to pull data from.</param>
        /// <returns>Reference to a contact's feature id.</returns>
        static abstract ref int GetFeatureIdReference(ref TManifold manifold, int contactIndex);

        /// <summary>
        /// Gets a reference to a nonconvex manifold's contact.
        /// </summary>
        /// <param name="manifold">Manifold to pull a reference from.</param>
        /// <param name="contactIndex">Contact to pull data from.</param>
        /// <returns>Reference to the requested contact.</returns>
        /// <remarks>This is a helper that avoids manual casting. If the manifold is not a <see cref="NonconvexContactManifold"/>, the function will throw an <see cref="NotSupportedException"/>.</remarks>
        static abstract ref Contact GetNonconvexContactReference(ref TManifold manifold, int contactIndex);

        /// <summary>
        /// Gets a reference to a convex manifold's contact.
        /// </summary>
        /// <param name="manifold">Manifold to pull a reference from.</param>
        /// <param name="contactIndex">Contact to pull data from.</param>
        /// <returns>Reference to the requested contact.</returns>
        /// <remarks>This is a helper that avoids manual casting. If the manifold is not a <see cref="ConvexContactManifold"/>, the function will throw an <see cref="NotSupportedException"/>.</remarks>
        static abstract ref ConvexContact GetConvexContactReference(ref TManifold manifold, int contactIndex);
    }

    //TODO: We could use specialized storage types for things like continuations if L2 can't actually hold it all. Seems unlikely, but it's not that hard if required.

    /// <summary>
    /// Contains the data associated with a nonconvex contact manifold.
    /// </summary>
    [StructLayout(LayoutKind.Explicit, Size = 144)]
    public unsafe struct NonconvexContactManifold : IContactManifold<NonconvexContactManifold>
    {
        /// <summary>
        /// Offset from collidable A to collidable B.
        /// </summary>
        [FieldOffset(0)]
        public Vector3 OffsetB;
        [FieldOffset(12)]
        public int Count;

        [FieldOffset(16)]
        public Contact Contact0;
        [FieldOffset(48)]
        public Contact Contact1;
        [FieldOffset(80)]
        public Contact Contact2;
        [FieldOffset(112)]
        public Contact Contact3;

        readonly int IContactManifold<NonconvexContactManifold>.Count => Count;

        readonly bool IContactManifold<NonconvexContactManifold>.Convex => false;

        public Contact this[int contactIndex]
        {
            get
            {
                ValidateIndex(contactIndex);
                return Unsafe.Add(ref Contact0, contactIndex);
            }
            set
            {
                ValidateIndex(contactIndex);
                Unsafe.Add(ref Contact0, contactIndex) = value;
            }
        }

        /// <summary>
        /// The maximum number of contacts that can exist within a nonconvex manifold.
        /// </summary>
        public const int MaximumContactCount = 4;

        [Conditional("DEBUG")]
        private readonly void ValidateIndex(int contactIndex)
        {
            Debug.Assert(contactIndex >= 0 && contactIndex < Count, "Contact index must be within the contact count.");
        }

        public void GetContact(int contactIndex, out Vector3 offset, out Vector3 normal, out float depth, out int featureId)
        {
            ValidateIndex(contactIndex);
            ref var contact = ref Unsafe.Add(ref Contact0, contactIndex);
            offset = contact.Offset;
            normal = contact.Normal;
            depth = contact.Depth;
            featureId = contact.FeatureId;
        }

        public void GetContact(int contactIndex, out Contact contactData)
        {
            ValidateIndex(contactIndex);
            contactData = Unsafe.Add(ref Contact0, contactIndex);
        }


        public float GetDepth(int contactIndex)
        {
            ValidateIndex(contactIndex);
            return Unsafe.Add(ref Contact0, contactIndex).Depth;
        }

        public Vector3 GetNormal(int contactIndex)
        {
            ValidateIndex(contactIndex);
            return Unsafe.Add(ref Contact0, contactIndex).Normal;
        }

        public Vector3 GetOffset(int contactIndex)
        {
            ValidateIndex(contactIndex);
            return Unsafe.Add(ref Contact0, contactIndex).Offset;
        }
        public int GetFeatureId(int contactIndex)
        {
            ValidateIndex(contactIndex);
            return Unsafe.Add(ref Contact0, contactIndex).FeatureId;
        }


        public static ref float GetDepthReference(ref NonconvexContactManifold manifold, int contactIndex)
        {
            manifold.ValidateIndex(contactIndex);
            return ref Unsafe.Add(ref manifold.Contact0, contactIndex).Depth;
        }

        public static ref Vector3 GetNormalReference(ref NonconvexContactManifold manifold, int contactIndex)
        {
            manifold.ValidateIndex(contactIndex);
            return ref Unsafe.Add(ref manifold.Contact0, contactIndex).Normal;
        }

        public static ref Vector3 GetOffsetReference(ref NonconvexContactManifold manifold, int contactIndex)
        {
            manifold.ValidateIndex(contactIndex);
            return ref Unsafe.Add(ref manifold.Contact0, contactIndex).Offset;
        }

        public static ref int GetFeatureIdReference(ref NonconvexContactManifold manifold, int contactIndex)
        {
            manifold.ValidateIndex(contactIndex);
            return ref Unsafe.Add(ref manifold.Contact0, contactIndex).FeatureId;
        }

        public static ref Contact GetNonconvexContactReference(ref NonconvexContactManifold manifold, int contactIndex)
        {
            manifold.ValidateIndex(contactIndex);
            return ref Unsafe.Add(ref manifold.Contact0, contactIndex);
        }

        public static ref ConvexContact GetConvexContactReference(ref NonconvexContactManifold manifold, int contactIndex)
        {
            throw new NotSupportedException("This is a NonconvexContactManifold; use GetNonconvexContactReference instead.");
        }


        public static void FastRemoveAt(NonconvexContactManifold* manifold, int index)
        {
            --manifold->Count;
            if (index < manifold->Count)
            {
                var contacts = &manifold->Contact0;
                contacts[index] = contacts[manifold->Count];
            }
        }

        public static void Add(NonconvexContactManifold* manifold, ref Vector3 normal, ref ConvexContact convexContact)
        {
            Debug.Assert(manifold->Count < MaximumContactCount);
            ref var targetContact = ref (&manifold->Contact0)[manifold->Count++];
            targetContact.Depth = convexContact.Depth;
            targetContact.Offset = convexContact.Offset;
            targetContact.Normal = normal;
            targetContact.FeatureId = convexContact.FeatureId;
        }
        public static ref Contact Allocate(NonconvexContactManifold* manifold)
        {
            Debug.Assert(manifold->Count < MaximumContactCount);
            return ref (&manifold->Contact0)[manifold->Count++];
        }
    }

    /// <summary>
    /// Contains the data associated with a convex contact manifold.
    /// </summary>
    [StructLayout(LayoutKind.Explicit, Size = 108)]
    public struct ConvexContactManifold : IContactManifold<ConvexContactManifold>
    {
        /// <summary>
        /// Offset from collidable A to collidable B.
        /// </summary>
        [FieldOffset(0)]
        public Vector3 OffsetB;
        [FieldOffset(12)]
        public int Count;

        /// <summary>
        /// Surface normal shared by all contacts. Points from collidable B to collidable A.
        /// </summary>
        [FieldOffset(16)]
        public Vector3 Normal;

        [FieldOffset(28)]
        public ConvexContact Contact0;
        [FieldOffset(48)]
        public ConvexContact Contact1;
        [FieldOffset(68)]
        public ConvexContact Contact2;
        [FieldOffset(88)]
        public ConvexContact Contact3;

        readonly int IContactManifold<ConvexContactManifold>.Count => Count;

        readonly bool IContactManifold<ConvexContactManifold>.Convex => true;

        public Contact this[int contactIndex]
        {
            get
            {
                GetContact(contactIndex, out var contact);
                return contact;
            }
            set
            {
                ValidateIndex(contactIndex);
                ref var target = ref Unsafe.Add(ref Contact0, contactIndex);
                target.Offset = value.Offset;
                Normal = value.Normal;
                target.Depth = value.Depth;
                target.FeatureId = value.FeatureId;
            }
        }

        [Conditional("DEBUG")]
        private void ValidateIndex(int contactIndex)
        {
            Debug.Assert(contactIndex >= 0 && contactIndex < Count, "Contact index must be within the contact count.");
        }

        /// <summary>
        /// Retrieves a copy of a contact's data.
        /// </summary>
        /// <param name="contactIndex">Index of the contact to copy data from.</param>
        /// <param name="offset">Offset from the first collidable's position to the contact position.</param>
        /// <param name="normal">Normal of the contact surface at the requested contact. Points from collidable B to collidable A.</param>
        /// <param name="depth">Penetration depth at the requested contact.</param>
        /// <param name="featureId">Feature id of the requested contact.
        /// Feature ids represent which parts of the collidables formed the contact and can be used to track unique contacts across frames.</param>
        public void GetContact(int contactIndex, out Vector3 offset, out Vector3 normal, out float depth, out int featureId)
        {
            ValidateIndex(contactIndex);
            ref var contact = ref Unsafe.Add(ref Contact0, contactIndex);
            offset = contact.Offset;
            normal = Normal;
            depth = contact.Depth;
            featureId = contact.FeatureId;
        }
        public void GetContact(int contactIndex, out Contact contactData)
        {
            ValidateIndex(contactIndex);
            ref var contact = ref Unsafe.Add(ref Contact0, contactIndex);
            contactData.Offset = contact.Offset;
            contactData.Normal = Normal;
            contactData.Depth = contact.Depth;
            contactData.FeatureId = contact.FeatureId;

        }
        public float GetDepth(int contactIndex)
        {
            ValidateIndex(contactIndex);
            return Unsafe.Add(ref Contact0, contactIndex).Depth;
        }

        public Vector3 GetNormal(int contactIndex)
        {
            ValidateIndex(contactIndex);
            return Normal;
        }

        public Vector3 GetOffset(int contactIndex)
        {
            ValidateIndex(contactIndex);
            return Unsafe.Add(ref Contact0, contactIndex).Offset;
        }
        public int GetFeatureId(int contactIndex)
        {
            ValidateIndex(contactIndex);
            return Unsafe.Add(ref Contact0, contactIndex).FeatureId;
        }
        public static void FastRemoveAt(ref ConvexContactManifold manifold, int index)
        {
            --manifold.Count;
            if (index < manifold.Count)
            {
                Unsafe.Add(ref manifold.Contact0, index) = Unsafe.Add(ref manifold.Contact0, manifold.Count);
            }
        }

        public static ref float GetDepthReference(ref ConvexContactManifold manifold, int contactIndex)
        {
            manifold.ValidateIndex(contactIndex);
            return ref Unsafe.Add(ref manifold.Contact0, contactIndex).Depth;
        }

        public static ref Vector3 GetNormalReference(ref ConvexContactManifold manifold, int contactIndex)
        {
            manifold.ValidateIndex(contactIndex);
            return ref manifold.Normal;
        }

        public static ref Vector3 GetOffsetReference(ref ConvexContactManifold manifold, int contactIndex)
        {
            manifold.ValidateIndex(contactIndex);
            return ref Unsafe.Add(ref manifold.Contact0, contactIndex).Offset;
        }

        public static ref int GetFeatureIdReference(ref ConvexContactManifold manifold, int contactIndex)
        {
            manifold.ValidateIndex(contactIndex);
            return ref Unsafe.Add(ref manifold.Contact0, contactIndex).FeatureId;
        }

        public static ref Contact GetNonconvexContactReference(ref ConvexContactManifold manifold, int contactIndex)
        {
            throw new NotImplementedException();
        }

        public static ref ConvexContact GetConvexContactReference(ref ConvexContactManifold manifold, int contactIndex)
        {
            manifold.ValidateIndex(contactIndex);
            return ref Unsafe.Add(ref manifold.Contact0, contactIndex);
        }
    }

}