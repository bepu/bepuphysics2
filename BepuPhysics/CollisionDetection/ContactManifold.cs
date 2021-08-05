using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BepuPhysics.CollisionDetection
{
    /// <summary>
    /// Information about a single contact in a nonconvex collidable pair.
    /// Nonconvex pairs can have different surface bases at each contact point, since the contact surface is not guaranteed to be a plane.
    /// </summary>
    [StructLayout(LayoutKind.Explicit, Size = 32)]
    public struct NonconvexContact
    {
        /// <summary>
        /// Offset from the position of collidable A to the contact position in collidable A's local space.. 
        /// </summary>
        [FieldOffset(0)]
        public Vector3 LocalOffsetA;
        /// <summary>
        /// Penetration depth between the two collidables at this contact. Negative values represent separation.
        /// </summary>
        [FieldOffset(12)]
        public float Depth;
        /// <summary>
        /// Normal of the surface at the contact point in collidable B's local space, or world space if B is a static.
        /// </summary>
        [FieldOffset(16)]
        public Vector3 LocalNormalB;
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
        /// Offset from the position of collidable A to the contact position in collidable A's local space.. 
        /// </summary>
        [FieldOffset(0)]
        public Vector3 LocalOffsetA;
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
        /// Retrieves the feature id associated with a requested contact.
        /// </summary>
        /// <param name="contactIndex">Index of the contact to grab the feature id of.</param>
        /// <returns>Feature id of the requested contact.</returns>
        int GetFeatureId(int contactIndex);

        /// <summary>
        /// Retrieves a copy of a contact's data.
        /// </summary>
        /// <param name="contactIndex">Index of the contact to copy data from.</param>
        /// <param name="localOffsetA">Offset from collidable A's position to the contact position in collidable A's local space.</param>
        /// <param name="localNormalB">Normal of the contact surface at the requested contact in collidable B's local space, or world space if B is a static. Points from collidable B to collidable A.</param>
        /// <param name="depth">Penetration depth at the requested contact.</param>
        /// <param name="featureId">Feature id of the requested contact.
        /// Feature ids represent which parts of the collidables formed the contact and can be used to track unique contacts across frames.</param>
        void GetContact(int contactIndex, out Vector3 localOffsetA, out Vector3 localNormalB, out float depth, out int featureId);

        //Can't return refs to the this instance, but it's convenient to have ref returns for parameters and interfaces can't require static functions, so...
        /// <summary>
        /// Pulls a reference to a contact's depth.
        /// </summary>
        /// <param name="manifold">Manifold to pull a reference from.</param>
        /// <param name="contactIndex">Contact to pull data from.</param>
        /// <returns>Reference to a contact's depth.</returns>
        ref float GetDepth(ref TManifold manifold, int contactIndex);

        /// <summary>
        /// Pulls a reference to a contact's normal in collidable B's local space, or world space if B is a static. Points from collidable B to collidable A. For convex manifolds that share a normal, all contact indices will simply return a reference to the manifold-wide normal.
        /// </summary>
        /// <param name="manifold">Manifold to pull a reference from.</param>
        /// <param name="contactIndex">Contact to pull data from.</param>
        /// <returns>Reference to a contact's normal (or the manifold-wide normal in a convex manifold).</returns>
        ref Vector3 GetLocalNormalB(ref TManifold manifold, int contactIndex);

        /// <summary>
        /// Pulls a reference to a contact's offset in collidable A's local space.
        /// </summary>
        /// <param name="manifold">Manifold to pull a reference from.</param>
        /// <param name="contactIndex">Contact to pull data from.</param>
        /// <returns>Reference to a contact's offset.</returns>
        ref Vector3 GetLocalOffsetA(ref TManifold manifold, int contactIndex);

        /// <summary>
        /// Pulls a reference to a contact's feature id.
        /// </summary>
        /// <param name="manifold">Manifold to pull a reference from.</param>
        /// <param name="contactIndex">Contact to pull data from.</param>
        /// <returns>Reference to a contact's feature id.</returns>
        ref int GetFeatureId(ref TManifold manifold, int contactIndex);

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
        public NonconvexContact Contact0;
        [FieldOffset(48)]
        public NonconvexContact Contact1;
        [FieldOffset(80)]
        public NonconvexContact Contact2;
        [FieldOffset(112)]
        public NonconvexContact Contact3;

        readonly int IContactManifold<NonconvexContactManifold>.Count => Count;

        readonly bool IContactManifold<NonconvexContactManifold>.Convex => false;

        /// <summary>
        /// The maximum number of contacts that can exist within a nonconvex manifold.
        /// </summary>
        public const int MaximumContactCount = 4;

        [Conditional("DEBUG")]
        private readonly void ValidateIndex(int contactIndex)
        {
            Debug.Assert(contactIndex >= 0 && contactIndex < Count, "Contact index must be within the contact count.");
        }

        /// <summary>
        /// Retrieves a copy of a contact's data.
        /// </summary>
        /// <param name="contactIndex">Index of the contact to copy data from.</param>
        /// <param name="localOffsetA">Offset from collidable A's position to the contact position in collidable A's local space..</param>
        /// <param name="localNormalB">Normal of the contact surface at the requested contact in collidable B's local space, or world space if B is a static. Points from collidable B to collidable A.</param>
        /// <param name="depth">Penetration depth at the requested contact.</param>
        /// <param name="featureId">Feature id of the requested contact.
        /// Feature ids represent which parts of the collidables formed the contact and can be used to track unique contacts across frames.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetContact(int contactIndex, out Vector3 localOffsetA, out Vector3 localNormalB, out float depth, out int featureId)
        {
            ValidateIndex(contactIndex);
            ref var contact = ref Unsafe.Add(ref Contact0, contactIndex);
            localOffsetA = contact.LocalOffsetA;
            localNormalB = contact.LocalNormalB;
            depth = contact.Depth;
            featureId = contact.FeatureId;
        }
        /// <summary>
        /// Retrieves the feature id associated with a requested contact.
        /// </summary>
        /// <param name="contactIndex">Index of the contact to grab the feature id of.</param>
        /// <returns>Feature id of the requested contact.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int GetFeatureId(int contactIndex)
        {
            ValidateIndex(contactIndex);
            return Unsafe.Add(ref Contact0, contactIndex).FeatureId;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void FastRemoveAt(NonconvexContactManifold* manifold, int index)
        {
            --manifold->Count;
            if (index < manifold->Count)
            {
                var contacts = &manifold->Contact0;
                contacts[index] = contacts[manifold->Count];
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(NonconvexContactManifold* manifold, ref Vector3 localNormalB, ref ConvexContact convexContact)
        {
            Debug.Assert(manifold->Count < MaximumContactCount);
            ref var targetContact = ref (&manifold->Contact0)[manifold->Count++];
            targetContact.Depth = convexContact.Depth;
            targetContact.LocalOffsetA = convexContact.LocalOffsetA;
            targetContact.LocalNormalB = localNormalB;
            targetContact.FeatureId = convexContact.FeatureId;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref NonconvexContact Allocate(NonconvexContactManifold* manifold)
        {
            Debug.Assert(manifold->Count < MaximumContactCount);
            return ref (&manifold->Contact0)[manifold->Count++];
        }

        /// <summary>
        /// Pulls a reference to a contact's depth.
        /// </summary>
        /// <param name="manifold">Manifold to pull a reference from.</param>
        /// <param name="contactIndex">Contact to pull data from.</param>
        /// <returns>Reference to a contact's depth.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref float GetDepth(ref NonconvexContactManifold manifold, int contactIndex)
        {
            return ref Unsafe.Add(ref manifold.Contact0, contactIndex).Depth;
        }

        /// <summary>
        /// Pulls a reference to a contact's normal in collidable B's local space, or world space if B is a static. Points from collidable B to collidable A.
        /// </summary>
        /// <param name="manifold">Manifold to pull a reference from.</param>
        /// <param name="contactIndex">Contact to pull data from.</param>
        /// <returns>Reference to a contact's normal.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector3 GetLocalNormalB(ref NonconvexContactManifold manifold, int contactIndex)
        {
            return ref Unsafe.Add(ref manifold.Contact0, contactIndex).LocalNormalB;
        }


        /// <summary>
        /// Pulls a reference to a contact's offset in collidable A's local space.
        /// </summary>
        /// <param name="manifold">Manifold to pull a reference from.</param>
        /// <param name="contactIndex">Contact to pull data from.</param>
        /// <returns>Reference to a contact's offset.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector3 GetLocalOffsetA(ref NonconvexContactManifold manifold, int contactIndex)
        {
            return ref Unsafe.Add(ref manifold.Contact0, contactIndex).LocalOffsetA;
        }

        /// <summary>
        /// Pulls a reference to a contact's feature id.
        /// </summary>
        /// <param name="manifold">Manifold to pull a reference from.</param>
        /// <param name="contactIndex">Contact to pull data from.</param>
        /// <returns>Reference to a contact's feature id.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref int GetFeatureId(ref NonconvexContactManifold manifold, int contactIndex)
        {
            return ref Unsafe.Add(ref manifold.Contact0, contactIndex).FeatureId;
        }
    }

    /// <summary>
    /// Contains the data associated with a convex contact manifold.
    /// </summary>
    [StructLayout(LayoutKind.Explicit, Size = 96)]
    public unsafe struct ConvexContactManifold : IContactManifold<ConvexContactManifold>
    {
        /// <summary>
        /// Surface normal shared by all contacts, stored in collidable B's local space if it is a body, or world space if it is a static. Points from collidable B to collidable A.
        /// </summary>
        [FieldOffset(0)]
        public Vector3 LocalNormalB;

        [FieldOffset(12)]
        public int Count;

        [FieldOffset(16)]
        public ConvexContact Contact0;
        [FieldOffset(36)]
        public ConvexContact Contact1;
        [FieldOffset(56)]
        public ConvexContact Contact2;
        [FieldOffset(76)]
        public ConvexContact Contact3;

        readonly int IContactManifold<ConvexContactManifold>.Count => Count;

        readonly bool IContactManifold<ConvexContactManifold>.Convex => true;

        [Conditional("DEBUG")]
        private void ValidateIndex(int contactIndex)
        {
            Debug.Assert(contactIndex >= 0 && contactIndex < Count, "Contact index must be within the contact count.");
        }

        /// <summary>
        /// Retrieves the feature id associated with a requested contact.
        /// </summary>
        /// <param name="contactIndex">Index of the contact to grab the feature id of.</param>
        /// <returns>Feature id of the requested contact.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int GetFeatureId(int contactIndex)
        {
            ValidateIndex(contactIndex);
            return Unsafe.Add(ref Contact0, contactIndex).FeatureId;
        }

        /// <summary>
        /// Retrieves a copy of a contact's data.
        /// </summary>
        /// <param name="contactIndex">Index of the contact to copy data from.</param>
        /// <param name="localOffsetA">Offset from collidable A's position to the contact position in collidable A's local space.</param>
        /// <param name="localNormalB">Normal of the contact surface at the requested contact in collidable B's local space, or world space if B is a static. Points from collidable B to collidable A.</param>
        /// <param name="depth">Penetration depth at the requested contact.</param>
        /// <param name="featureId">Feature id of the requested contact.
        /// Feature ids represent which parts of the collidables formed the contact and can be used to track unique contacts across frames.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetContact(int contactIndex, out Vector3 localOffsetA, out Vector3 localNormalB, out float depth, out int featureId)
        {
            ValidateIndex(contactIndex);
            ref var contact = ref Unsafe.Add(ref Contact0, contactIndex);
            localOffsetA = contact.LocalOffsetA;
            localNormalB = LocalNormalB;
            depth = contact.Depth;
            featureId = contact.FeatureId;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void FastRemoveAt(ref ConvexContactManifold manifold, int index)
        {
            --manifold.Count;
            if (index < manifold.Count)
            {
                Unsafe.Add(ref manifold.Contact0, index) = Unsafe.Add(ref manifold.Contact0, manifold.Count);
            }
        }

        /// <summary>
        /// Pulls a reference to a contact's depth.
        /// </summary>
        /// <param name="manifold">Manifold to pull a reference from.</param>
        /// <param name="contactIndex">Contact to pull data from.</param>
        /// <returns>Reference to a contact's depth.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref float GetDepth(ref ConvexContactManifold manifold, int contactIndex)
        {
            return ref Unsafe.Add(ref manifold.Contact0, contactIndex).Depth;
        }

        /// <summary>
        /// Pulls a reference to a contact manifold's normal in collidable B's local space, or world space if B is a static. Points from collidable B to collidable A. Convex manifolds share a single normal across all contacts.
        /// </summary>
        /// <param name="manifold">Manifold to pull a reference from.</param>
        /// <param name="contactIndex">Contact to pull data from.</param>
        /// <returns>Reference to the contact manifold's normal.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector3 GetLocalNormalB(ref ConvexContactManifold manifold, int contactIndex)
        {
            return ref manifold.LocalNormalB;
        }


        /// <summary>
        /// Pulls a reference to a contact's offset in collidable A's local space.
        /// </summary>
        /// <param name="manifold">Manifold to pull a reference from.</param>
        /// <param name="contactIndex">Contact to pull data from.</param>
        /// <returns>Reference to a contact's offset.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector3 GetLocalOffsetA(ref ConvexContactManifold manifold, int contactIndex)
        {
            return ref Unsafe.Add(ref manifold.Contact0, contactIndex).LocalOffsetA;
        }

        /// <summary>
        /// Pulls a reference to a contact's feature id.
        /// </summary>
        /// <param name="manifold">Manifold to pull a reference from.</param>
        /// <param name="contactIndex">Contact to pull data from.</param>
        /// <returns>Reference to a contact's feature id.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref int GetFeatureId(ref ConvexContactManifold manifold, int contactIndex)
        {
            return ref Unsafe.Add(ref manifold.Contact0, contactIndex).FeatureId;
        }
    }

}