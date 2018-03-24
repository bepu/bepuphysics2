using System;
using System.Collections.Generic;
using System.Text;

namespace BepuPhysics.CollisionDetection
{
    public interface IPairCacheEntry
    {
        /// <summary>
        /// Gets the cache's type id. 
        /// Note that this is not the same as a constraint type id or other type ids; it only refers to the type of the caches for storage within the PairCache's structures.
        /// </summary>
        int CacheTypeId { get; }
    }
    public struct ConstraintCache1 : IPairCacheEntry
    {
        public int ConstraintHandle;
        public int FeatureId0;

        public int CacheTypeId => 0;
    }
    public struct ConstraintCache2 : IPairCacheEntry
    {
        public int ConstraintHandle;
        public int FeatureId0;
        public int FeatureId1;

        public int CacheTypeId => 1;
    }
    public struct ConstraintCache3 : IPairCacheEntry
    {
        public int ConstraintHandle;
        public int FeatureId0;
        public int FeatureId1;
        public int FeatureId2;

        public int CacheTypeId => 2;
    }
    public struct ConstraintCache4 : IPairCacheEntry
    {
        public int ConstraintHandle;
        public int FeatureId0;
        public int FeatureId1;
        public int FeatureId2;
        public int FeatureId3;

        public int CacheTypeId => 3;
    }

    public struct ConstraintCache5 : IPairCacheEntry
    {
        public int ConstraintHandle;
        public int FeatureId0;
        public int FeatureId1;
        public int FeatureId2;
        public int FeatureId3;
        public int FeatureId4;

        public int CacheTypeId => 4;
    }
    public struct ConstraintCache6 : IPairCacheEntry
    {
        public int ConstraintHandle;
        public int FeatureId0;
        public int FeatureId1;
        public int FeatureId2;
        public int FeatureId3;
        public int FeatureId4;
        public int FeatureId5;

        public int CacheTypeId => 5;
    }
    public struct ConstraintCache7 : IPairCacheEntry
    {
        public int ConstraintHandle;
        public int FeatureId0;
        public int FeatureId1;
        public int FeatureId2;
        public int FeatureId3;
        public int FeatureId4;
        public int FeatureId5;
        public int FeatureId6;

        public int CacheTypeId => 6;
    }
    public struct ConstraintCache8 : IPairCacheEntry
    {
        public int ConstraintHandle;
        public int FeatureId0;
        public int FeatureId1;
        public int FeatureId2;
        public int FeatureId3;
        public int FeatureId4;
        public int FeatureId5;
        public int FeatureId6;
        public int FeatureId7;

        public int CacheTypeId => 7;
    }

}
