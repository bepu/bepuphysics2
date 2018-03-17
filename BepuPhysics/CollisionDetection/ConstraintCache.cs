using System;
using System.Collections.Generic;
using System.Text;

namespace BepuPhysics.CollisionDetection
{
    public interface IPairCacheEntry
    {
        int TypeId { get; }
    }
    public struct ConstraintCache1 : IPairCacheEntry
    {
        public int ConstraintHandle;
        public int FeatureId0;

        public int TypeId => 0;
    }
    public struct ConstraintCache2 : IPairCacheEntry
    {
        public int ConstraintHandle;
        public int FeatureId0;
        public int FeatureId1;

        public int TypeId => 1;
    }
    public struct ConstraintCache3 : IPairCacheEntry
    {
        public int ConstraintHandle;
        public int FeatureId0;
        public int FeatureId1;
        public int FeatureId2;

        public int TypeId => 2;
    }
    public struct ConstraintCache4 : IPairCacheEntry
    {
        public int ConstraintHandle;
        public int FeatureId0;
        public int FeatureId1;
        public int FeatureId2;
        public int FeatureId3;

        public int TypeId => 3;
    }

    public struct ConstraintCache5 : IPairCacheEntry
    {
        public int ConstraintHandle;
        public int FeatureId0;
        public int FeatureId1;
        public int FeatureId2;
        public int FeatureId3;
        public int FeatureId4;

        public int TypeId => 4;
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

        public int TypeId => 5;
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

        public int TypeId => 6;
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

        public int TypeId => 7;
    }

}
