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
        //No need for feature ids in 1 contact constraints. Just assume the old impulse can be reused regardless. Reasonably good guess.
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

}
