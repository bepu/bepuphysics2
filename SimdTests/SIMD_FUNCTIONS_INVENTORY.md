# BepuPhysics2 SIMD Functions Inventory

Complete inventory of SIMD-accelerated functions in BepuPhysics2, identifying optimization opportunities.

**Total Functions Found**: 25+
**Functions Missing ARM64 Support**: 13+
**Functions Supporting AVX512**: 1

---

## 1. Mathematical Operations

**File**: `BepuUtilities/MathHelper.cs`

### FastReciprocal
- **Line**: ~380
- **Signature**: `Vector<float> FastReciprocal(Vector<float> v)`
- **Current Support**: Vector256 (AVX), Vector128 (SSE)
- **Missing**: AVX512 (Vector512), ARM64 (AdvSimd)
- **Fallback**: Software division (Vector<float>.One / v)
- **TODO Comment**: "Arm!" at line 394
- **Usage**: Constraint solving (CenterDistanceLimit.cs, CenterDistanceConstraint.cs)

### FastReciprocalSquareRoot
- **Line**: ~397
- **Signature**: `Vector<float> FastReciprocalSquareRoot(Vector<float> v)`
- **Current Support**: Vector256 (AVX), Vector128 (SSE)
- **Missing**: AVX512 (Vector512), ARM64 (AdvSimd)
- **Fallback**: Software (Vector<float>.One / Vector.SquareRoot(v))
- **TODO Comment**: "Arm!" at line 411
- **Usage**: Constraint solving

### Cos, Sin, Acos
- **Lines**: Various
- **Current Support**: Platform-independent using System.Numerics.Vector
- **Status**: ✅ Good cross-platform support

---

## 2. Bundle Indexing Operations

**File**: `BepuUtilities/BundleIndexing.cs`

### CreateTrailingMaskForCountInBundle
- **Line**: ~62
- **Signature**: `Vector<int> CreateTrailingMaskForCountInBundle(int countInBundle)`
- **Current Support**:
  - Vector256 (8-wide): Uses `Avx.CompareLessThanOrEqual`
  - Vector128 (4-wide): Uses `Sse.CompareLessThanOrEqual`
- **Missing**: AVX512 (16-wide), ARM64 (AdvSimd)
- **Fallback**: Scalar loop with unsafe pointer writes
- **TODO Comment**: "Cross platform intrinsics rewrite" at line 64
- **Purpose**: Creates mask where elements at index >= countInBundle are -1

### CreateMaskForCountInBundle
- **Line**: ~87
- **Signature**: `Vector<int> CreateMaskForCountInBundle(int countInBundle)`
- **Current Support**:
  - Vector256 (8-wide): Uses `Avx.CompareGreaterThan`
  - Vector128 (4-wide): Uses `Sse.CompareGreaterThan`
- **Missing**: AVX512 (16-wide), ARM64 (AdvSimd)
- **Fallback**: Scalar loop
- **TODO Comment**: "Cross platform intrinsics rewrite" at line 89
- **Purpose**: Creates mask where elements at index < countInBundle are -1

### GetFirstSetLaneIndex
- **Line**: ~109
- **Signature**: `int GetFirstSetLaneIndex(Vector<int> mask)`
- **Current Support**:
  - Vector256: Uses `Avx.MoveMask` + BitOperations.TrailingZeroCount
  - Vector128: Uses `Sse2.MoveMask` + BitOperations.TrailingZeroCount
- **Missing**: AVX512, ARM64
- **Fallback**: Scalar loop
- **TODO Comment**: "Probable cross platform intrinsics rewrite"
- **Purpose**: Returns index of first set lane, or -1 if none

### GetLastSetLaneCount
- **Line**: ~131
- **Signature**: `int GetLastSetLaneCount(Vector<int> mask)`
- **Current Support**:
  - Vector256: Uses `Avx.MoveMask` + BitOperations.LeadingZeroCount
  - Vector128: Uses `Sse2.MoveMask` + BitOperations.LeadingZeroCount
- **Missing**: AVX512, ARM64
- **Fallback**: Scalar loop
- **TODO Comment**: "Cross platform intrinsics rewrite"
- **Purpose**: Returns count of consecutive set lanes from the end

---

## 3. Sorting Operations

**File**: `BepuUtilities/Collections/VectorizedSorts.cs`

### VectorCountingSort
- **Line**: Various in file
- **Current Support**: Vector256 (8 elements), Vector128 (4 elements)
- **Implementation**: Vectorized comparisons for counting sort
- **Missing**: AVX512 (16 elements), ARM64
- **Requirement**: Hardware acceleration required for reasonable performance
- **Purpose**: SIMD-accelerated counting sort

---

## 4. Axis-Aligned Bounding Box Operations

**File**: `BepuUtilities/BoundingBox.cs`

### Intersects
- **Line**: ~93
- **Signature**: `bool Intersects(Vector3 min1, max1, min2, max2)`
- **Current Support**: Vector128 only (SSE)
- **Missing**: Vector256, Vector512, ARM64
- **Note**: Comment states AVX path "wasn't helpful"
- **Purpose**: AABB intersection test

### IntersectsUnsafe
- **Line**: ~126
- **Current Support**: Vector128 with SSE4.1 blend optimization
- **Missing**: Vector256, Vector512, ARM64
- **Purpose**: Optimized AABB intersection

### CreateMergedUnsafeWithPreservation
- **Line**: ~146
- **Current Support**: Vector128 with SSE4.1 blend
- **Missing**: Vector256, Vector512, ARM64
- **Purpose**: AABB merging with empty slot preservation

### CreateMergedUnsafe
- **Line**: ~177
- **Current Support**: Vector128 only
- **Missing**: Vector256, Vector512, ARM64
- **Purpose**: AABB merging without preservation

**Note**: All BoundingBox operations limited to 128-bit despite being potential broad-phase bottleneck.

---

## 5. Body State Operations

**File**: `BepuPhysics/Bodies_GatherScatter.cs`

### TransposeMotionStates
- **Line**: ~38
- **Signature**: `void TransposeMotionStates(...)`
- **Current Support**: Vector256 (AVX) for 8-wide operation
- **Implementation**: Complex use of:
  - `Avx.LoadAlignedVector256`
  - `Avx.UnpackLow` / `Avx.UnpackHigh`
  - `Avx.Shuffle`
  - `Avx.Permute2x128`
- **Missing**: Vector512 (16-wide), ARM64
- **Fallback**: Scalar loop
- **Purpose**: Transposes 8 bodies' motion states (position, orientation, velocity)
- **Criticality**: High - used in constraint solving hot path

### GatherState
- **Line**: ~155
- **Signature**: `void GatherState<TAccessFilter>(...)`
- **Current Support**: Vector256 (AVX) for 8-wide
- **Missing**: Vector512 (16-wide), ARM64
- **Purpose**: Gathers 8 bodies' dynamics into SIMD structures

### GatherInertia
- **Line**: ~195
- **Current Support**: Vector256 (AVX) 8-wide
- **Missing**: Vector512, ARM64
- **Purpose**: Gathers body inertia tensors

---

## 6. Kinematic State Checking

**File**: `BepuPhysics/Bodies.cs`

### IsKinematic
- **Line**: ~2084
- **Signature**: `Vector<int> IsKinematic(BodyInertia* inertias)`
- **Current Support**: Vector256 (AVX)
- **Implementation**: `Avx.LoadVector256` + `Avx.CompareEqual`
- **Missing**: Vector512, ARM64
- **Fallback**: Scalar
- **Purpose**: Fast kinematic body detection (checks for zero inverse mass)

### HasLockedInertia
- **Line**: ~2105
- **Signature**: `Vector<int> HasLockedInertia(Symmetric3x3* inertiaInverse)`
- **Current Support**: Vector256 (AVX)
- **Missing**: Vector512, ARM64
- **Purpose**: Checks for locked angular inertia

---

## 7. Constraint Solving

**File**: `BepuPhysics/Solver_Solve.cs`

### Constraint Flag Merging
- **Line**: ~1158
- **Current Support**:
  - ✅ Vector512 (16-wide) - **ONLY Vector512 usage in entire codebase**
  - Fallback to Vector256 (8-wide)
  - Fallback to scalar
- **Implementation**:
```csharp
if (Vector512.IsHardwareAccelerated && Vector<int>.Count == 16)
{
    // Vector512 path
}
else if (Vector256.IsHardwareAccelerated && Vector<int>.Count == 8)
{
    // Vector256 path
}
else
{
    // Scalar path
}
```
- **Missing**: ARM64
- **Status**: ✅ Good x86 coverage with proper fallbacks
- **Purpose**: Merges constrained body handles flags

---

## 8. Tree Operations

**File**: `BepuPhysics/Trees/Tree_SelfQueries.cs`

### GetLeftPackMask
- **Line**: ~187
- **Signature**: `int GetLeftPackMask(Vector<int> intersected)`
- **Current Support**: AVX2 ONLY
- **Implementation**: `Avx2.ExtractMostSignificantBits` + lookup tables
- **Missing**: All other platforms
- **Fallback**: ⚠️ **NONE - throws NotSupportedException**
- **Criticality**: ⚠️ **HIGH - Hard requirement, no graceful degradation**
- **Purpose**: Packing masks for tree traversal

**File**: `BepuPhysics/Trees/Tree_BinnedBuilder.cs`

### ComputeBinIndex
- **Line**: ~173
- **Current Support**:
  - Avx.PermuteVar for variable shuffle
  - Cross-platform Vector128.GetElement fallback
- **Missing**: Wider SIMD, optimized ARM64
- **Purpose**: Compute bin indices for tree building

---

## Optimization Priority Matrix

### High Priority (Performance Critical + Missing Wide SIMD)

1. **TransposeMotionStates** - Constraint solver hot path
   - Missing: AVX512 (16-wide), ARM64
   - Location: BepuPhysics/Bodies_GatherScatter.cs:38

2. **GatherState** - Body state gathering
   - Missing: AVX512 (16-wide), ARM64
   - Location: BepuPhysics/Bodies_GatherScatter.cs:155

3. **GetLeftPackMask** - Tree queries
   - ⚠️ **CRITICAL**: No fallback, AVX2 required
   - Location: BepuPhysics/Trees/Tree_SelfQueries.cs:187

### Medium Priority (Frequently Used)

4. **FastReciprocal** - Math operations
   - Missing: AVX512, ARM64
   - Location: BepuUtilities/MathHelper.cs:380

5. **FastReciprocalSquareRoot** - Math operations
   - Missing: AVX512, ARM64
   - Location: BepuUtilities/MathHelper.cs:397

6. **Bundle Indexing Functions** (4 functions)
   - Missing: AVX512, ARM64
   - Location: BepuUtilities/BundleIndexing.cs

### Lower Priority (Less Critical or Commented as Not Helpful)

7. **BoundingBox Operations** - Currently 128-bit only
   - Note: AVX path commented as "wasn't helpful"
   - Location: BepuUtilities/BoundingBox.cs

---

## Quick Reference: Platform Support

```
Legend:
✅ Full support
⚠️  Partial support / Has fallback
❌ Not supported / No fallback
```

| Function Category | SSE/AVX | AVX2 | AVX512 | ARM NEON |
|-------------------|---------|------|---------|----------|
| Math (Reciprocal) | ✅ | N/A | ❌ | ❌ |
| Bundle Indexing | ✅ | ⚠️ | ❌ | ❌ |
| Sorting | ✅ | N/A | ❌ | ❌ |
| AABB Operations | ✅ (128) | N/A | ❌ | ❌ |
| Body Transpose | ✅ (256) | N/A | ❌ | ❌ |
| Body Gather | ✅ (256) | N/A | ❌ | ❌ |
| Kinematic Check | ✅ | N/A | ❌ | ❌ |
| Constraint Flags | ✅ | N/A | ✅ | ❌ |
| Tree GetLeftPack | N/A | ✅ **required** | ❌ | ❌ |
| Tree BinIndex | ✅ | N/A | ❌ | ❌ |

---

## Testing Strategy

The SimdTests project provides:

1. ✅ **Reference implementations** for correctness testing
2. ✅ **Automated tests** using xUnit
3. ✅ **Performance benchmarks** measuring ns/iteration
4. ✅ **Extensible framework** for adding new tests

Currently tested functions:
- FastReciprocal
- FastReciprocalSquareRoot
- CreateTrailingMaskForCountInBundle
- CreateMaskForCountInBundle
- GetFirstSetLaneIndex
- GetLastSetLaneCount

## Next Steps for Optimization

1. **Add AVX512 support** to high-priority functions
2. **Add ARM64/NEON support** for cross-platform performance
3. **Fix GetLeftPackMask** to have graceful fallback
4. **Profile real-world workloads** to validate optimization priorities
5. **Extend test coverage** to all SIMD functions
