using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;

namespace AosBaselines;

/// <summary>
/// Scalar mirrors of the wide math helpers used by the collision testers.
/// Each function preserves the per-component operation order of its Vector&lt;float&gt; counterpart so that results are bitwise identical per lane.
/// </summary>
/// <remarks>
/// Implementation style matters enormously here: RyuJIT promotes Vector3 locals to SIMD registers (TYP_SIMD12), so componentwise
/// field access on them generates an insert/extract shuffle storm and spills. Helpers therefore operate on whole vectors, and lean on the
/// portable Vector128 API — the JIT maps it onto SSE/AVX or NEON with the same lane semantics, so nothing here is x64-specific.
/// Vector3 is treated roughly like a Vector128 by the compiler (shared backend, modulo W-lane clearing), so prefer Vector3's own
/// operators and intrinsics at call sites: Vector3.Cross, Matrix3x3.CreateFromQuaternion, and Matrix3x3.Transform were
/// fuzz-verified bitwise-equal to the wide code and their mirror helpers deleted as redundant.
/// For scalar min/max use float.MinNative/MaxNative (the platform-native per-lane match to Vector.Min/Max).
/// What remains earned its place with evidence:
/// - Dot/Length/TransformByTransposed: Vector3.Dot's lowering is the pairwise (x + y) + (z + 0) tree (dotprobe mode), which
///   diverges from the wide linear sums at negative-zero partial sums; aligning the engine instead was explored, verified
///   (requires a non-removable + Vector<float>.Zero in Vector3Wide.Dot), and declined — so the ordered mirrors stay.
/// - Select/mask helpers: pin ConditionalSelect blend semantics and keep condition chains in the SIMD register domain
///   (measured: the mask-select conversion was one of the larger box-box wins).
/// - MultiplyByTranspose (a * transpose(b)) and TransformUnitXY: no scalar engine equivalent exists (Matrix3x3.MultiplyTransposed
///   is transpose(a) * b, a different operation).
/// </remarks>
public static class ScalarMath
{
    /// <summary>
    /// Branchless equivalent of condition ? a : b for floats. Bitwise identical to the ternary.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float Select(bool condition, float a, float b)
    {
        if (Vector128.IsHardwareAccelerated)
        {
            var mask = Vector128.CreateScalarUnsafe(condition ? -1 : 0).AsSingle();
            return Vector128.ConditionalSelect(mask, Vector128.CreateScalarUnsafe(a), Vector128.CreateScalarUnsafe(b)).ToScalar();
        }
        var bits = condition ? uint.MaxValue : 0u;
        return BitConverter.UInt32BitsToSingle((BitConverter.SingleToUInt32Bits(a) & bits) | (BitConverter.SingleToUInt32Bits(b) & ~bits));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Select(bool condition, int a, int b)
    {
        var mask = condition ? -1 : 0;
        return (a & mask) | (b & ~mask);
    }

    //Vector3 results are returned by value: Vector3 is a SIMD type and comes back in a register, whereas an out parameter is a
    //byref whose address-taken pattern the JIT must undo to keep the destination enregistered (usually does, not always).
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 Select(bool condition, Vector3 a, Vector3 b)
    {
        if (Vector128.IsHardwareAccelerated)
        {
            //AsVector128Unsafe skips the W-lane zeroing insert; the select is bitwise per lane, so garbage in W never touches lanes 0-2.
            var mask = Vector128.Create(condition ? -1 : 0).AsSingle();
            return Vector128.ConditionalSelect(mask, a.AsVector128Unsafe(), b.AsVector128Unsafe()).AsVector3();
        }
        return new Vector3(Select(condition, a.X, b.X), Select(condition, a.Y, b.Y), Select(condition, a.Z, b.Z));
    }

    //Mask-producing comparisons and mask-consuming selects: computing the condition as a vector compare keeps select chains entirely in the
    //SIMD register domain (a bool-based select costs a compare + setcc/cmov + a gpr->xmm domain crossing before the blend).
    //All comparisons are ordered/non-signaling, so NaN behavior matches the C# operators, and the resulting all-ones/all-zeros masks
    //make the selects bitwise identical to the bool-based forms.
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<float> LessMask(float a, float b)
    {
        if (Vector128.IsHardwareAccelerated)
            return Vector128.LessThan(Vector128.Create(a), Vector128.Create(b));
        return Vector128.Create(a < b ? -1 : 0).AsSingle();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<float> LessOrEqualMask(float a, float b)
    {
        if (Vector128.IsHardwareAccelerated)
            return Vector128.LessThanOrEqual(Vector128.Create(a), Vector128.Create(b));
        return Vector128.Create(a <= b ? -1 : 0).AsSingle();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<float> GreaterMask(float a, float b)
    {
        if (Vector128.IsHardwareAccelerated)
            return Vector128.GreaterThan(Vector128.Create(a), Vector128.Create(b));
        return Vector128.Create(a > b ? -1 : 0).AsSingle();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<float> GreaterOrEqualMask(float a, float b)
    {
        if (Vector128.IsHardwareAccelerated)
            return Vector128.GreaterThanOrEqual(Vector128.Create(a), Vector128.Create(b));
        return Vector128.Create(a >= b ? -1 : 0).AsSingle();
    }

    /// <summary>
    /// Broadcasts a bool into an all-ones/all-zeros mask for use with the mask-based selects.
    /// One gpr->simd crossing; prefer the comparison mask helpers when the condition is a float compare.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<float> Mask(bool condition)
    {
        return Vector128.Create(condition ? -1 : 0).AsSingle();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<float> EqualMask(float a, float b)
    {
        if (Vector128.IsHardwareAccelerated)
            return Vector128.Equals(Vector128.Create(a), Vector128.Create(b));
        return Vector128.Create(a == b ? -1 : 0).AsSingle();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float Select(Vector128<float> mask, float a, float b)
    {
        if (Vector128.IsHardwareAccelerated)
            return Vector128.ConditionalSelect(mask, Vector128.CreateScalarUnsafe(a), Vector128.CreateScalarUnsafe(b)).ToScalar();
        return mask.AsInt32().ToScalar() != 0 ? a : b;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Select(Vector128<float> mask, int a, int b)
    {
        if (Vector128.IsHardwareAccelerated)
            return Vector128.ConditionalSelect(mask.AsInt32(), Vector128.CreateScalarUnsafe(a), Vector128.CreateScalarUnsafe(b)).ToScalar();
        var bits = mask.AsInt32().ToScalar();
        return (a & bits) | (b & ~bits);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 Select(Vector128<float> mask, Vector3 a, Vector3 b)
    {
        if (Vector128.IsHardwareAccelerated)
            return Vector128.ConditionalSelect(mask, a.AsVector128Unsafe(), b.AsVector128Unsafe()).AsVector3();
        return mask.AsInt32().ToScalar() != 0 ? a : b;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2 Select(Vector128<float> mask, Vector2 a, Vector2 b)
    {
        if (Vector128.IsHardwareAccelerated)
            return Vector128.ConditionalSelect(mask, a.AsVector128Unsafe(), b.AsVector128Unsafe()).AsVector2();
        return mask.AsInt32().ToScalar() != 0 ? a : b;
    }

    /// <summary>
    /// Mirrors Vector3Wide.Dot: x then y then z, left associative. Vector3.Dot's lowering reduces as the pairwise tree
    /// (x + y) + (z + 0) instead (probed via the harness's dotprobe mode), which differs from the wide linear sum exactly at
    /// negative-zero partial sums — matching it engine-side requires a non-removable + Vector&lt;float&gt;.Zero term per dot,
    /// a bullet deliberately not bitten. Hence the explicit ordered helper.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float Dot(Vector3 a, Vector3 b)
    {
        if (Vector128.IsHardwareAccelerated)
            return OrderedDot(a.AsVector128(), b.AsVector128());
        return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
    }

    /// <summary>
    /// Mirrors Vector3Wide.Length: sqrt(x*x + y*y + z*z).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float Length(Vector3 v) => MathF.Sqrt(Dot(v, v));

    /// <summary>
    /// Sum of the products of the first three lanes, reduced in lane order: (m0 + m1) + m2, matching the componentwise mirrors.
    /// The adds run full width; only lane 0 is read, and per-lane adds cannot contaminate it.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float OrderedDot(Vector128<float> a, Vector128<float> b)
    {
        var products = a * b;
        var sum01 = products + Vector128.Shuffle(products, Vector128.Create(1, 1, 3, 3));
        return (sum01 + Vector128.Shuffle(products, Vector128.Create(2, 2, 2, 2))).ToScalar();
    }

    /// <summary>
    /// Mirrors Matrix3x3Wide.MultiplyByTransposeWithoutOverlap: result = a * transpose(b). Each result row is a-row dotted with b's rows.
    /// Implemented as transpose-once + broadcast-row sums: per component, (a_i.X * b_c.X + a_i.Y * b_c.Y) + a_i.Z * b_c.Z with the
    /// same multiplicand order and add associativity as the ordered dot, so results are bitwise identical with far better ILP
    /// than nine sequential horizontal dots.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void MultiplyByTranspose(Matrix3x3 a, Matrix3x3 b, out Matrix3x3 result)
    {
        Matrix3x3.Transpose(b, out var bT);
        result.X = a.X.X * bT.X + a.X.Y * bT.Y + a.X.Z * bT.Z;
        result.Y = a.Y.X * bT.X + a.Y.Y * bT.Y + a.Y.Z * bT.Z;
        result.Z = a.Z.X * bT.X + a.Z.Y * bT.Y + a.Z.Z * bT.Z;
    }

    /// <summary>
    /// Mirrors Matrix3x3Wide.TransformByTransposedWithoutOverlap: result = v * transpose(m); each component is v dotted with a row of m.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 TransformByTransposed(Vector3 v, Matrix3x3 m)
    {
        if (Vector128.IsHardwareAccelerated)
        {
            var vv = v.AsVector128();
            return new Vector3(OrderedDot(vv, m.X.AsVector128()), OrderedDot(vv, m.Y.AsVector128()), OrderedDot(vv, m.Z.AsVector128()));
        }
        return new Vector3(
            v.X * m.X.X + v.Y * m.X.Y + v.Z * m.X.Z,
            v.X * m.Y.X + v.Y * m.Y.Y + v.Z * m.Y.Z,
            v.X * m.Z.X + v.Y * m.Z.Y + v.Z * m.Z.Z);
    }

    /// <summary>
    /// Mirrors QuaternionWide.TransformUnitXY.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void TransformUnitXY(Quaternion rotation, out Vector3 x, out Vector3 y)
    {
        var x2 = rotation.X + rotation.X;
        var y2 = rotation.Y + rotation.Y;
        var z2 = rotation.Z + rotation.Z;
        var xx2 = rotation.X * x2;
        var xy2 = rotation.X * y2;
        var xz2 = rotation.X * z2;
        var yy2 = rotation.Y * y2;
        var yz2 = rotation.Y * z2;
        var zz2 = rotation.Z * z2;
        var wx2 = rotation.W * x2;
        var wy2 = rotation.W * y2;
        var wz2 = rotation.W * z2;
        x = new Vector3(1f - yy2 - zz2, xy2 + wz2, xz2 - wy2);
        y = new Vector3(xy2 - wz2, 1f - xx2 - zz2, yz2 + wx2);
    }
}
