using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Threading;

namespace BepuPhysics.Collidables
{
    public static partial class ConvexHullHelper
    {
        /// <summary>
        /// Exact orientation of float points: a double evaluation decides almost every case, with Shewchuk's orient3d
        /// error bound saying when it can be trusted; the rest are evaluated exactly in integers (every float is an
        /// integer times a power of two).
        /// </summary>
        internal static class ExactOrientation
        {
            /// <summary>
            /// Shewchuk's orient3d error bound coefficient: (7 + 56 eps) eps, eps = 2^-53.
            /// </summary>
            const double Bound = 7.771561172376103e-16;

            /// <summary>
            /// Orientations the double evaluation couldn't decide, evaluated exactly.
            /// </summary>
            public static int ExactEvaluations;

            /// <summary>
            /// The sign of (b - a) x (c - a) . (d - a), exactly: positive when d is in front of triangle abc (on the side
            /// cross(b - a, c - a) points to), zero when the four points are coplanar.
            /// </summary>
            public static int Orient(Vector3 a, Vector3 b, Vector3 c, Vector3 d)
            {
                double ux = (double)b.X - a.X, uy = (double)b.Y - a.Y, uz = (double)b.Z - a.Z;
                double vx = (double)c.X - a.X, vy = (double)c.Y - a.Y, vz = (double)c.Z - a.Z;
                double wx = (double)d.X - a.X, wy = (double)d.Y - a.Y, wz = (double)d.Z - a.Z;
                double vywz = vy * wz, vzwy = vz * wy;
                double vzwx = vz * wx, vxwz = vx * wz;
                double vxwy = vx * wy, vywx = vy * wx;
                var determinant = ux * (vywz - vzwy) + uy * (vzwx - vxwz) + uz * (vxwy - vywx);
                var permanent = (Math.Abs(vywz) + Math.Abs(vzwy)) * Math.Abs(ux)
                              + (Math.Abs(vzwx) + Math.Abs(vxwz)) * Math.Abs(uy)
                              + (Math.Abs(vxwy) + Math.Abs(vywx)) * Math.Abs(uz);
                var errorBound = Bound * permanent;
                if (determinant > errorBound)
                    return 1;
                if (-determinant > errorBound)
                    return -1;
                Interlocked.Increment(ref ExactEvaluations);
                return OrientExact(a, b, c, d);
            }

            /// <summary>
            /// Orient evaluated exactly with floating point expansions (Shewchuk's arithmetic): every value is kept as a sum
            /// of non-overlapping doubles, built with error-free sums and products, so nothing is rounded away. The sign of
            /// the result is the sign of its largest component.
            /// </summary>
            internal static int OrientExact(Vector3 a, Vector3 b, Vector3 c, Vector3 d)
            {
                //Differences: two components each.
                Span<double> ux = stackalloc double[2], uy = stackalloc double[2], uz = stackalloc double[2];
                Span<double> vx = stackalloc double[2], vy = stackalloc double[2], vz = stackalloc double[2];
                Span<double> wx = stackalloc double[2], wy = stackalloc double[2], wz = stackalloc double[2];
                var uxLength = TwoDiff(b.X, a.X, ux);
                var uyLength = TwoDiff(b.Y, a.Y, uy);
                var uzLength = TwoDiff(b.Z, a.Z, uz);
                var vxLength = TwoDiff(c.X, a.X, vx);
                var vyLength = TwoDiff(c.Y, a.Y, vy);
                var vzLength = TwoDiff(c.Z, a.Z, vz);
                var wxLength = TwoDiff(d.X, a.X, wx);
                var wyLength = TwoDiff(d.Y, a.Y, wy);
                var wzLength = TwoDiff(d.Z, a.Z, wz);
                //u . (v x w), one 2x2 minor at a time: a minor is at most 16 components, times a difference at most 64.
                Span<double> minor = stackalloc double[16];
                Span<double> term = stackalloc double[64];
                Span<double> total = stackalloc double[192];
                Span<double> scratch = stackalloc double[192];
                var totalLength = 0;
                var minorLength = Minor(vy[..vyLength], wz[..wzLength], vz[..vzLength], wy[..wyLength], minor);
                var termLength = Multiply(ux[..uxLength], minor[..minorLength], term);
                totalLength = Add(total[..totalLength], term[..termLength], scratch);
                scratch[..totalLength].CopyTo(total);
                minorLength = Minor(vz[..vzLength], wx[..wxLength], vx[..vxLength], wz[..wzLength], minor);
                termLength = Multiply(uy[..uyLength], minor[..minorLength], term);
                totalLength = Add(total[..totalLength], term[..termLength], scratch);
                scratch[..totalLength].CopyTo(total);
                minorLength = Minor(vx[..vxLength], wy[..wyLength], vy[..vyLength], wx[..wxLength], minor);
                termLength = Multiply(uz[..uzLength], minor[..minorLength], term);
                totalLength = Add(total[..totalLength], term[..termLength], scratch);
                for (int i = totalLength - 1; i >= 0; --i)
                {
                    if (scratch[i] != 0)
                        return scratch[i] > 0 ? 1 : -1;
                }
                return 0;
            }

            /// <summary>
            /// a - b exactly, as up to two components (smallest first, zeros dropped).
            /// </summary>
            static int TwoDiff(double a, double b, Span<double> result)
            {
                var x = a - b;
                var bVirtual = a - x;
                var aVirtual = x + bVirtual;
                var y = (a - aVirtual) + (bVirtual - b);
                var length = 0;
                if (y != 0)
                    result[length++] = y;
                if (x != 0)
                    result[length++] = x;
                return length;
            }

            /// <summary>
            /// p * q - r * s exactly.
            /// </summary>
            static int Minor(Span<double> p, Span<double> q, Span<double> r, Span<double> s, Span<double> result)
            {
                Span<double> left = stackalloc double[8];
                Span<double> right = stackalloc double[8];
                var leftLength = Multiply(p, q, left);
                var rightLength = Multiply(r, s, right);
                for (int i = 0; i < rightLength; ++i)
                    right[i] = -right[i];
                return Add(left[..leftLength], right[..rightLength], result);
            }

            /// <summary>
            /// The product of two expansions exactly: the first scaled by each component of the second, summed.
            /// </summary>
            static int Multiply(Span<double> e, Span<double> f, Span<double> result)
            {
                var length = 0;
                Span<double> scaled = stackalloc double[2 * e.Length];
                Span<double> sum = stackalloc double[result.Length];
                for (int i = 0; i < f.Length; ++i)
                {
                    var scaledLength = Scale(e, f[i], scaled);
                    var sumLength = Add(result[..length], scaled[..scaledLength], sum);
                    sum[..sumLength].CopyTo(result);
                    length = sumLength;
                }
                return length;
            }

            /// <summary>
            /// An expansion times a double exactly (Shewchuk's scale_expansion_zeroelim).
            /// </summary>
            static int Scale(Span<double> e, double b, Span<double> result)
            {
                var length = 0;
                if (e.Length == 0 || b == 0)
                    return 0;
                TwoProduct(e[0], b, out var q, out var h);
                if (h != 0)
                    result[length++] = h;
                for (int i = 1; i < e.Length; ++i)
                {
                    TwoProduct(e[i], b, out var product1, out var product0);
                    TwoSum(q, product0, out var sum, out h);
                    if (h != 0)
                        result[length++] = h;
                    FastTwoSum(product1, sum, out q, out h);
                    if (h != 0)
                        result[length++] = h;
                }
                if (q != 0)
                    result[length++] = q;
                return length;
            }

            /// <summary>
            /// The sum of two expansions exactly: each component of f grown into e in turn (Shewchuk's
            /// grow_expansion_zeroelim, repeated).
            /// </summary>
            static int Add(Span<double> e, Span<double> f, Span<double> result)
            {
                e.CopyTo(result);
                var length = e.Length;
                Span<double> grown = stackalloc double[result.Length];
                for (int i = 0; i < f.Length; ++i)
                {
                    var q = f[i];
                    var grownLength = 0;
                    for (int j = 0; j < length; ++j)
                    {
                        TwoSum(q, result[j], out q, out var h);
                        if (h != 0)
                            grown[grownLength++] = h;
                    }
                    if (q != 0)
                        grown[grownLength++] = q;
                    grown[..grownLength].CopyTo(result);
                    length = grownLength;
                }
                return length;
            }

            static void TwoSum(double a, double b, out double x, out double y)
            {
                x = a + b;
                var bVirtual = x - a;
                var aVirtual = x - bVirtual;
                y = (a - aVirtual) + (b - bVirtual);
            }

            static void FastTwoSum(double a, double b, out double x, out double y)
            {
                x = a + b;
                y = b - (x - a);
            }

            static void TwoProduct(double a, double b, out double x, out double y)
            {
                x = a * b;
                y = Math.FusedMultiplyAdd(a, b, -x);
            }
        }

        /// <summary>
        /// QuickHull with exact orientation tests, then merging of coplanar triangles into faces. Memory comes from the pool.
        /// </summary>
        static class ExactHull
        {
            /// <summary>
            /// Point sets thinner than this share of their size have no volume (the gift wrapping's coplanarity slab).
            /// </summary>
            const double FlatShare = 1e-4;
            /// <summary>
            /// Triangles whose vertices lie within this share of the point set's size of a face's plane are merged into it.
            /// </summary>
            const double MergeShare = 1e-5;

            struct Triangle
            {
                public int A, B, C;
                //Neighbouring triangles across edges AB, BC and CA.
                public int N0, N1, N2;
                //Unit normal and plane offset in doubles relative to the centroid, for choosing the furthest point.
                public double NX, NY, NZ, Offset;
                //First of the points in front of it (linked through Context.NextOutside), or -1.
                public int OutsideHead;
                public int Stamp;
                public bool Alive;

                public readonly int Vertex(int index) => index == 0 ? A : index == 1 ? B : C;
                public readonly int Neighbor(int edge) => edge == 0 ? N0 : edge == 1 ? N1 : N2;
                public void SetNeighbor(int edge, int triangle)
                {
                    if (edge == 0) N0 = triangle;
                    else if (edge == 1) N1 = triangle;
                    else N2 = triangle;
                }
            }

            ref struct Context
            {
                public Span<Vector3> Points;
                public Buffer<double> X, Y, Z;
                public QuickList<Triangle> Triangles;
                public Buffer<int> NextOutside;
                public QuickList<int> Pending;
            }

            static double Distance(ref Context context, in Triangle triangle, int point) =>
                triangle.NX * context.X[point] + triangle.NY * context.Y[point] + triangle.NZ * context.Z[point] - triangle.Offset;

            static bool InFront(ref Context context, in Triangle triangle, int point) =>
                ExactOrientation.Orient(context.Points[triangle.A], context.Points[triangle.B], context.Points[triangle.C], context.Points[point]) > 0;

            static int AddTriangle(ref Context context, int a, int b, int c, BufferPool pool)
            {
                double ux = context.X[b] - context.X[a], uy = context.Y[b] - context.Y[a], uz = context.Z[b] - context.Z[a];
                double vx = context.X[c] - context.X[a], vy = context.Y[c] - context.Y[a], vz = context.Z[c] - context.Z[a];
                double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
                var length = Math.Sqrt(nx * nx + ny * ny + nz * nz);
                if (length > 0)
                {
                    nx /= length;
                    ny /= length;
                    nz /= length;
                }
                var index = context.Triangles.Count;
                context.Triangles.Add(new Triangle
                {
                    A = a, B = b, C = c, N0 = -1, N1 = -1, N2 = -1,
                    NX = nx, NY = ny, NZ = nz, Offset = nx * context.X[a] + ny * context.Y[a] + nz * context.Z[a],
                    OutsideHead = -1, Stamp = 0, Alive = true
                }, pool);
                return index;
            }

            /// <summary>
            /// Puts a point in the outside list of the first candidate triangle it's exactly in front of.
            /// </summary>
            static void Assign(ref Context context, int point, scoped Span<int> candidates, BufferPool pool)
            {
                for (int i = 0; i < candidates.Length; ++i)
                {
                    ref var triangle = ref context.Triangles[candidates[i]];
                    if (InFront(ref context, triangle, point))
                    {
                        if (triangle.OutsideHead < 0)
                            context.Pending.Add(candidates[i], pool);
                        context.NextOutside[point] = triangle.OutsideHead;
                        triangle.OutsideHead = point;
                        return;
                    }
                }
            }

            /// <summary>
            /// Links triangle t's edge from a to b with the triangle across it.
            /// </summary>
            static void Link(ref Context context, int t, int a, int b, int other)
            {
                ref var triangle = ref context.Triangles[t];
                for (int edge = 0; edge < 3; ++edge)
                {
                    if (triangle.Vertex(edge) == a && triangle.Vertex((edge + 1) % 3) == b)
                    {
                        triangle.SetNeighbor(edge, other);
                        return;
                    }
                }
                Debug.Fail("Edge to link not found on its triangle.");
            }

            /// <summary>
            /// Computes the hull data. False when the points have no volume (they're then left to gift wrapping).
            /// </summary>
            public static bool TryCompute(Span<Vector3> points, BufferPool pool, out HullData hullData)
            {
                hullData = default;
                var count = points.Length;
                for (int i = 0; i < count; ++i)
                {
                    if (!float.IsFinite(points[i].X) || !float.IsFinite(points[i].Y) || !float.IsFinite(points[i].Z))
                        return false;
                }
                Context context;
                context.Points = points;
                pool.Take(count, out context.X);
                pool.Take(count, out context.Y);
                pool.Take(count, out context.Z);
                double cx = 0, cy = 0, cz = 0;
                for (int i = 0; i < count; ++i)
                {
                    cx += points[i].X;
                    cy += points[i].Y;
                    cz += points[i].Z;
                }
                cx /= count;
                cy /= count;
                cz /= count;
                for (int i = 0; i < count; ++i)
                {
                    context.X[i] = points[i].X - cx;
                    context.Y[i] = points[i].Y - cy;
                    context.Z[i] = points[i].Z - cz;
                }
                context.Triangles = new QuickList<Triangle>(Math.Max(16, count * 2), pool);
                pool.Take(count, out context.NextOutside);
                context.Pending = new QuickList<int>(32, pool);
                var success = Build(ref context, pool, out var size) && MergeFaces(ref context, size * MergeShare, pool, out hullData);
                pool.Return(ref context.X);
                pool.Return(ref context.Y);
                pool.Return(ref context.Z);
                context.Triangles.Dispose(pool);
                pool.Return(ref context.NextOutside);
                context.Pending.Dispose(pool);
                return success;
            }

            static bool Build(ref Context context, BufferPool pool, out double size)
            {
                size = 0;
                var count = context.Points.Length;
                ref var x = ref context.X;
                ref var y = ref context.Y;
                ref var z = ref context.Z;
                //The first tetrahedron: the furthest apart pair of axis extremes, the point furthest from their line, then the
                //point furthest from their plane; the last must be exactly off it.
                //(cleared: the library skips local initialization, so stackalloc memory isn't zeroed)
                Span<int> extremes = stackalloc int[6];
                extremes.Clear();
                for (int i = 0; i < count; ++i)
                {
                    if (x[i] < x[extremes[0]]) extremes[0] = i;
                    if (x[i] > x[extremes[1]]) extremes[1] = i;
                    if (y[i] < y[extremes[2]]) extremes[2] = i;
                    if (y[i] > y[extremes[3]]) extremes[3] = i;
                    if (z[i] < z[extremes[4]]) extremes[4] = i;
                    if (z[i] > z[extremes[5]]) extremes[5] = i;
                }
                int i0 = 0, i1 = 0;
                var best = -1.0;
                for (int a = 0; a < 6; ++a)
                {
                    for (int b = a + 1; b < 6; ++b)
                    {
                        double dx = x[extremes[a]] - x[extremes[b]], dy = y[extremes[a]] - y[extremes[b]], dz = z[extremes[a]] - z[extremes[b]];
                        var d = dx * dx + dy * dy + dz * dz;
                        if (d > best)
                        {
                            best = d;
                            i0 = extremes[a];
                            i1 = extremes[b];
                        }
                    }
                }
                if (!(best > 0))
                    return false;
                size = Math.Sqrt(best);
                double ax = (x[i1] - x[i0]) / size, ay = (y[i1] - y[i0]) / size, az = (z[i1] - z[i0]) / size;
                int i2 = -1;
                var far = 0.0;
                for (int i = 0; i < count; ++i)
                {
                    double rx = x[i] - x[i0], ry = y[i] - y[i0], rz = z[i] - z[i0];
                    var along = rx * ax + ry * ay + rz * az;
                    rx -= ax * along;
                    ry -= ay * along;
                    rz -= az * along;
                    var d = rx * rx + ry * ry + rz * rz;
                    if (d > far)
                    {
                        far = d;
                        i2 = i;
                    }
                }
                //(flat or collinear within the gift wrapping's own slab: no volume, as before)
                if (i2 < 0 || Math.Sqrt(far) <= size * FlatShare)
                    return false;
                double ux = x[i1] - x[i0], uy = y[i1] - y[i0], uz = z[i1] - z[i0];
                double vx = x[i2] - x[i0], vy = y[i2] - y[i0], vz = z[i2] - z[i0];
                double px = uy * vz - uz * vy, py = uz * vx - ux * vz, pz = ux * vy - uy * vx;
                var planeLength = Math.Sqrt(px * px + py * py + pz * pz);
                int i3 = -1;
                far = 0;
                for (int i = 0; i < count; ++i)
                {
                    var d = Math.Abs(px * (x[i] - x[i0]) + py * (y[i] - y[i0]) + pz * (z[i] - z[i0]));
                    if (d > far)
                    {
                        far = d;
                        i3 = i;
                    }
                }
                if (i3 < 0 || far <= size * FlatShare * planeLength || ExactOrientation.Orient(context.Points[i0], context.Points[i1], context.Points[i2], context.Points[i3]) == 0)
                    return false;
                //Wound so the fourth vertex is behind each face.
                if (ExactOrientation.Orient(context.Points[i0], context.Points[i1], context.Points[i2], context.Points[i3]) > 0)
                    (i1, i2) = (i2, i1);
                //Now i3 is behind (i0, i1, i2); the other faces follow.
                AddTriangle(ref context, i0, i1, i2, pool);
                AddTriangle(ref context, i0, i3, i1, pool);
                AddTriangle(ref context, i1, i3, i2, pool);
                AddTriangle(ref context, i2, i3, i0, pool);
                for (int t = 0; t < 4; ++t)
                {
                    ref var triangle = ref context.Triangles[t];
                    for (int edge = 0; edge < 3; ++edge)
                    {
                        int a = triangle.Vertex(edge), b = triangle.Vertex((edge + 1) % 3);
                        for (int other = 0; other < 4; ++other)
                        {
                            if (other == t)
                                continue;
                            ref var candidate = ref context.Triangles[other];
                            for (int otherEdge = 0; otherEdge < 3; ++otherEdge)
                            {
                                if (candidate.Vertex(otherEdge) == b && candidate.Vertex((otherEdge + 1) % 3) == a)
                                    triangle.SetNeighbor(edge, other);
                            }
                        }
                    }
                }
                Span<int> first = stackalloc int[] { 0, 1, 2, 3 };
                for (int i = 0; i < count; ++i)
                {
                    if (i != i0 && i != i1 && i != i2 && i != i3)
                        Assign(ref context, i, first, pool);
                }

                var visible = new QuickList<int>(32, pool);
                var stack = new QuickList<int>(32, pool);
                var created = new QuickList<int>(32, pool);
                var orphans = new QuickList<int>(32, pool);
                //For stitching new triangles together: the new triangle whose horizon edge starts at a vertex.
                pool.Take<int>(count, out var createdFrom);
                var stamp = 0;
                while (context.Pending.Count > 0)
                {
                    var top = context.Pending[context.Pending.Count - 1];
                    --context.Pending.Count;
                    if (!context.Triangles[top].Alive || context.Triangles[top].OutsideHead < 0)
                        continue;
                    //Every point in its list is exactly in front of it; the furthest is chosen by the doubles.
                    var apex = -1;
                    var apexDistance = double.MinValue;
                    for (var point = context.Triangles[top].OutsideHead; point >= 0; point = context.NextOutside[point])
                    {
                        var d = Distance(ref context, context.Triangles[top], point);
                        if (d > apexDistance)
                        {
                            apexDistance = d;
                            apex = point;
                        }
                    }
                    //The triangles the apex is exactly in front of: one connected patch from this one.
                    ++stamp;
                    visible.Count = 0;
                    stack.Count = 0;
                    stack.Add(top, pool);
                    context.Triangles[top].Stamp = stamp;
                    while (stack.Count > 0)
                    {
                        var t = stack[stack.Count - 1];
                        --stack.Count;
                        visible.Add(t, pool);
                        for (int edge = 0; edge < 3; ++edge)
                        {
                            var neighbor = context.Triangles[t].Neighbor(edge);
                            ref var other = ref context.Triangles[neighbor];
                            if (other.Stamp != stamp && InFront(ref context, other, apex))
                            {
                                other.Stamp = stamp;
                                stack.Add(neighbor, pool);
                            }
                        }
                    }
                    //A new triangle on each horizon edge (a visible triangle's edge whose neighbour isn't visible), joined to
                    //the triangle across it and to its new neighbours.
                    created.Count = 0;
                    for (int i = 0; i < visible.Count; ++i)
                    {
                        var t = visible[i];
                        for (int edge = 0; edge < 3; ++edge)
                        {
                            var neighbor = context.Triangles[t].Neighbor(edge);
                            if (context.Triangles[neighbor].Stamp == stamp)
                                continue;
                            int a = context.Triangles[t].Vertex(edge), b = context.Triangles[t].Vertex((edge + 1) % 3);
                            var newTriangle = AddTriangle(ref context, a, b, apex, pool);
                            context.Triangles[newTriangle].N0 = neighbor;
                            Link(ref context, neighbor, b, a, newTriangle);
                            createdFrom[a] = newTriangle;
                            created.Add(newTriangle, pool);
                        }
                    }
                    for (int i = 0; i < created.Count; ++i)
                    {
                        ref var triangle = ref context.Triangles[created[i]];
                        //Edge B -> apex meets the new triangle starting at B; edge apex -> A meets the one ending at A, which
                        //(the horizon being a loop) is the one whose own B -> apex edge was linked to this one.
                        var next = createdFrom[triangle.B];
                        triangle.N1 = next;
                        context.Triangles[next].N2 = created[i];
                    }
                    //The visible triangles go; the points in front of them go to the new triangles, or are inside.
                    orphans.Count = 0;
                    for (int i = 0; i < visible.Count; ++i)
                    {
                        ref var triangle = ref context.Triangles[visible[i]];
                        triangle.Alive = false;
                        for (var point = triangle.OutsideHead; point >= 0; point = context.NextOutside[point])
                        {
                            if (point != apex)
                                orphans.Add(point, pool);
                        }
                        triangle.OutsideHead = -1;
                    }
                    for (int i = 0; i < orphans.Count; ++i)
                        Assign(ref context, orphans[i], ((Span<int>)created.Span).Slice(0, created.Count), pool);
                }
                visible.Dispose(pool);
                stack.Dispose(pool);
                created.Dispose(pool);
                orphans.Dispose(pool);
                pool.Return(ref createdFrom);
                return true;
            }

            /// <summary>
            /// Merges the hull's triangles into flat faces and writes the hull data: from the largest triangle not yet in
            /// a face, neighbours across the face's boundary join it while all their vertices lie within tolerance of the
            /// first triangle's plane (whatever their own normals: a sliver's normal is noise) and the boundary stays a
            /// single loop. Each face's boundary, walked round, is its vertex list.
            /// </summary>
            static bool MergeFaces(ref Context context, double tolerance, BufferPool pool, out HullData hullData)
            {
                hullData = default;
                var count = context.Points.Length;
                var triangleCount = context.Triangles.Count;
                var alive = new QuickList<int>(triangleCount, pool);
                for (int t = 0; t < triangleCount; ++t)
                {
                    if (context.Triangles[t].Alive)
                        alive.AllocateUnsafely() = t;
                }
                //Largest first: sorted on negated twice-areas.
                pool.Take<double>(alive.Count, out var keys);
                for (int i = 0; i < alive.Count; ++i)
                {
                    ref var triangle = ref context.Triangles[alive[i]];
                    double ux = context.X[triangle.B] - context.X[triangle.A], uy = context.Y[triangle.B] - context.Y[triangle.A], uz = context.Z[triangle.B] - context.Z[triangle.A];
                    double vx = context.X[triangle.C] - context.X[triangle.A], vy = context.Y[triangle.C] - context.Y[triangle.A], vz = context.Z[triangle.C] - context.Z[triangle.A];
                    double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
                    keys[i] = -(nx * nx + ny * ny + nz * nz);
                }
                MemoryExtensions.Sort(((Span<double>)keys).Slice(0, alive.Count), ((Span<int>)alive.Span).Slice(0, alive.Count));
                pool.Return(ref keys);

                pool.Take<int>(triangleCount, out var faceOf);
                for (int t = 0; t < triangleCount; ++t)
                    faceOf[t] = -1;
                //Per vertex: the next vertex along the current face's boundary (valid when boundaryStamp matches), and
                //whether it's on the current face.
                pool.Take<int>(count, out var boundaryNext);
                pool.Take<int>(count, out var boundaryStamp);
                pool.Take<int>(count, out var onFaceStamp);
                for (int i = 0; i < count; ++i)
                {
                    boundaryStamp[i] = -1;
                    onFaceStamp[i] = -1;
                }
                var boundaryVertices = new QuickList<int>(32, pool);
                var queue = new QuickList<int>(32, pool);
                var faceStarts = new QuickList<int>(Math.Max(4, alive.Count), pool);
                var faceVertices = new QuickList<int>(Math.Max(12, alive.Count * 3), pool);
                Span<int> vertices = stackalloc int[3];
                var success = true;
                for (int seedIndex = 0; seedIndex < alive.Count && success; ++seedIndex)
                {
                    var seed = alive[seedIndex];
                    if (faceOf[seed] >= 0)
                        continue;
                    var face = faceStarts.Count;
                    var plane = context.Triangles[seed];
                    var planeValid = plane.NX != 0 || plane.NY != 0 || plane.NZ != 0;
                    boundaryVertices.Count = 0;
                    queue.Count = 0;
                    var queueStart = 0;
                    var boundaryCount = 0;
                    Take(ref context, seed);
                    while (planeValid && queueStart < queue.Count)
                    {
                        var t = queue[queueStart++];
                        if (faceOf[t] >= 0)
                            continue;
                        ref var triangle = ref context.Triangles[t];
                        if (triangle.NX * plane.NX + triangle.NY * plane.NY + triangle.NZ * plane.NZ < 0)
                            continue;
                        vertices[0] = triangle.A;
                        vertices[1] = triangle.B;
                        vertices[2] = triangle.C;
                        var flat = true;
                        var shared = 0;
                        var lone = -1;
                        for (int k = 0; k < 3; ++k)
                        {
                            int u = vertices[k], v = vertices[(k + 1) % 3];
                            flat &= Math.Abs(Distance(ref context, plane, u)) <= tolerance;
                            if (boundaryStamp[v] == face && boundaryNext[v] == u)
                            {
                                ++shared;
                                lone = vertices[(k + 2) % 3];
                            }
                        }
                        //The boundary stays one loop: sharing two edges, they're consecutive on it (they meet at a vertex, and
                        //each vertex has one outgoing boundary edge), so they're swapped for the third; sharing one, the
                        //third vertex must be new to the face, or the boundary would pinch there into two loops.
                        if (!flat || shared == 0 || (shared == 1 && onFaceStamp[lone] == face))
                            continue;
                        Take(ref context, t);
                    }
                    //The boundary walked round, from any vertex still on it.
                    var start = -1;
                    for (int i = 0; i < boundaryVertices.Count; ++i)
                    {
                        if (boundaryStamp[boundaryVertices[i]] == face)
                        {
                            start = boundaryVertices[i];
                            break;
                        }
                    }
                    if (start < 0)
                    {
                        success = false;
                        break;
                    }
                    faceStarts.Add(faceVertices.Count, pool);
                    var at = start;
                    var steps = 0;
                    do
                    {
                        faceVertices.Add(at, pool);
                        at = boundaryNext[at];
                        ++steps;
                    }
                    while (at != start && steps <= boundaryCount);
                    if (at != start || steps != boundaryCount)
                    {
                        success = false;
                        break;
                    }

                    void Take(ref Context context, int t)
                    {
                        faceOf[t] = face;
                        ref var triangle = ref context.Triangles[t];
                        //Edges shared with the face cancel, all of them first; the rest join the boundary.
                        var shared0 = boundaryStamp[triangle.B] == face && boundaryNext[triangle.B] == triangle.A;
                        var shared1 = boundaryStamp[triangle.C] == face && boundaryNext[triangle.C] == triangle.B;
                        var shared2 = boundaryStamp[triangle.A] == face && boundaryNext[triangle.A] == triangle.C;
                        onFaceStamp[triangle.A] = face;
                        onFaceStamp[triangle.B] = face;
                        onFaceStamp[triangle.C] = face;
                        if (shared0) { boundaryStamp[triangle.B] = -1; --boundaryCount; }
                        if (shared1) { boundaryStamp[triangle.C] = -1; --boundaryCount; }
                        if (shared2) { boundaryStamp[triangle.A] = -1; --boundaryCount; }
                        if (!shared0) AddBoundary(triangle.A, triangle.B);
                        if (!shared1) AddBoundary(triangle.B, triangle.C);
                        if (!shared2) AddBoundary(triangle.C, triangle.A);
                        for (int k = 0; k < 3; ++k)
                        {
                            var neighbor = triangle.Neighbor(k);
                            if (faceOf[neighbor] < 0)
                                queue.Add(neighbor, pool);
                        }
                    }

                    void AddBoundary(int u, int v)
                    {
                        boundaryStamp[u] = face;
                        boundaryNext[u] = v;
                        boundaryVertices.Add(u, pool);
                        ++boundaryCount;
                    }
                }
                if (success)
                {
                    //The hull's vertices in order of first use, mapped to the original points.
                    pool.Take(faceStarts.Count, out hullData.FaceStartIndices);
                    pool.Take(faceVertices.Count, out hullData.FaceVertexIndices);
                    var hullToOriginal = new QuickList<int>(count, pool);
                    //(onFaceStamp reused as the original to hull index mapping)
                    for (int i = 0; i < count; ++i)
                        onFaceStamp[i] = -1;
                    for (int i = 0; i < faceStarts.Count; ++i)
                        hullData.FaceStartIndices[i] = faceStarts[i];
                    for (int i = 0; i < faceVertices.Count; ++i)
                    {
                        ref var hullIndex = ref onFaceStamp[faceVertices[i]];
                        if (hullIndex < 0)
                        {
                            hullIndex = hullToOriginal.Count;
                            hullToOriginal.AllocateUnsafely() = faceVertices[i];
                        }
                        hullData.FaceVertexIndices[i] = hullIndex;
                    }
                    pool.Take(hullToOriginal.Count, out hullData.OriginalVertexMapping);
                    hullToOriginal.Span.CopyTo(0, hullData.OriginalVertexMapping, 0, hullToOriginal.Count);
                    hullToOriginal.Dispose(pool);
                }
                alive.Dispose(pool);
                pool.Return(ref faceOf);
                pool.Return(ref boundaryNext);
                pool.Return(ref boundaryStamp);
                pool.Return(ref onFaceStamp);
                boundaryVertices.Dispose(pool);
                queue.Dispose(pool);
                faceStarts.Dispose(pool);
                faceVertices.Dispose(pool);
                return success;
            }
        }
    }
}
