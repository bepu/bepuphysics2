using BepuPhysics.Collidables;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Numerics;
using Xunit;

namespace DemoTests
{
    /// <summary>
    /// Checks ConvexHullHelper.ComputeHull produces a valid convex polytope for point sets that have caused trouble
    /// (the point sets are in ConvexHullTests.PointSets.cs).
    /// </summary>
    public partial class ConvexHullTests
    {
        public static IEnumerable<object[]> PointSets()
        {
            foreach (var name in VolumetricSetNames)
                yield return new object[] { name };
        }

        /// <summary>
        /// Checks hull data describes a closed convex polytope holding every input point:
        /// 1. Closed: the faces' area vectors sum to zero, and the volume they enclose is the same measured from two
        ///    different origins (an open or overlapping surface encloses a volume that depends on the origin). This
        ///    allows T-junctions, where one face keeps a vertex lying on its edge that the neighbouring face doesn't.
        /// 2. Each face's vertices lie on its plane.
        /// 3. No input point is outside any face's plane.
        /// Returns null when valid, else what's wrong.
        /// </summary>
        public static string Validate(Span<Vector3> points, in HullData hullData, float tolerance)
        {
            var faceCount = hullData.FaceStartIndices.Length;
            if (faceCount < 4)
                return $"only {faceCount} faces";
            var min = new Vector3(float.MaxValue);
            var max = new Vector3(float.MinValue);
            foreach (var point in points)
            {
                min = Vector3.Min(min, point);
                max = Vector3.Max(max, point);
            }
            var origin0 = (min + max) * 0.5f;
            var origin1 = min;
            double areaX = 0, areaY = 0, areaZ = 0, totalArea = 0, volume0 = 0, volume1 = 0;
            var planes = new (Vector3 Normal, float Offset)[faceCount];
            for (int faceIndex = 0; faceIndex < faceCount; ++faceIndex)
            {
                hullData.GetFace(faceIndex, out var face);
                if (face.VertexCount < 3)
                    return $"face {faceIndex} has {face.VertexCount} vertices";
                var pivot = points[face[0]];
                var normal = Vector3.Zero;
                for (int i = 2; i < face.VertexCount; ++i)
                {
                    var b = points[face[i - 1]];
                    var c = points[face[i]];
                    normal += Vector3.Cross(b - pivot, c - pivot);
                    // (sums in doubles)
                    double ux = (double)b.X - pivot.X, uy = (double)b.Y - pivot.Y, uz = (double)b.Z - pivot.Z;
                    double vx = (double)c.X - pivot.X, vy = (double)c.Y - pivot.Y, vz = (double)c.Z - pivot.Z;
                    double cx = uy * vz - uz * vy, cy = uz * vx - ux * vz, cz = ux * vy - uy * vx;
                    areaX += cx; areaY += cy; areaZ += cz;
                    totalArea += Math.Sqrt(cx * cx + cy * cy + cz * cz);
                    volume0 += cx * ((double)pivot.X - origin0.X) + cy * ((double)pivot.Y - origin0.Y) + cz * ((double)pivot.Z - origin0.Z);
                    volume1 += cx * ((double)pivot.X - origin1.X) + cy * ((double)pivot.Y - origin1.Y) + cz * ((double)pivot.Z - origin1.Z);
                }
                var length = normal.Length();
                if (!(length > 0))
                    return $"face {faceIndex} has no area";
                normal /= length;
                var offset = float.MinValue;
                for (int i = 0; i < face.VertexCount; ++i)
                    offset = MathF.Max(offset, Vector3.Dot(normal, points[face[i]]));
                planes[faceIndex] = (normal, offset);
                for (int i = 0; i < face.VertexCount; ++i)
                {
                    var distance = offset - Vector3.Dot(normal, points[face[i]]);
                    if (distance > tolerance)
                        return $"face {faceIndex} vertex {face[i]} is {distance} off the face's plane";
                }
            }
            var openness = Math.Sqrt(areaX * areaX + areaY * areaY + areaZ * areaZ);
            if (openness > totalArea * 1e-3)
                return $"the faces' area vectors don't cancel ({openness / totalArea:P3} of the total area): the surface isn't closed";
            if (!(volume0 > 0))
                return $"the volume is {volume0 / 6} (faces wound inward or missing)";
            if (Math.Abs(volume0 - volume1) > volume0 * 1e-3)
                return $"the volume depends on where it's measured from ({volume0 / 6} and {volume1 / 6}): the surface is open or overlaps itself";
            for (int i = 0; i < points.Length; ++i)
            {
                for (int faceIndex = 0; faceIndex < faceCount; ++faceIndex)
                {
                    var outside = Vector3.Dot(planes[faceIndex].Normal, points[i]) - planes[faceIndex].Offset;
                    if (outside > tolerance)
                        return $"point {i} is {outside} outside face {faceIndex}";
                }
            }
            return null;
        }

        static string ComputeAndValidate(Vector3[] points)
        {
            var pool = new BufferPool();
            ConvexHullHelper.ComputeHull(points, pool, out var hullData);
            var min = new Vector3(float.MaxValue);
            var max = new Vector3(float.MinValue);
            foreach (var point in points)
            {
                min = Vector3.Min(min, point);
                max = Vector3.Max(max, point);
            }
            var problem = Validate(points, hullData, (max - min).Length() * 1e-4f);
            hullData.Dispose(pool);
            pool.Clear();
            return problem;
        }

        /// <summary>
        /// Point sets the builder handles, to show the validation passes good hulls: a cube's corners, points on a sphere,
        /// points scattered in a box
        /// </summary>
        [Fact]
        public void ControlHullsAreValid()
        {
            var cube = new Vector3[8];
            for (int i = 0; i < 8; ++i)
                cube[i] = new Vector3((i & 1) == 0 ? -1 : 1, (i & 2) == 0 ? -1 : 1, (i & 4) == 0 ? -1 : 1);
            Assert.Null(ComputeAndValidate(cube));
            var random = new Random(5);
            var sphere = new Vector3[200];
            for (int i = 0; i < sphere.Length; ++i)
                sphere[i] = Vector3.Normalize(new Vector3(random.NextSingle() * 2 - 1, random.NextSingle() * 2 - 1, random.NextSingle() * 2 - 1));
            Assert.Null(ComputeAndValidate(sphere));
            var box = new Vector3[200];
            for (int i = 0; i < box.Length; ++i)
                box[i] = new Vector3(random.NextSingle() * 3, random.NextSingle(), random.NextSingle() * 2);
            Assert.Null(ComputeAndValidate(box));
        }

        [Theory]
        [MemberData(nameof(PointSets))]
        public void HullIsClosedAndContainsItsPoints(string name)
        {
            var points = Load(name);
            var pool = new BufferPool();
            ConvexHullHelper.ComputeHull(points, pool, out var hullData);
            var min = new Vector3(float.MaxValue);
            var max = new Vector3(float.MinValue);
            foreach (var point in points)
            {
                min = Vector3.Min(min, point);
                max = Vector3.Max(max, point);
            }
            // Same scale as ComputeHull's own coplanarity slab
            var tolerance = (max - min).Length() * 1e-4f;
            var problem = Validate(points, hullData, tolerance);
            hullData.Dispose(pool);
            pool.Clear();
            Assert.True(problem == null, $"{name}: {problem}");
        }
    
        /// <summary>
        /// The runtime shape made from each point set (CreateShape) has a positive inertia and bounding planes that hold
        /// every input point.
        /// </summary>
        [Theory]
        [MemberData(nameof(PointSets))]
        public void ShapeHasPositiveInertiaAndContainsItsPoints(string name)
        {
            var points = Load(name);
            var pool = new BufferPool();
            Assert.True(ConvexHullHelper.CreateShape(points, pool, out var center, out ConvexHull hull), $"{name}: no shape");
            var min = new Vector3(float.MaxValue);
            var max = new Vector3(float.MinValue);
            foreach (var point in points)
            {
                min = Vector3.Min(min, point);
                max = Vector3.Max(max, point);
            }
            var tolerance = (max - min).Length() * 1e-4f;
            var inverseInertia = hull.ComputeInertia(1).InverseInertiaTensor;
            var worst = 0f;
            for (int faceIndex = 0; faceIndex < hull.FaceToVertexIndicesStart.Length; ++faceIndex)
            {
                BepuUtilities.BundleIndexing.GetBundleIndices(faceIndex, out var bundleIndex, out var innerIndex);
                ref var planes = ref hull.BoundingPlanes[bundleIndex];
                var normal = new Vector3(planes.Normal.X[innerIndex], planes.Normal.Y[innerIndex], planes.Normal.Z[innerIndex]);
                foreach (var point in points)
                    worst = MathF.Max(worst, Vector3.Dot(normal, point - center) - planes.Offset[innerIndex]);
            }
            hull.Dispose(pool);
            pool.Clear();
            Assert.True(inverseInertia.XX > 0 && inverseInertia.YY > 0 && inverseInertia.ZZ > 0, $"{name}: inverse inertia diagonal {inverseInertia.XX}, {inverseInertia.YY}, {inverseInertia.ZZ}");
            Assert.True(worst <= tolerance, $"{name}: a point is {worst} outside the shape's bounding planes");
        }
    
        /// <summary>
        /// The demo's generated point sets: its random cloud (seed 5) and its "hell cube", a cube's six faces covered in
        /// grids of points, turned off the axes, so nearly every point is exactly coplanar with thousands of others.
        /// </summary>
        public static IEnumerable<object[]> GeneratedPointSets()
        {
            yield return new object[] { "random 50", 0 };
            yield return new object[] { "hell cube 10", 10 };
            yield return new object[] { "hell cube 50", 50 };
            yield return new object[] { "hell cube 200", 200 };
        }

        public static Vector3[] Generate(int hellCubeWidth)
        {
            if (hellCubeWidth == 0)
            {
                var points = new Vector3[50];
                var random = new Random(5);
                for (int i = 0; i < points.Length; ++i)
                    points[i] = new(3 * random.NextSingle(), 1 * random.NextSingle(), 3 * random.NextSingle());
                return points;
            }
            //As ConvexHullTestDemo.CreateHellCube.
            var facePointCount = hellCubeWidth * hellCubeWidth;
            var buffer = new Vector3[facePointCount * 6];
            var size = 8f;
            var halfSize = size / 2;
            var step = size / (hellCubeWidth - 1);
            BepuUtilities.Matrix3x3.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1f, 1, 1f)), float.Pi / 2, out var cubeTransform);
            BepuUtilities.Matrix3x3.CreateFromAxisAngle(new Vector3(0, 1, 0), float.Pi / 2, out var zFace);
            zFace *= cubeTransform;
            BepuUtilities.Matrix3x3.CreateFromAxisAngle(new Vector3(1, 0, 0), -float.Pi / 2, out var yFace);
            yFace *= cubeTransform;
            void Face(Span<Vector3> facePoints, BepuUtilities.Matrix3x3 transform, float localFaceOffset)
            {
                var offset = step * (hellCubeWidth - 1) * 0.5f;
                for (int i = 0; i < hellCubeWidth; ++i)
                {
                    for (int j = 0; j < hellCubeWidth; ++j)
                    {
                        BepuUtilities.Matrix3x3.Transform(new Vector3(i * step - offset, j * step - offset, localFaceOffset), transform, out var world);
                        facePoints[i * hellCubeWidth + j] = world;
                    }
                }
            }
            Face(buffer.AsSpan(facePointCount * 0, facePointCount), cubeTransform, halfSize);
            Face(buffer.AsSpan(facePointCount * 1, facePointCount), zFace, halfSize);
            Face(buffer.AsSpan(facePointCount * 2, facePointCount), cubeTransform, -halfSize);
            Face(buffer.AsSpan(facePointCount * 3, facePointCount), zFace, -halfSize);
            Face(buffer.AsSpan(facePointCount * 4, facePointCount), yFace, halfSize);
            Face(buffer.AsSpan(facePointCount * 5, facePointCount), yFace, -halfSize);
            return buffer;
        }

        [Theory]
        [MemberData(nameof(GeneratedPointSets))]
        public void GeneratedHullIsClosedAndContainsItsPoints(string name, int hellCubeWidth)
        {
            var problem = ComputeAndValidate(Generate(hellCubeWidth));
            Assert.True(problem == null, $"{name}: {problem}");
        }

        /// <summary>
        /// Flat point sets (no volume) give the same result as gift wrapping, which ComputeHull falls back to for them.
        /// </summary>
        [Theory]
        [InlineData(nameof(FlatPlaneish))]
        [InlineData(nameof(FlatDistantPlane))]
        public void FlatPointSetsMatchGiftWrapping(string name)
        {
            var points = Load(name);
            var pool = new BufferPool();
            ConvexHullHelper.ComputeHull(points, pool, out var hull);
            ConvexHullHelper.ComputeHullGiftWrapping(points, pool, out var giftWrapped);
            Assert.Equal(giftWrapped.OriginalVertexMapping.Length, hull.OriginalVertexMapping.Length);
            Assert.Equal(giftWrapped.FaceStartIndices.Length, hull.FaceStartIndices.Length);
            for (int i = 0; i < hull.OriginalVertexMapping.Length; ++i)
                Assert.Equal(giftWrapped.OriginalVertexMapping[i], hull.OriginalVertexMapping[i]);
            Assert.False(ConvexHullHelper.CreateShape(points, pool, out _, out ConvexHull _), "a flat point set made a shape");
            hull.Dispose(pool);
            giftWrapped.Dispose(pool);
            pool.Clear();
        }
    }
}
