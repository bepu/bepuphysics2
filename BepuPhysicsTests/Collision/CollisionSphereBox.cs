using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using System.Numerics;

namespace BepuPhysicsTests.Collision;

public class CollisionSphereBox : CollisionBase<Sphere, SphereWide, Box, BoxWide, Convex1ContactManifoldWide, SphereBoxTester>
{
    protected override Sphere CreateShapeA() => new(1f);
    protected override Box CreateShapeB() => new(1, 2, 3);

    [Fact]
    public void Concentric_Colliding()
    {
        AssertColliding(new Vector3(0, 0, 0));
    }

    [Fact]
    public void OffsetAlongX_Colliding()
    {
        AssertColliding(new Vector3(1.25f, 0, 0));
    }

    [Fact]
    public void OffsetAlongX_Separated()
    {
        AssertSeparated(new Vector3(1.6f, 0, 0));
    }

    [Fact]
    public void OffsetAlongY_Colliding()
    {
        AssertColliding(new Vector3(0, 1.75f, 0));
    }

    [Fact]
    public void OffsetAlongY_Separated()
    {
        AssertSeparated(new Vector3(0, 2.1f, 0));
    }

    [Fact]
    public void OffsetAlongZ_Colliding()
    {
        AssertColliding(new Vector3(0, 0, 2.25f));
    }

    [Fact]
    public void OffsetAlongZ_Separated()
    {
        AssertSeparated(new Vector3(0, 0, 2.6f));
    }

    [Fact]
    public void OffsetAlongXYZ_Colliding()
    {
        AssertColliding(new Vector3(0.75f, 0.75f, 1.75f));
    }

    [Fact]
    public void OffsetAlongXYZ_Separated()
    {
        AssertSeparated(new Vector3(1.1f, 2.1f, 3.1f));
    }

    [Fact]
    public void Rotated_Colliding()
    {
        var orientationB = Quaternion.CreateFromYAxisAngle(float.Pi * 0.25f);
        AssertColliding(new Vector3(1.5f, 0, 0), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void Rotated_Separated()
    {
        var orientationB = Quaternion.CreateFromYAxisAngle(float.Pi * 0.25f);
        AssertSeparated(new Vector3(2.6f, 0, 0), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void OverlappingPair_HasSingleContact()
    {
        AssertContactCount(1, new Vector3(1.25f, 0, 0));
        AssertContactCount(1, Vector3.Zero);
    }

    [Theory]
    [InlineData(1.25f, 0, 0, -1, 0, 0)]
    [InlineData(-1.25f, 0, 0, 1, 0, 0)]
    [InlineData(0, 1.75f, 0, 0, -1, 0)]
    [InlineData(0, -1.75f, 0, 0, 1, 0)]
    [InlineData(0, 0, 2.25f, 0, 0, -1)]
    [InlineData(0, 0, -2.25f, 0, 0, 1)]
    public void FaceRegion_NormalDepthAndContactPosition(float offsetX, float offsetY, float offsetZ, float normalX, float normalY, float normalZ)
    {
        //The sphere is 0.75 away from a face, so depth is 0.25 and the contact is at the midpoint of the overlap.
        var offset = new Vector3(offsetX, offsetY, offsetZ);
        var normal = new Vector3(normalX, normalY, normalZ);
        AssertNormal(normal, offset);
        AssertMaximumDepth(0.25f, offset);
        AssertHasContactAt(normal * (0.25f * 0.5f - 1f), offset);
    }

    [Fact]
    public void FaceRegion_OffCenterOnFace_KeepsFaceNormal()
    {
        AssertNormal(new Vector3(-1, 0, 0), new Vector3(1.25f, 0.4f, -1.2f));
        AssertMaximumDepth(0.25f, new Vector3(1.25f, 0.4f, -1.2f));
    }

    [Fact]
    public void EdgeRegion_NormalPointsAwayFromEdge()
    {
        //Closest point on the box is the edge at (0.5, 1, z). Sphere-to-edge offset is (0.5, 0.5, 0).
        var offset = new Vector3(1f, 1.5f, 0);
        AssertNormal(new Vector3(-1, -1, 0), offset);
        AssertMaximumDepth(1f - float.Sqrt(0.5f), offset);
    }

    [Theory]
    [InlineData(1f, 0, 1.75f, -0.5f, 0, -0.25f)]
    [InlineData(0, 1.4f, 2.2f, 0, -0.4f, -0.7f)]
    [InlineData(-1f, -1.5f, 0, 1, 1, 0)]
    public void EdgeRegion_AllEdgeOrientations(float offsetX, float offsetY, float offsetZ, float normalX, float normalY, float normalZ)
    {
        var manifold = Collide(new Vector3(offsetX, offsetY, offsetZ));
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
        AssertEqual(Vector3.Normalize(new Vector3(normalX, normalY, normalZ)), manifold.Normal, 1e-3f);
    }

    [Fact]
    public void CornerRegion_NormalPointsAwayFromCorner()
    {
        //Closest point is the corner (0.5, 1, 1.5); the sphere-to-corner offset is (0.3, 0.6, 0.5).
        var offset = new Vector3(0.8f, 1.6f, 2f);
        AssertNormal(new Vector3(-0.3f, -0.6f, -0.5f), offset);
        AssertMaximumDepth(1f - float.Sqrt(0.3f * 0.3f + 0.6f * 0.6f + 0.5f * 0.5f), offset);
    }

    [Fact]
    public void CornerRegion_AllOctants()
    {
        for (int xSign = -1; xSign <= 1; xSign += 2)
        {
            for (int ySign = -1; ySign <= 1; ySign += 2)
            {
                for (int zSign = -1; zSign <= 1; zSign += 2)
                {
                    var offset = new Vector3(0.8f * xSign, 1.6f * ySign, 2f * zSign);
                    AssertNormal(new Vector3(-0.3f * xSign, -0.6f * ySign, -0.5f * zSign), offset);
                }
            }
        }
    }

    [Fact]
    public void CornerRegion_BeyondRadius_Separated()
    {
        //Corner distance is sqrt(1.25 + 1 + 1) > 1.
        AssertSeparated(new Vector3(1.0f, 2.0f, 2.5f));
    }

    [Fact]
    public void Concentric_UsesLocalXAxisAtTie()
    {
        //Every axis is a candidate only when the center is at the origin; the smallest half extent (X) wins.
        AssertNormal(new Vector3(-1, 0, 0), Vector3.Zero);
        AssertMaximumDepth(1.5f, Vector3.Zero);
    }

    [Fact]
    public void Inside_ExitsThroughShortestAxis()
    {
        //Half extents are (0.5, 1, 1.5). Normals point from the box to the sphere, and a sphere at -offset is nearest the opposite face.
        AssertNormal(new Vector3(-1, 0, 0), new Vector3(0.1f, 0, 0));
        AssertMaximumDepth(1.4f, new Vector3(0.1f, 0, 0));
        AssertNormal(new Vector3(1, 0, 0), new Vector3(-0.1f, 0, 0));

        AssertNormal(new Vector3(0, -1, 0), new Vector3(0, 0.8f, 0));
        AssertMaximumDepth(1.2f, new Vector3(0, 0.8f, 0));
        AssertNormal(new Vector3(0, 1, 0), new Vector3(0, -0.8f, 0));

        AssertNormal(new Vector3(0, 0, 1), new Vector3(0, 0, -1.3f));
        AssertMaximumDepth(1.2f, new Vector3(0, 0, -1.3f));
        AssertNormal(new Vector3(0, 0, -1), new Vector3(0, 0, 1.3f));
    }

    [Fact]
    public void CenterExactlyOnFace_HasValidManifold()
    {
        var manifold = Collide(new Vector3(0.5f, 0, 0));
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
        AssertEqual(new Vector3(-1, 0, 0), manifold.Normal);
        Assert.InRange(manifold.Contact0.Depth, 1f - 1e-4f, 1f + 1e-4f);
    }

    [Fact]
    public void CenterOnCornerOrEdge_HasValidManifold()
    {
        AssertValidManifold(Collide(new Vector3(0.5f, 1f, 1.5f)));
        AssertValidManifold(Collide(new Vector3(0.5f, 1f, 0f)));
        AssertContactCount(1, new Vector3(-0.5f, -1f, -1.5f));
    }

    [Fact]
    public void Touching_IsColliding()
    {
        AssertMaximumDepth(0f, new Vector3(1.5f, 0, 0), speculativeMargin: 0.01f);
    }

    [Fact]
    public void SeparatedWithinMargin_ProducesSpeculativeContact()
    {
        AssertSeparated(new Vector3(1.6f, 0, 0));
        AssertColliding(new Vector3(1.6f, 0, 0), 0.2f);
        AssertMaximumDepth(-0.1f, new Vector3(1.6f, 0, 0), speculativeMargin: 0.2f);
        AssertNormal(new Vector3(-1, 0, 0), new Vector3(1.6f, 0, 0), speculativeMargin: 0.2f);
    }

    [Fact]
    public void SeparatedBeyondMargin_Separated()
    {
        AssertSeparated(new Vector3(1.8f, 0, 0), 0.2f);
        AssertSeparated(new Vector3(0, 2.3f, 0), 0.2f);
        AssertSeparated(new Vector3(0, 0, 2.8f), 0.2f);
    }

    [Fact]
    public void SpeculativeContactNearEdge_UsesEdgeNormal()
    {
        //Sphere-to-edge distance is sqrt(0.5) * 1.6 > 1 but within margin 0.2 of touching (depth -0.13).
        var offset = new Vector3(0.5f + 1.13f / float.Sqrt(2f), 1f + 1.13f / float.Sqrt(2f), 0);
        AssertNormal(new Vector3(-1, -1, 0), offset, speculativeMargin: 0.2f);
        AssertSeparated(offset);
    }

    [Fact]
    public void RotatedBox_FaceContactUsesWorldSpaceNormal()
    {
        //A 90 degree yaw moves the half width (0.5) onto the world Z axis and the half length (1.5) onto the world X axis.
        var orientationB = Quaternion.CreateFromYAxisAngle(float.Pi * 0.5f);
        AssertNormal(new Vector3(0, 0, -1), new Vector3(0, 0, 1.25f), Quaternion.Identity, orientationB);
        AssertMaximumDepth(0.25f, new Vector3(0, 0, 1.25f), Quaternion.Identity, orientationB);
        AssertNormal(new Vector3(-1, 0, 0), new Vector3(1.75f, 0, 0), Quaternion.Identity, orientationB);
        AssertMaximumDepth(0.75f, new Vector3(1.75f, 0, 0), Quaternion.Identity, orientationB);
        AssertSeparated(new Vector3(0, 0, 1.6f), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void RotatedBox_PitchedFace_NormalFollowsFace()
    {
        //Pitching about Z by 45 degrees tilts the +X face normal toward +Y.
        var orientationB = Quaternion.CreateFromZAxisAngle(float.Pi * 0.25f);
        var faceNormalB = Vector3.Transform(Vector3.UnitX, orientationB);
        var secondShapeOffset = Vector3.Transform(new Vector3(0.5f + 0.75f, 0, 0), orientationB);
        AssertNormal(-faceNormalB, secondShapeOffset, Quaternion.Identity, orientationB);
        AssertMaximumDepth(0.25f, secondShapeOffset, Quaternion.Identity, orientationB);
    }

    [Fact]
    public void SphereOrientation_DoesNotAffectResult()
    {
        var orientationA = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 2, 3)), 1.1f);
        AssertNormal(new Vector3(-1, 0, 0), new Vector3(1.25f, 0, 0), orientationA, Quaternion.Identity);
        AssertMaximumDepth(0.25f, new Vector3(1.25f, 0, 0), orientationA, Quaternion.Identity);
    }

    [Fact]
    public void BundleResult_MatchesSingleLane()
    {
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(-2, 1, 0.5f)), 2.3f);
        AssertBundleConsistent(new Vector3(1.25f, 0.2f, -0.3f), Quaternion.Identity, Quaternion.Identity);
        AssertBundleConsistent(new Vector3(1.0f, 1.5f, 0f), Quaternion.Identity, orientationB);
        AssertBundleConsistent(new Vector3(0.1f, 0f, 0f), Quaternion.Identity, orientationB);
        AssertBundleConsistent(new Vector3(1.6f, 0, 0), Quaternion.Identity, Quaternion.Identity, 0.2f);
    }

    [Fact]
    public void RotatingWholeConfiguration_RotatesNormal()
    {
        var world = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(0.3f, 1, -0.6f)), 0.9f);
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), 0.7f);
        AssertRotationInvariant(new Vector3(0.9f, 1.4f, 0.3f), Quaternion.Identity, orientationB, world);
        AssertRotationInvariant(new Vector3(0.1f, 0.3f, 0.2f), Quaternion.Identity, orientationB, world);
    }

    [Fact]
    public void DifferentSizes_DepthAndContact()
    {
        var manifold = Collide(new Sphere(0.5f), new Box(2, 2, 2), 0f, new Vector3(1.25f, 0, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
        Assert.InRange(manifold.Contact0.Depth, 0.25f - 1e-4f, 0.25f + 1e-4f);
        //Overlap spans x in [0.25, 0.5], so the midpoint is 0.375 from the sphere center.
        AssertEqual(new Vector3(0.375f, 0, 0), manifold.Contact0.Offset);
        AssertEqual(new Vector3(-1, 0, 0), manifold.Normal);
    }

    [Fact]
    public void DegenerateThinBox_StillProducesValidManifold()
    {
        var manifold = Collide(new Sphere(1f), new Box(0.001f, 2, 2), 0f, new Vector3(0.5f, 0.2f, 0.1f), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
    }
}
