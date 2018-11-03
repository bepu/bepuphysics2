using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.Constraints;
using System.Numerics;
using Quaternion = BepuUtilities.Quaternion;
using BepuUtilities;
using BepuPhysics.Constraints.Contact;
using System.Runtime.CompilerServices;
using System;

namespace DemoRenderer.Constraints
{
    public static class ContactLines
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void BuildOrthnormalBasis(in Vector3 normal, out Vector3 t1, out Vector3 t2)
        {
            //No frisvad or friends here- just want a simple and consistent basis with only one singularity.
            //Could be faster if needed.
            t1 = Vector3.Cross(normal, new Vector3(1, -1, 1));
            var lengthSquared = t1.LengthSquared();
            if (lengthSquared < 1e-8f)
            {
                t1 = Vector3.Cross(normal, new Vector3(-1, 1, 1));
                lengthSquared = t1.LengthSquared();
            }
            t1 /= MathF.Sqrt(lengthSquared);
            t2 = Vector3.Cross(normal, t1);
        }

        public static void Add(in RigidPose poseA, ref Vector3Wide offsetAWide, ref Vector3Wide normalWide, ref Vector<float> depthWide, int innerIndex, 
            in Vector3 tint, ref QuickList<LineInstance, Buffer<LineInstance>> lines)
        {
            Vector3Wide.ReadSlot(ref offsetAWide, innerIndex, out var offsetA);
            Vector3Wide.ReadSlot(ref normalWide, innerIndex, out var normal);
            var depth = depthWide[innerIndex];
            var contactPosition = offsetA + poseA.Position;
            BuildOrthnormalBasis(normal, out var t1, out var t2);
            var packedColor = Helpers.PackColor(tint * (depth >= 0 ? new Vector3(0,1,0) : new Vector3(0.15f, 0.25f, 0.15f)));
            t1 *= 0.5f;
            t2 *= 0.5f;
            var t1Line = new LineInstance(contactPosition - t1, contactPosition + t1, packedColor, 0);
            lines.AddUnsafely(ref t1Line);
            var t2Line = new LineInstance(contactPosition - t2, contactPosition + t2, packedColor, 0);
            lines.AddUnsafely(ref t2Line);

        }
    }

    //This is a little goofy. We actually used text templates in the engine proper to handle similar situations, but the visualizer is quite a bit less important.
    //If you do find yourself needing to refactor this in any significant way, you probably should just bite the bullet and codegen it.

    struct Contact1OneBodyLineExtractor : IConstraintLineExtractor<Contact1OneBodyPrestepData>
    {
        public int LinesPerConstraint => 2;

        public unsafe void ExtractLines(ref Contact1OneBodyPrestepData prestepBundle, int innerIndex, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance, Buffer<LineInstance>> lines)
        {
            var poseA = bodies.Sets[setIndex].Poses[bodyIndices[0]];
            ContactLines.Add(poseA, ref prestepBundle.OffsetA0, ref prestepBundle.Normal, ref prestepBundle.PenetrationDepth0, innerIndex, tint, ref lines);            
        }
    }
    struct Contact2OneBodyLineExtractor : IConstraintLineExtractor<Contact2OneBodyPrestepData>
    {
        public int LinesPerConstraint => 4;

        public unsafe void ExtractLines(ref Contact2OneBodyPrestepData prestepBundle, int innerIndex, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance, Buffer<LineInstance>> lines)
        {
            var poseA = bodies.Sets[setIndex].Poses[bodyIndices[0]];
            ContactLines.Add(poseA, ref prestepBundle.OffsetA0, ref prestepBundle.Normal, ref prestepBundle.PenetrationDepth0, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.OffsetA1, ref prestepBundle.Normal, ref prestepBundle.PenetrationDepth1, innerIndex, tint, ref lines);
        }
    }
    struct Contact3OneBodyLineExtractor : IConstraintLineExtractor<Contact3OneBodyPrestepData>
    {
        public int LinesPerConstraint => 6;

        public unsafe void ExtractLines(ref Contact3OneBodyPrestepData prestepBundle, int innerIndex, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance, Buffer<LineInstance>> lines)
        {
            var poseA = bodies.Sets[setIndex].Poses[bodyIndices[0]];
            ContactLines.Add(poseA, ref prestepBundle.OffsetA0, ref prestepBundle.Normal, ref prestepBundle.PenetrationDepth0, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.OffsetA1, ref prestepBundle.Normal, ref prestepBundle.PenetrationDepth1, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.OffsetA2, ref prestepBundle.Normal, ref prestepBundle.PenetrationDepth2, innerIndex, tint, ref lines);
        }
    }
    struct Contact4OneBodyLineExtractor : IConstraintLineExtractor<Contact4OneBodyPrestepData>
    {
        public int LinesPerConstraint => 8;

        public unsafe void ExtractLines(ref Contact4OneBodyPrestepData prestepBundle, int innerIndex, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance, Buffer<LineInstance>> lines)
        {
            var poseA = bodies.Sets[setIndex].Poses[bodyIndices[0]];
            ContactLines.Add(poseA, ref prestepBundle.OffsetA0, ref prestepBundle.Normal, ref prestepBundle.PenetrationDepth0, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.OffsetA1, ref prestepBundle.Normal, ref prestepBundle.PenetrationDepth1, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.OffsetA2, ref prestepBundle.Normal, ref prestepBundle.PenetrationDepth2, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.OffsetA3, ref prestepBundle.Normal, ref prestepBundle.PenetrationDepth3, innerIndex, tint, ref lines);
        }
    }

    struct Contact1LineExtractor : IConstraintLineExtractor<Contact1PrestepData>
    {
        public int LinesPerConstraint => 2;

        public unsafe void ExtractLines(ref Contact1PrestepData prestepBundle, int innerIndex, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance, Buffer<LineInstance>> lines)
        {
            var poseA = bodies.Sets[setIndex].Poses[bodyIndices[0]];
            ContactLines.Add(poseA, ref prestepBundle.OffsetA0, ref prestepBundle.Normal, ref prestepBundle.PenetrationDepth0, innerIndex, tint, ref lines);
        }
    }
    struct Contact2LineExtractor : IConstraintLineExtractor<Contact2PrestepData>
    {
        public int LinesPerConstraint => 4;

        public unsafe void ExtractLines(ref Contact2PrestepData prestepBundle, int innerIndex, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance, Buffer<LineInstance>> lines)
        {
            var poseA = bodies.Sets[setIndex].Poses[bodyIndices[0]];
            ContactLines.Add(poseA, ref prestepBundle.OffsetA0, ref prestepBundle.Normal, ref prestepBundle.PenetrationDepth0, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.OffsetA1, ref prestepBundle.Normal, ref prestepBundle.PenetrationDepth1, innerIndex, tint, ref lines);
        }
    }
    struct Contact3LineExtractor : IConstraintLineExtractor<Contact3PrestepData>
    {
        public int LinesPerConstraint => 6;

        public unsafe void ExtractLines(ref Contact3PrestepData prestepBundle, int innerIndex, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance, Buffer<LineInstance>> lines)
        {
            var poseA = bodies.Sets[setIndex].Poses[bodyIndices[0]];
            ContactLines.Add(poseA, ref prestepBundle.OffsetA0, ref prestepBundle.Normal, ref prestepBundle.PenetrationDepth0, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.OffsetA1, ref prestepBundle.Normal, ref prestepBundle.PenetrationDepth1, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.OffsetA2, ref prestepBundle.Normal, ref prestepBundle.PenetrationDepth2, innerIndex, tint, ref lines);
        }
    }
    struct Contact4LineExtractor : IConstraintLineExtractor<Contact4PrestepData>
    {
        public int LinesPerConstraint => 8;

        public unsafe void ExtractLines(ref Contact4PrestepData prestepBundle, int innerIndex, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance, Buffer<LineInstance>> lines)
        {
            var poseA = bodies.Sets[setIndex].Poses[bodyIndices[0]];
            ContactLines.Add(poseA, ref prestepBundle.OffsetA0, ref prestepBundle.Normal, ref prestepBundle.PenetrationDepth0, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.OffsetA1, ref prestepBundle.Normal, ref prestepBundle.PenetrationDepth1, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.OffsetA2, ref prestepBundle.Normal, ref prestepBundle.PenetrationDepth2, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.OffsetA3, ref prestepBundle.Normal, ref prestepBundle.PenetrationDepth3, innerIndex, tint, ref lines);
        }
    }


    struct Contact2NonconvexOneBodyLineExtractor : IConstraintLineExtractor<Contact2NonconvexOneBodyPrestepData>
    {
        public int LinesPerConstraint => 4;

        public unsafe void ExtractLines(ref Contact2NonconvexOneBodyPrestepData prestepBundle, int innerIndex, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance, Buffer<LineInstance>> lines)
        {
            var poseA = bodies.Sets[setIndex].Poses[bodyIndices[0]];
            ContactLines.Add(poseA, ref prestepBundle.Contact0.Offset, ref prestepBundle.Contact0.Normal, ref prestepBundle.Contact0.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact1.Offset, ref prestepBundle.Contact1.Normal, ref prestepBundle.Contact1.Depth, innerIndex, tint, ref lines);
        }
    }
    struct Contact3NonconvexOneBodyLineExtractor : IConstraintLineExtractor<Contact3NonconvexOneBodyPrestepData>
    {
        public int LinesPerConstraint => 6;

        public unsafe void ExtractLines(ref Contact3NonconvexOneBodyPrestepData prestepBundle, int innerIndex, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance, Buffer<LineInstance>> lines)
        {
            var poseA = bodies.Sets[setIndex].Poses[bodyIndices[0]];
            ContactLines.Add(poseA, ref prestepBundle.Contact0.Offset, ref prestepBundle.Contact0.Normal, ref prestepBundle.Contact0.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact1.Offset, ref prestepBundle.Contact1.Normal, ref prestepBundle.Contact1.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact2.Offset, ref prestepBundle.Contact2.Normal, ref prestepBundle.Contact2.Depth, innerIndex, tint, ref lines);
        }
    }
    struct Contact4NonconvexOneBodyLineExtractor : IConstraintLineExtractor<Contact4NonconvexOneBodyPrestepData>
    {
        public int LinesPerConstraint => 8;

        public unsafe void ExtractLines(ref Contact4NonconvexOneBodyPrestepData prestepBundle, int innerIndex, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance, Buffer<LineInstance>> lines)
        {
            var poseA = bodies.Sets[setIndex].Poses[bodyIndices[0]];
            ContactLines.Add(poseA, ref prestepBundle.Contact0.Offset, ref prestepBundle.Contact0.Normal, ref prestepBundle.Contact0.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact1.Offset, ref prestepBundle.Contact1.Normal, ref prestepBundle.Contact1.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact2.Offset, ref prestepBundle.Contact2.Normal, ref prestepBundle.Contact2.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact3.Offset, ref prestepBundle.Contact3.Normal, ref prestepBundle.Contact3.Depth, innerIndex, tint, ref lines);
        }
    }
    struct Contact5NonconvexOneBodyLineExtractor : IConstraintLineExtractor<Contact5NonconvexOneBodyPrestepData>
    {
        public int LinesPerConstraint => 10;

        public unsafe void ExtractLines(ref Contact5NonconvexOneBodyPrestepData prestepBundle, int innerIndex, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance, Buffer<LineInstance>> lines)
        {
            var poseA = bodies.Sets[setIndex].Poses[bodyIndices[0]];
            ContactLines.Add(poseA, ref prestepBundle.Contact0.Offset, ref prestepBundle.Contact0.Normal, ref prestepBundle.Contact0.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact1.Offset, ref prestepBundle.Contact1.Normal, ref prestepBundle.Contact1.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact2.Offset, ref prestepBundle.Contact2.Normal, ref prestepBundle.Contact2.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact3.Offset, ref prestepBundle.Contact3.Normal, ref prestepBundle.Contact3.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact4.Offset, ref prestepBundle.Contact4.Normal, ref prestepBundle.Contact4.Depth, innerIndex, tint, ref lines);
        }
    }
    struct Contact6NonconvexOneBodyLineExtractor : IConstraintLineExtractor<Contact6NonconvexOneBodyPrestepData>
    {
        public int LinesPerConstraint => 12;

        public unsafe void ExtractLines(ref Contact6NonconvexOneBodyPrestepData prestepBundle, int innerIndex, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance, Buffer<LineInstance>> lines)
        {
            var poseA = bodies.Sets[setIndex].Poses[bodyIndices[0]];
            ContactLines.Add(poseA, ref prestepBundle.Contact0.Offset, ref prestepBundle.Contact0.Normal, ref prestepBundle.Contact0.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact1.Offset, ref prestepBundle.Contact1.Normal, ref prestepBundle.Contact1.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact2.Offset, ref prestepBundle.Contact2.Normal, ref prestepBundle.Contact2.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact3.Offset, ref prestepBundle.Contact3.Normal, ref prestepBundle.Contact3.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact4.Offset, ref prestepBundle.Contact4.Normal, ref prestepBundle.Contact4.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact5.Offset, ref prestepBundle.Contact5.Normal, ref prestepBundle.Contact5.Depth, innerIndex, tint, ref lines);
        }
    }
    struct Contact7NonconvexOneBodyLineExtractor : IConstraintLineExtractor<Contact7NonconvexOneBodyPrestepData>
    {
        public int LinesPerConstraint => 14;

        public unsafe void ExtractLines(ref Contact7NonconvexOneBodyPrestepData prestepBundle, int innerIndex, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance, Buffer<LineInstance>> lines)
        {
            var poseA = bodies.Sets[setIndex].Poses[bodyIndices[0]];
            ContactLines.Add(poseA, ref prestepBundle.Contact0.Offset, ref prestepBundle.Contact0.Normal, ref prestepBundle.Contact0.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact1.Offset, ref prestepBundle.Contact1.Normal, ref prestepBundle.Contact1.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact2.Offset, ref prestepBundle.Contact2.Normal, ref prestepBundle.Contact2.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact3.Offset, ref prestepBundle.Contact3.Normal, ref prestepBundle.Contact3.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact4.Offset, ref prestepBundle.Contact4.Normal, ref prestepBundle.Contact4.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact5.Offset, ref prestepBundle.Contact5.Normal, ref prestepBundle.Contact5.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact6.Offset, ref prestepBundle.Contact6.Normal, ref prestepBundle.Contact6.Depth, innerIndex, tint, ref lines);
        }
    }
    struct Contact8NonconvexOneBodyLineExtractor : IConstraintLineExtractor<Contact8NonconvexOneBodyPrestepData>
    {
        public int LinesPerConstraint => 16;

        public unsafe void ExtractLines(ref Contact8NonconvexOneBodyPrestepData prestepBundle, int innerIndex, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance, Buffer<LineInstance>> lines)
        {
            var poseA = bodies.Sets[setIndex].Poses[bodyIndices[0]];
            ContactLines.Add(poseA, ref prestepBundle.Contact0.Offset, ref prestepBundle.Contact0.Normal, ref prestepBundle.Contact0.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact1.Offset, ref prestepBundle.Contact1.Normal, ref prestepBundle.Contact1.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact2.Offset, ref prestepBundle.Contact2.Normal, ref prestepBundle.Contact2.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact3.Offset, ref prestepBundle.Contact3.Normal, ref prestepBundle.Contact3.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact4.Offset, ref prestepBundle.Contact4.Normal, ref prestepBundle.Contact4.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact5.Offset, ref prestepBundle.Contact5.Normal, ref prestepBundle.Contact5.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact6.Offset, ref prestepBundle.Contact6.Normal, ref prestepBundle.Contact6.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact7.Offset, ref prestepBundle.Contact7.Normal, ref prestepBundle.Contact7.Depth, innerIndex, tint, ref lines);
        }
    }

    struct Contact2NonconvexLineExtractor : IConstraintLineExtractor<Contact2NonconvexPrestepData>
    {
        public int LinesPerConstraint => 4;

        public unsafe void ExtractLines(ref Contact2NonconvexPrestepData prestepBundle, int innerIndex, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance, Buffer<LineInstance>> lines)
        {
            var poseA = bodies.Sets[setIndex].Poses[bodyIndices[0]];
            ContactLines.Add(poseA, ref prestepBundle.Contact0.Offset, ref prestepBundle.Contact0.Normal, ref prestepBundle.Contact0.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact1.Offset, ref prestepBundle.Contact1.Normal, ref prestepBundle.Contact1.Depth, innerIndex, tint, ref lines);
        }
    }
    struct Contact3NonconvexLineExtractor : IConstraintLineExtractor<Contact3NonconvexPrestepData>
    {
        public int LinesPerConstraint => 6;

        public unsafe void ExtractLines(ref Contact3NonconvexPrestepData prestepBundle, int innerIndex, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance, Buffer<LineInstance>> lines)
        {
            var poseA = bodies.Sets[setIndex].Poses[bodyIndices[0]];
            ContactLines.Add(poseA, ref prestepBundle.Contact0.Offset, ref prestepBundle.Contact0.Normal, ref prestepBundle.Contact0.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact1.Offset, ref prestepBundle.Contact1.Normal, ref prestepBundle.Contact1.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact2.Offset, ref prestepBundle.Contact2.Normal, ref prestepBundle.Contact2.Depth, innerIndex, tint, ref lines);
        }
    }
    struct Contact4NonconvexLineExtractor : IConstraintLineExtractor<Contact4NonconvexPrestepData>
    {
        public int LinesPerConstraint => 8;

        public unsafe void ExtractLines(ref Contact4NonconvexPrestepData prestepBundle, int innerIndex, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance, Buffer<LineInstance>> lines)
        {
            var poseA = bodies.Sets[setIndex].Poses[bodyIndices[0]];
            ContactLines.Add(poseA, ref prestepBundle.Contact0.Offset, ref prestepBundle.Contact0.Normal, ref prestepBundle.Contact0.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact1.Offset, ref prestepBundle.Contact1.Normal, ref prestepBundle.Contact1.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact2.Offset, ref prestepBundle.Contact2.Normal, ref prestepBundle.Contact2.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact3.Offset, ref prestepBundle.Contact3.Normal, ref prestepBundle.Contact3.Depth, innerIndex, tint, ref lines);
        }
    }
    struct Contact5NonconvexLineExtractor : IConstraintLineExtractor<Contact5NonconvexPrestepData>
    {
        public int LinesPerConstraint => 10;

        public unsafe void ExtractLines(ref Contact5NonconvexPrestepData prestepBundle, int innerIndex, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance, Buffer<LineInstance>> lines)
        {
            var poseA = bodies.Sets[setIndex].Poses[bodyIndices[0]];
            ContactLines.Add(poseA, ref prestepBundle.Contact0.Offset, ref prestepBundle.Contact0.Normal, ref prestepBundle.Contact0.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact1.Offset, ref prestepBundle.Contact1.Normal, ref prestepBundle.Contact1.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact2.Offset, ref prestepBundle.Contact2.Normal, ref prestepBundle.Contact2.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact3.Offset, ref prestepBundle.Contact3.Normal, ref prestepBundle.Contact3.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact4.Offset, ref prestepBundle.Contact4.Normal, ref prestepBundle.Contact4.Depth, innerIndex, tint, ref lines);
        }
    }
    struct Contact6NonconvexLineExtractor : IConstraintLineExtractor<Contact6NonconvexPrestepData>
    {
        public int LinesPerConstraint => 12;

        public unsafe void ExtractLines(ref Contact6NonconvexPrestepData prestepBundle, int innerIndex, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance, Buffer<LineInstance>> lines)
        {
            var poseA = bodies.Sets[setIndex].Poses[bodyIndices[0]];
            ContactLines.Add(poseA, ref prestepBundle.Contact0.Offset, ref prestepBundle.Contact0.Normal, ref prestepBundle.Contact0.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact1.Offset, ref prestepBundle.Contact1.Normal, ref prestepBundle.Contact1.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact2.Offset, ref prestepBundle.Contact2.Normal, ref prestepBundle.Contact2.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact3.Offset, ref prestepBundle.Contact3.Normal, ref prestepBundle.Contact3.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact4.Offset, ref prestepBundle.Contact4.Normal, ref prestepBundle.Contact4.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact5.Offset, ref prestepBundle.Contact5.Normal, ref prestepBundle.Contact5.Depth, innerIndex, tint, ref lines);
        }
    }
    struct Contact7NonconvexLineExtractor : IConstraintLineExtractor<Contact7NonconvexPrestepData>
    {
        public int LinesPerConstraint => 14;

        public unsafe void ExtractLines(ref Contact7NonconvexPrestepData prestepBundle, int innerIndex, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance, Buffer<LineInstance>> lines)
        {
            var poseA = bodies.Sets[setIndex].Poses[bodyIndices[0]];
            ContactLines.Add(poseA, ref prestepBundle.Contact0.Offset, ref prestepBundle.Contact0.Normal, ref prestepBundle.Contact0.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact1.Offset, ref prestepBundle.Contact1.Normal, ref prestepBundle.Contact1.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact2.Offset, ref prestepBundle.Contact2.Normal, ref prestepBundle.Contact2.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact3.Offset, ref prestepBundle.Contact3.Normal, ref prestepBundle.Contact3.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact4.Offset, ref prestepBundle.Contact4.Normal, ref prestepBundle.Contact4.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact5.Offset, ref prestepBundle.Contact5.Normal, ref prestepBundle.Contact5.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact6.Offset, ref prestepBundle.Contact6.Normal, ref prestepBundle.Contact6.Depth, innerIndex, tint, ref lines);
        }
    }
    struct Contact8NonconvexLineExtractor : IConstraintLineExtractor<Contact8NonconvexPrestepData>
    {
        public int LinesPerConstraint => 16;

        public unsafe void ExtractLines(ref Contact8NonconvexPrestepData prestepBundle, int innerIndex, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance, Buffer<LineInstance>> lines)
        {
            var poseA = bodies.Sets[setIndex].Poses[bodyIndices[0]];
            ContactLines.Add(poseA, ref prestepBundle.Contact0.Offset, ref prestepBundle.Contact0.Normal, ref prestepBundle.Contact0.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact1.Offset, ref prestepBundle.Contact1.Normal, ref prestepBundle.Contact1.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact2.Offset, ref prestepBundle.Contact2.Normal, ref prestepBundle.Contact2.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact3.Offset, ref prestepBundle.Contact3.Normal, ref prestepBundle.Contact3.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact4.Offset, ref prestepBundle.Contact4.Normal, ref prestepBundle.Contact4.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact5.Offset, ref prestepBundle.Contact5.Normal, ref prestepBundle.Contact5.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact6.Offset, ref prestepBundle.Contact6.Normal, ref prestepBundle.Contact6.Depth, innerIndex, tint, ref lines);
            ContactLines.Add(poseA, ref prestepBundle.Contact7.Offset, ref prestepBundle.Contact7.Normal, ref prestepBundle.Contact7.Depth, innerIndex, tint, ref lines);
        }
    }
}
