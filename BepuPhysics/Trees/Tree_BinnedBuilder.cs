using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;

namespace BepuPhysics.Trees
{
    partial struct Tree
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static unsafe Int4 Truncate(Vector4 v)
        {
            Int4 discrete;
            if (Vector128.IsHardwareAccelerated)
            {
                Vector128.Store(Vector128.ConvertToInt32(v.AsVector128()), (int*)&discrete);
            }
            else
            {
                discrete.X = (int)v.X;
                discrete.Y = (int)v.Y;
                discrete.Z = (int)v.Z;
                discrete.W = (int)v.W;
            }
            return discrete;
        }
        static unsafe void BinnedBuilderInternal(Buffer<int> indices, Buffer<BoundingBox4> boundingBoxes, Buffer<BoundingBox4> binBoundingBoxes, Buffer<Node> nodes)
        {
            var centroidMin = new Vector4(float.MaxValue);
            var centroidMax = new Vector4(float.MinValue);
            var leafCount = indices.Length;
            for (int i = 0; i < leafCount; ++i)
            {
                ref var box = ref boundingBoxes[i];
                //Note that centroids never bother scaling by 0.5. It's fine as long as we're consistent.
                var centroid = box.Min + box.Max;
                centroidMin = Vector4.Min(centroidMin, centroid);
                centroidMax = Vector4.Max(centroidMax, centroid);
            }
            var binCount = Math.Min(MaximumBinCount, Math.Max((int)(leafCount * 0.25f), 4));
            Debug.Assert(binBoundingBoxes.Length >= binCount);
            var centroidSpan = centroidMax - centroidMin;
            var offsetToBinIndex = centroidSpan / binCount;

            var zeroedUpperSpan = centroidSpan.AsVector128().WithElement(3, 0);
            if (Vector128.Dot(zeroedUpperSpan, zeroedUpperSpan) <= 1e-12f)
            {
                //This node is completely degenerate; there is no 'good' ordering of the children. Pick a split in the middle and shrug.
                //This shouldn't happen unless something is badly wrong with the input; no point in optimizing it.
                var midpoint = indices.Length / 2;
                var secondCount = indices.Length - midpoint;
                //Still have to compute the child bounding boxes, because the centroid bounds span being zero doesn't imply that the full bounds are zero.
                for (int i = 0; i < midpoint; ++i)
                {

                }
                //nodes[0] =
                BinnedBuilderInternal(indices.Slice(midpoint), boundingBoxes.Slice(midpoint), binBoundingBoxes, nodes.Slice(midpoint - 1));
                BinnedBuilderInternal(indices.Slice(midpoint, secondCount), boundingBoxes.Slice(midpoint, secondCount), binBoundingBoxes, nodes.Slice(secondCount - 1));
            }

            for (int i = 0; i < binCount; ++i)
            {
                ref var box = ref binBoundingBoxes[i];
                box.Min = new Vector4(float.MaxValue);
                box.Max = new Vector4(float.MinValue);
            }

            var maximumBinIndex = new Vector4(binCount - 1);
            for (int i = 0; i < leafCount; ++i)
            {
                ref var box = ref boundingBoxes[i];
                var centroid = box.Min + box.Max;
                var binIndicesForLeafContinuous = Vector4.Min(maximumBinIndex, (centroid - centroidMin) * offsetToBinIndex);
                //Note that we don't store out any of the indices into per-bin lists here. We only *really* want two final groups for the children,
                //and we can easily compute those by performing another scan. It requires recomputing the bin indices, but that's really not much of a concern.
                var binIndicesForLeaf = Truncate(binIndicesForLeafContinuous);
                ref var xBounds = ref binBoundingBoxes[binIndicesForLeaf.X];
                ref var yBounds = ref binBoundingBoxes[binIndicesForLeaf.Y];
                ref var zBounds = ref binBoundingBoxes[binIndicesForLeaf.Z];
                xBounds.Min = Vector4.Min(xBounds.Min, box.Min);
                xBounds.Max = Vector4.Max(xBounds.Max, box.Max);
                yBounds.Min = Vector4.Min(yBounds.Min, box.Min);
                yBounds.Max = Vector4.Max(yBounds.Max, box.Max);
                zBounds.Min = Vector4.Min(zBounds.Min, box.Min);
                zBounds.Max = Vector4.Max(zBounds.Max, box.Max);
            }

            //Identify the split index by examining the SAH of very split option.
            var leftBounds = binBoundingBoxes[0];
            Debug.Assert(leftBounds.Min.X > float.MinValue && leftBounds.Min.Y > float.MinValue && leftBounds.Min.Z > float.MinValue, "Bin 0 should have been updated ");
            for (int i = 1; i < binCount; ++i)
            {

            }
        }

        public static unsafe void BinnedBuilder(Buffer<int> indices, Buffer<BoundingBox> boundingBoxes, Buffer<Node> nodes, BufferPool pool)
        {
            var leafCount = indices.Length;
            Debug.Assert(boundingBoxes.Length >= leafCount, "The bounding boxes provided must cover the range of indices provided.");
            Debug.Assert(nodes.Length > leafCount - 1, "The output nodes must be able to contain the nodes created for the leaves.");
            if (leafCount == 0)
                return;
            if (leafCount == 1)
            {
                //If there's only one leaf, the tree has a special format: the root node has only one child.
                ref var root = ref nodes[0];
                root.A.Min = boundingBoxes[0].Min;
                root.A.Index = Encode(indices[0]);
                root.A.Max = boundingBoxes[0].Max;
                root.A.LeafCount = 1;
                root.B = default;
                return;
            }
            boundingBoxes = boundingBoxes.Slice(indices.Length);
            nodes = nodes.Slice(leafCount - 1);

            var binBoundsMemory = stackalloc BoundingBox4[MaximumBinCount];
            var binBounds = new Buffer<BoundingBox4>(binBoundsMemory, MaximumBinCount);

            //While we could avoid a recursive implementation, the overhead is low compared to the per-iteration cost.
            BinnedBuilderInternal(indices, boundingBoxes.As<BoundingBox4>(), binBounds, nodes);
        }

    }
}
