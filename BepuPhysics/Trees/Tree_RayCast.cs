using BepuUtilities.Memory;
using System.Diagnostics;
using System.Numerics;

namespace BepuPhysics.Trees
{
    partial struct Tree
    {
        //Working around https://github.com/dotnet/runtime/issues/95043:
        //Under x86 with optimizations, forcing inlining seems to cause problems for sweeps. *Not* forcing it also harms performance.
        //Under x64, though, there's not really any cost to letting the JIT decide. TODO: Probably should look into ARM eventually.
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static bool Intersects(Vector3 min, Vector3 max, TreeRay* ray, out float t)
        {
            var t0 = min * ray->InverseDirection - ray->OriginOverDirection;
            var t1 = max * ray->InverseDirection - ray->OriginOverDirection;
            var tExit = Vector3.Max(t0, t1);
            var tEntry = Vector3.Min(t0, t1);
            //TODO: Note the use of broadcast and SIMD min/max here. This is much faster than using branches to compute minimum elements, since the branches
            //get mispredicted extremely frequently. Also note 4-wide operations; they're actually faster than using Vector2 or Vector3 due to some unnecessary codegen as of this writing.
            var earliestExit = Vector4.Min(Vector4.Min(new Vector4(ray->MaximumT), new Vector4(tExit.X)), Vector4.Min(new Vector4(tExit.Y), new Vector4(tExit.Z))).X;
            t = Vector4.Max(Vector4.Max(new Vector4(tEntry.X), Vector4.Zero), Vector4.Max(new Vector4(tEntry.Y), new Vector4(tEntry.Z))).X;
            return t <= earliestExit;
        }


        internal readonly unsafe void RayCast<TLeafTester>(int nodeIndex, TreeRay* treeRay, RayData* rayData, Buffer<int> stack, BufferPool pool, ref TLeafTester leafTester) where TLeafTester : IRayLeafTester
        {
            Debug.Assert((nodeIndex >= 0 && nodeIndex < NodeCount) || (Encode(nodeIndex) >= 0 && Encode(nodeIndex) < LeafCount));
            Debug.Assert(LeafCount >= 2, "This implementation assumes all nodes are filled.");

            int stackEnd = 0;
            while (true)
            {
                if (nodeIndex < 0)
                {
                    //This is actually a leaf node.
                    var leafIndex = Encode(nodeIndex);
                    leafTester.TestLeaf(leafIndex, rayData, &treeRay->MaximumT, pool);
                    //Leaves have no children; have to pull from the stack to get a new target.
                    if (stackEnd == 0)
                        break;
                    nodeIndex = stack[--stackEnd];
                }
                else
                {
                    ref var node = ref Nodes[nodeIndex];
                    var aIntersected = Intersects(node.A.Min, node.A.Max, treeRay, out var tA);
                    var bIntersected = Intersects(node.B.Min, node.B.Max, treeRay, out var tB);

                    if (aIntersected)
                    {
                        if (bIntersected)
                        {
                            //Visit the earlier AABB intersection first.
                            if (stackEnd == stack.Length)
                            {
                                if (stack.Length == TraversalStackCapacity)
                                {
                                    // First allocation is on the stack.
                                    pool.TakeAtLeast<int>(TraversalStackCapacity * 2, out var newStack);
                                    stack.CopyTo(0, newStack, 0, TraversalStackCapacity);
                                    stack = newStack;
                                }
                                else
                                    pool.Resize(ref stack, stackEnd * 2, stackEnd);
                            }
                            if (tA < tB)
                            {
                                nodeIndex = node.A.Index;
                                stack[stackEnd++] = node.B.Index;
                            }
                            else
                            {
                                nodeIndex = node.B.Index;
                                stack[stackEnd++] = node.A.Index;
                            }
                        }
                        else
                        {
                            //Single intersection cases don't require an explicit stack entry.
                            nodeIndex = node.A.Index;
                        }
                    }
                    else if (bIntersected)
                    {
                        nodeIndex = node.B.Index;
                    }
                    else
                    {
                        //No intersection. Need to pull from the stack to get a new target.
                        if (stackEnd == 0)
                            break;
                        nodeIndex = stack[--stackEnd];
                    }
                }
            }
            if (stack.Length > TraversalStackCapacity)
            {
                // We rented a larger stack at some point. Return it.
                pool.Return(ref stack);
            }
        }

        internal const int TraversalStackCapacity = 256;

        internal readonly unsafe void RayCast<TLeafTester>(TreeRay* treeRay, RayData* rayData, BufferPool pool, ref TLeafTester leafTester) where TLeafTester : IRayLeafTester
        {
            if (LeafCount == 0)
                return;

            if (LeafCount == 1)
            {
                //If the first node isn't filled, we have to use a special case.
                if (Intersects(Nodes[0].A.Min, Nodes[0].A.Max, treeRay, out var tA))
                {
                    leafTester.TestLeaf(0, rayData, &treeRay->MaximumT, pool);
                }
            }
            else
            {
                var stack = stackalloc int[TraversalStackCapacity];
                RayCast(0, treeRay, rayData, new Buffer<int>(stack, TraversalStackCapacity), pool, ref leafTester);
            }
        }

        /// <summary>
        /// Tests a ray against the tree and invokes the <paramref name="leafTester"/> for each leaf node that the ray intersects.
        /// </summary>
        /// <typeparam name="TLeafTester">The type of the <see cref="IRayLeafTester"/> used to process the intersecting leaves.</typeparam>
        /// <param name="origin">The origin point of the ray.</param>
        /// <param name="direction">The direction of the ray.</param>
        /// <param name="maximumT">The maximum parametric distance along the ray to test. This value may be modified by the leaf tester during traversal.</param>
        /// <param name="pool">The buffer pool used for temporary allocations during the operation. Only used if the tree is pathologically deep; stack memory is used preferentially.</param>
        /// <param name="leafTester">A reference to the tester that processes the indices of intersecting leaves.</param>
        /// <param name="id">An optional identifier for the ray that can be used by the leaf tester.</param>
        public readonly unsafe void RayCast<TLeafTester>(Vector3 origin, Vector3 direction, ref float maximumT, BufferPool pool, ref TLeafTester leafTester, int id = 0) where TLeafTester : IRayLeafTester
        {
            TreeRay.CreateFrom(origin, direction, maximumT, id, out var rayData, out var treeRay);
            RayCast(&treeRay, &rayData, pool, ref leafTester);
            //The maximumT could have been mutated by the leaf tester. Propagate that change. This is important for when we jump between tree traversals and such.
            maximumT = treeRay.MaximumT;
        }

    }
}
