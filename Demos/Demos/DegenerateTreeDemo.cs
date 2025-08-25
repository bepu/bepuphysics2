using System;
using System.Diagnostics;
using System.IO;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuPhysics.Constraints.Contact;
using BepuPhysics.Trees;
using BepuUtilities;
using BepuUtilities.Memory;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.Constraints;
using DemoRenderer.UI;
using DemoUtilities;

namespace Demos.Demos;

public class DegenerateTreeDemo : Demo
{
    Tree tree, tree2;
    public override void Initialize(ContentArchive content, Camera camera)
    {
        Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)),
            new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));

        {
            const int width = 1 << 23;
            const int height = 1;
            const int length = 1;
            var treePerfTest = new Tree(BufferPool, initialLeafCapacity: width * height * length);
            var spacing = new Vector3(16.01f);
            float randomization = 0.9f;
            var randomizationSpan = (spacing - new Vector3(1)) * randomization;
            var randomizationBase = randomizationSpan * -0.5f;
            var random = new Random(5);
            var start = Stopwatch.GetTimestamp();
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    for (int k = 0; k < length; ++k)
                    {
                        var r = new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle());
                        //var location = spacing * (new Vector3(i, j, k) + new Vector3(-width, 1, -length)) + randomizationBase + r * randomizationSpan;
                        //var location = (r - new Vector3(0.5f)) * (r - new Vector3(0.5f)) * spacing * new Vector3(width, height, length);
                        //var location = (r - new Vector3(0.5f)) * spacing * new Vector3(width, height, length);
                        //var location = new Vector3(15, 15, 15);
                        var location = spacing * (new Vector3(i, j, k) + new Vector3(-width / 2f, 1, -length / 2f));

                        treePerfTest.Add(new BoundingBox(location - new Vector3(0.5f), location + new Vector3(0.5f)), BufferPool);
                    }
                }
            }
            var end = Stopwatch.GetTimestamp();
            var elapsedTime = (end - start) / (double)Stopwatch.Frequency;
            Console.WriteLine($"Inserted {treePerfTest.LeafCount} leaves in {elapsedTime:F2} seconds ({treePerfTest.LeafCount / elapsedTime:F2} inserts/second, {1e6 * elapsedTime / treePerfTest.LeafCount} us/insert)");
        }
        tree = LoadTree("C:\\Users\\norbe\\source\\repos\\bepuphysics2repro\\Demos\\Content\\tree.bin");
        Console.WriteLine($"Pre-insertion depth: {tree.ComputeMaximumDepth()}");
        for (var i = 0; i < 100; i++)
        {
            if (i == 99)
                Console.WriteLine("Inserting final item");
            Insert(ref tree);
        }
        Console.WriteLine($"Post-insertion depth: {tree.ComputeMaximumDepth()}");
        //for (var i = 0; i < 100; i++) tree.RefitAndRefine(BufferPool, i, 10000000);
        //int refineIndex = 0;
        //for (var i = 0; i < 100; i++) tree.Refine2(256, ref refineIndex, 4, 128, BufferPool);

        //Console.WriteLine(AlexDebug.Print(tree));

        tree2 = RebuildTree(tree);

    }

    private unsafe Tree RebuildTree(Tree tree)
    {
        var tree2 = new Tree(BufferPool, tree.LeafCount)
        {
            NodeCount = tree.LeafCount - 1,
            LeafCount = tree.LeafCount
        };
        BufferPool.Take(tree.LeafCount, out Buffer<NodeChild> subTree);
        FillSubtree(subTree, tree);

        tree2.BinnedBuild(subTree, BufferPool);
        tree2.RefitAndRefine(BufferPool, 0, 1000f);
        BufferPool.Return(ref subTree);
        return tree2;
    }

    private void FillSubtree(Buffer<NodeChild> subTree, Tree tree)
    {
        for (var i = 0; i < tree.LeafCount; i++)
        {
            var leaf = tree.Leaves[i];
            var node = tree.Nodes[leaf.NodeIndex];
            var nodeChild = leaf.ChildIndex == 0 ? node.A : node.B;

            subTree[i] = nodeChild with { LeafCount = 1, Index = Tree.Encode(i) };
        }
    }

    private void Insert(ref Tree tree)
    {
        //tree.Add(new BoundingBox(new Vector3(-0.49999994f), new Vector3(0.49999994f)), BufferPool);
        tree.Add(new BoundingBox(new Vector3(-0.3f, -0.3f, -0.020537015f), new Vector3(0.3f, 0.3f, 0.020537015f)),
            BufferPool);
    }

    private Tree LoadTree(string path)
    {
        using var binReader = new BinaryReader(File.OpenRead(path));
        var byteCount = binReader.ReadInt32();
        var data = binReader.ReadBytes(byteCount);
        return new Tree(data, BufferPool);
    }


    // Claude did an okay job of this visualization, I'd say.
    public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
    {
        base.Render(renderer, camera, input, text, font);

        VisualizeTreeTopology(renderer, camera, text, font, new Vector3(0, 0, 0), ref tree);
        VisualizeTreeTopology(renderer, camera, text, font, new Vector3(100, 0, 0), ref tree2);

        // Also, just draw the tree's leaf node bounding boxes somewhere.
        VisualizeLeafNodeBounds(renderer, camera, new Vector3(0, 0, -100), ref tree);
        VisualizeLeafNodeBounds(renderer, camera, new Vector3(100, 0, -100), ref tree2);

    }

    void VisualizeLeafNodeBounds(Renderer renderer, Camera camera, Vector3 position, ref Tree tree)
    {
        var color = new Vector3(0, 0.8f, 0.8f);
        for (int i = 0; i < tree.LeafCount; ++i)
        {
            var leaf = tree.Leaves[i];
            var node = tree.Nodes[leaf.NodeIndex];
            var bounds = leaf.ChildIndex == 0 ? node.A : node.B;
            var span = bounds.Max - bounds.Min;
            renderer.Shapes.AddShape(new Box(span.X, span.Y, span.Z), null, position + 0.5f * (bounds.Min + bounds.Max), color);
        }
    }

    void VisualizeTreeTopology(Renderer renderer, Camera camera, TextBuilder text, Font font, Vector3 position, ref Tree tree)
    {
        if (tree.LeafCount > 0)
        {
            const float levelHeight = 3f;
            const float nodeSpacing = 20f;

            // Start visualization from the root (node 0) at origin
            RenderNodeRecursive(renderer, camera, text, font, ref tree, 0, 0, 0f, levelHeight, nodeSpacing, position);
        }
    }

    void RenderNodeRecursive(Renderer renderer, Camera camera, TextBuilder text, Font font, ref Tree tree, int nodeIndex, int depth, float horizontalOffset, float levelHeight, float nodeSpacing, Vector3 position)
    {
        var nodePosition = position + new Vector3(horizontalOffset, depth * levelHeight, 0);

        var nodeColor = new Vector3(0.8f, 0.2f, 0.2f);
        renderer.Shapes.AddShape(new Sphere(0.3f), null, nodePosition, nodeColor);

        if (DemoRenderer.Helpers.GetScreenLocation(nodePosition, camera.ViewProjection, renderer.Surface.Resolution, out var location))
        {
            renderer.TextBatcher.Write(text.Clear().Append(nodeIndex), location, 10, new Vector3(1), font);
        }

        ref var node = ref tree.Nodes[nodeIndex];

        // Calculate child spacing (gets tighter with depth)
        float childSpacing = nodeSpacing / float.Pow(1.7f, depth);
        float leftOffset = horizontalOffset - childSpacing;
        float rightOffset = horizontalOffset + childSpacing;

        // Process child A (left)
        var childAPosition = position + new Vector3(leftOffset, (depth + 1) * levelHeight, 0);
        if (node.A.Index >= 0)
        {
            // Internal node - recurse
            renderer.Lines.Allocate() = new LineInstance(nodePosition, childAPosition, new Vector3(0.8f, 0.8f, 0.8f), default);
            RenderNodeRecursive(renderer, camera, text, font, ref tree, node.A.Index, depth + 1, leftOffset, levelHeight, nodeSpacing, position);
        }
        else
        {
            // Leaf node
            renderer.Lines.Allocate() = new LineInstance(nodePosition, childAPosition, new Vector3(0.2f, 0.8f, 0.2f), default);
            renderer.Shapes.AddShape(new Sphere(0.15f), null, childAPosition, new Vector3(0.2f, 1f, 0.2f));
            if (DemoRenderer.Helpers.GetScreenLocation(childAPosition, camera.ViewProjection, renderer.Surface.Resolution, out var childLocation))
                renderer.TextBatcher.Write(text.Clear().Append(Tree.Encode(node.A.Index)), childLocation, 10, new Vector3(1), font);
        }

        // Process child B (right)
        var childBPosition = position + new Vector3(rightOffset, (depth + 1) * levelHeight, 0);
        if (node.B.Index >= 0)
        {
            // Internal node - recurse
            renderer.Lines.Allocate() = new LineInstance(nodePosition, childBPosition, new Vector3(0.8f, 0.8f, 0.8f), default);
            RenderNodeRecursive(renderer, camera, text, font, ref tree, node.B.Index, depth + 1, rightOffset, levelHeight, nodeSpacing, position);
        }
        else
        {
            // Leaf node
            renderer.Lines.Allocate() = new LineInstance(nodePosition, childBPosition, new Vector3(0.2f, 0.8f, 0.2f), default);
            renderer.Shapes.AddShape(new Sphere(0.15f), null, childBPosition, new Vector3(0.2f, 1f, 0.2f));
            if (DemoRenderer.Helpers.GetScreenLocation(childBPosition, camera.ViewProjection, renderer.Surface.Resolution, out var childLocation))
                renderer.TextBatcher.Write(text.Clear().Append(Tree.Encode(node.B.Index)), childLocation, 10, new Vector3(1), font);
        }
    }
}