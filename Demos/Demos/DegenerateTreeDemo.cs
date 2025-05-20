using System;
using System.IO;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Constraints;
using BepuPhysics.Trees;
using BepuUtilities;
using BepuUtilities.Memory;
using DemoContentLoader;
using DemoRenderer;

namespace Demos.Demos;

public class DegenerateTreeDemo : Demo
{
    public override void Initialize(ContentArchive content, Camera camera)
    {
        Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)),
            new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));

        var tree = LoadTree("C:\\Temp\\tree.bin");
        for (var i = 0; i < 3000; i++) Insert(ref tree);
        for (var i = 0; i < 100; i++) tree.RefitAndRefine(BufferPool, i, 10000000);

        Console.WriteLine(AlexDebug.Print(tree));

        var tree2 = RebuildTree(tree);
        Console.WriteLine();
        Console.WriteLine("-----------------------------------------");
        Console.WriteLine();
        Console.WriteLine(AlexDebug.Print(tree2));
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
        tree.Add(new BoundingBox(new Vector3(-0.49999994f), new Vector3(0.49999994f)), BufferPool);
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
}