using System;
using System.Collections.Generic;
using System.IO;
using BepuPhysics.Trees;

namespace BepuPhysics;

public static class AlexDebug
{
    public static string Print(Tree tree)
    {
        if (tree.NodeCount == 0)
        {
            return "";
        }

        using var sw = new StringWriter();
        PrintTree(0, n => n < 0 ? Array.Empty<int>() : [tree.Nodes[n].A.Index, tree.Nodes[n].B.Index], i => i < 0 ? $"Leaf({Tree.Encode(i)})" : $"Node({i})", sw);
        var res = sw.ToString();
        return res;
    }


    public const string Node = "● ";
    public const string NodePad = "  ";
    public const string Line = "│ ";
    public const string LineDotted = "╵ ";
    public const string Arm = "├─";
    public const string ArmLast = "└─";
    public const string Pad = "  ";

    public static void PrintTree<T>(T node, Func<T, IReadOnlyList<T>> children, Func<T, string> format,
                                    StringWriter output, string indentSelf = "", string indent = "")
    {
        var strs = format(node).Split('\n');

        // var multilineIndent = indent + NodePad;
        for (var i = 0; i < strs.Length; i++)
        {
            if (i == 0)
            {
                output.Write(indentSelf);
                output.Write(Node);
            }
            else
            {
                output.Write(indent);
                output.Write(LineDotted);
            }

            output.WriteLine(strs[i]);
        }

        var ch = children(node);
        if (ch == null || ch.Count == 0)
        {
            return;
        }

        for (var i = 0; i < ch.Count; i++)
        {
            if (i >= ch.Count - 1)
            {
                PrintTree(ch[i], children, format, output, indent + ArmLast, indent + Pad);
            }
            else
            {
                PrintTree(ch[i], children, format, output, indent + Arm, indent + Line);
            }
        }
    }

    // same but with a list at the first level
    public static void PrintTree<T>(IList<T> nodes, Func<T, IReadOnlyList<T>?> children, Func<T, string> format,
                                    StringWriter output, string indentSelf = "", string indent = "")
    {
        output.Write(indentSelf);
        output.WriteLine(Node);

        if (nodes.Count == 0)
        {
            return;
        }

        for (var i = 0; i < nodes.Count; i++)
        {
            if (i >= nodes.Count - 1)
            {
                PrintTree(nodes[i], children, format, output, indent + ArmLast, indent + Pad);
            }
            else
            {
                PrintTree(nodes[i], children, format, output, indent + Arm, indent + Line);
            }
        }
    }
}