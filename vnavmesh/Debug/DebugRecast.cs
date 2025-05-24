﻿using DotRecast.Core.Numerics;
using System;
using System.Numerics;
using Navmesh.Utilities;

namespace Navmesh.Debug;

public abstract class DebugRecast : IDisposable
{
    public abstract void Dispose();

    public static void DrawBaseInfo(UITree _tree, int gridW, int gridH, RcVec3f bbMin, RcVec3f bbMax, float cellSize, float cellHeight)
    {
        _tree.LeafNode($"单元格数量: {gridW}x{gridH}");
        DrawBaseInfo(_tree, bbMin, bbMax, cellSize, cellHeight);
    }

    public static void DrawBaseInfo(UITree _tree, RcVec3f bbMin, RcVec3f bbMax, float cellSize, float cellHeight)
    {
        var playerPos = Service.ClientState.LocalPlayer?.Position ?? default;
        _tree.LeafNode($"边界: [{bbMin}] - [{bbMax}]");
        _tree.LeafNode($"单元格大小: {cellSize}x{cellHeight}");
        _tree.LeafNode($"玩家单元格: {(int)((playerPos.X - bbMin.X) / cellSize)}x{(int)((playerPos.Y - bbMin.Y) / cellHeight)}x{(int)((playerPos.Z - bbMin.Z) / cellSize)}");
    }

    public static Vector4 IntColor(int v, float a)
    {
        var mask = new BitMask((ulong)v);
        float r = (mask[1] ? 0.25f : 0) + (mask[3] ? 0.5f : 0) + 0.25f;
        float g = (mask[2] ? 0.25f : 0) + (mask[4] ? 0.5f : 0) + 0.25f;
        float b = (mask[0] ? 0.25f : 0) + (mask[5] ? 0.5f : 0) + 0.25f;
        return new(r, g, b, a);
    }
}
