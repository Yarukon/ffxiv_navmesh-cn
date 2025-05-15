using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

namespace Navmesh.NavVolume;

public class PathfindLoopException(ulong from, ulong to, Vector3 fromP, Vector3 toP) : Exception
{
    public readonly ulong   FromVoxel = from;
    public readonly ulong   ToVoxel   = to;
    public readonly Vector3 FromPos   = fromP;
    public readonly Vector3 ToPos     = toP;

    public override string Message => $"An infinite loop occurred during the pathfind operation. (from={FromVoxel:X} / {FromPos}, to={ToVoxel:X} / {ToPos})";
}

public static class VoxelSearch
{
    public static IEnumerable<(ulong index, bool empty)> EnumerateLeafVoxels(VoxelMap volume, Vector3 center, Vector3 halfExtent)
        => volume.RootTile.EnumerateLeafVoxels(center - halfExtent, center + halfExtent);

    public static Vector3 FindClosestVoxelPoint(VoxelMap volume, ulong index, Vector3 p, float eps = 0.1f)
    {
        var (min, max) = volume.VoxelBounds(index, eps);
        return Vector3.Clamp(p, min, max);
    }

    public static ulong FindNearestEmptyVoxel(VoxelMap volume, Vector3 center, Vector3 halfExtent)
    {
        var cv = volume.FindLeafVoxel(center);
        //Service.Log.Debug($"Searching {cv}");
        if (cv.empty)
            return cv.voxel; // fast path: the cell is empty already

        var minDist = float.MaxValue;
        var res     = VoxelMap.InvalidVoxel;
        foreach (var v in volume.RootTile.EnumerateLeafVoxels(center - halfExtent, center + halfExtent))
        {
            if (!v.empty)
                continue;

            var p    = FindClosestVoxelPoint(volume, v.index, center, 0);
            var d    = p - center;
            var dist = d.LengthSquared();
            if (d.X != 0 || d.Z != 0)
                dist += 100; // penalty for moving sideways vs up - TODO reconsider...
            if (d.Y < 0)
                dist += 400; // penalty for lower voxels to reduce chance of it being underground - TODO reconsider...

            if (dist < minDist)
            {
                minDist = dist;
                res     = v.index;
            }
        }

        return res;
    }

    // enumerate entered voxels along line; starting voxel is not returned, ending voxel is
    public static IEnumerable<(ulong voxel, float t, bool empty)> EnumerateVoxelsInLine(
        VoxelMap volume, ulong fromVoxel, ulong toVoxel, Vector3 fromPos, Vector3 toPos)
    {
        if (fromVoxel == toVoxel)
            yield break; // 如果起点和终点是同一个体素，则直接返回，避免不必要的计算

        var origFrom = fromVoxel;
        var ab       = toPos - fromPos;
        var abLength = ab.Length();

        // 如果两点非常接近，直接返回终点体素
        if (abLength < 1e-5f)
        {
            yield return (toVoxel, 1.0f, volume.IsEmpty(toVoxel));
            yield break;
        }

        // 增加 eps 的大小，以应对边界情况
        var eps = Math.Max(0.2f / abLength, 1e-4f);

        // 标记已经访问过的体素，防止循环
        var visitedVoxels = new HashSet<ulong> { fromVoxel };

        var       iterations    = 0;
        const int MaxIterations = 1000; // 防止无限循环的安全限制

        while (fromVoxel != toVoxel && iterations < MaxIterations)
        {
            iterations++;
            var (vmin, vmax) = volume.VoxelBounds(fromVoxel, 0);

            // 找到与三个平面的交点（六个面中的三个）
            // 直线-平面交点: Q = A + AB*t, PQ*n=0 => (PA + tAB)*n = 0 => t = AP*n / AB*n
            var tx = ab.X == 0 ? float.MaxValue : ((ab.X > 0 ? vmax.X : vmin.X) - fromPos.X) / ab.X;
            var ty = ab.Y == 0 ? float.MaxValue : ((ab.Y > 0 ? vmax.Y : vmin.Y) - fromPos.Y) / ab.Y;
            var tz = ab.Z == 0 ? float.MaxValue : ((ab.Z > 0 ? vmax.Z : vmin.Z) - fromPos.Z) / ab.Z;

            // 修正极小或负数 t 值，防止精度问题
            if (tx <= eps) tx = eps;
            if (ty <= eps) ty = eps;
            if (tz <= eps) tz = eps;

            var t = Math.Min(Math.Min(tx, ty), Math.Min(tz, 1.0f));

            // 确保 t 始终前进
            t = Math.Max(t, eps);
            t = Math.Min(t + eps, 1.0f);

            // 处理接近目标体素的情况
            var distToEnd = (toPos - (fromPos + (t * ab))).Length();
            if (distToEnd < 0.5f)
            {
                // 已经非常接近终点，直接返回终点体素
                yield return (toVoxel, 1.0f, volume.IsEmpty(toVoxel));
                break;
            }

            // 计算下一个体素的位置
            var proj = fromPos + (t * ab);

            // 为了避免精度问题，确保投影点不在当前体素的边界上
            const float safeDelta                              = 1e-3f;
            if (MathF.Abs(proj.X - vmin.X) < safeDelta) proj.X = vmin.X + safeDelta;
            if (MathF.Abs(proj.Y - vmin.Y) < safeDelta) proj.Y = vmin.Y + safeDelta;
            if (MathF.Abs(proj.Z - vmin.Z) < safeDelta) proj.Z = vmin.Z + safeDelta;
            if (MathF.Abs(proj.X - vmax.X) < safeDelta) proj.X = vmax.X - safeDelta;
            if (MathF.Abs(proj.Y - vmax.Y) < safeDelta) proj.Y = vmax.Y - safeDelta;
            if (MathF.Abs(proj.Z - vmax.Z) < safeDelta) proj.Z = vmax.Z - safeDelta;

            var (nextVoxel, nextEmpty) = volume.FindLeafVoxel(proj);

            // 检查是否仍在同一个体素内
            if (nextVoxel == fromVoxel)
            {
                // 尝试更大的步进以突破边界
                t                      = Math.Min(t + 0.01f, 1.0f);
                proj                   = fromPos + (t * ab);
                (nextVoxel, nextEmpty) = volume.FindLeafVoxel(proj);

                // 如果仍然在同一个体素内，检查是否已经接近终点
                if (nextVoxel == fromVoxel)
                {
                    // 如果已经非常接近终点，直接返回终点体素
                    if ((toPos - fromPos).LengthSquared() < 1.0f)
                    {
                        yield return (toVoxel, 1.0f, volume.IsEmpty(toVoxel));
                        break;
                    }

                    // 最后尝试直接跳到中间点
                    var midPoint = (fromPos + toPos) * 0.5f;
                    (nextVoxel, nextEmpty) = volume.FindLeafVoxel(midPoint);

                    if (nextVoxel == fromVoxel || visitedVoxels.Contains(nextVoxel))
                    {
                        // 如果仍然失败，抛出异常，但提供更多上下文信息
                        throw new PathfindLoopException(origFrom, toVoxel, fromPos, toPos);
                    }
                }
            }

            // 防止循环遍历同一组体素
            if (visitedVoxels.Contains(nextVoxel))
            {
                // 尝试更大步进，跳过当前体素
                t                      = Math.Min(t + 0.05f, 1.0f);
                proj                   = fromPos + (t * ab);
                (nextVoxel, nextEmpty) = volume.FindLeafVoxel(proj);

                // 如果仍然在已访问的体素中，尝试直接跳到终点
                if (visitedVoxels.Contains(nextVoxel))
                {
                    yield return (toVoxel, 1.0f, volume.IsEmpty(toVoxel));
                    break;
                }
            }

            // 添加到已访问列表
            visitedVoxels.Add(nextVoxel);

            yield return (nextVoxel, t, nextEmpty);
            fromVoxel = nextVoxel;
            fromPos   = proj;

            // 重新计算到终点的向量
            ab       = toPos - fromPos;
            abLength = ab.Length();
            if (abLength < 1e-5f)
            {
                // 如果已经非常接近终点，直接返回终点体素
                if (fromVoxel != toVoxel) yield return (toVoxel, 1.0f, volume.IsEmpty(toVoxel));
                break;
            }
        }

        // 如果达到最大迭代次数仍未到达目标
        if (iterations >= MaxIterations && fromVoxel != toVoxel) yield return (toVoxel, 1.0f, volume.IsEmpty(toVoxel));
    }

    public static bool LineOfSight(VoxelMap volume, ulong fromVoxel, ulong toVoxel, Vector3 fromPos, Vector3 toPos)
    {
        if (fromVoxel == toVoxel)
            return true;

        // 如果两点非常近，直接检查目标体素是否为空
        if ((fromPos - toPos).LengthSquared() < 0.1f)
            return volume.IsEmpty(toVoxel);

        try { return EnumerateVoxelsInLine(volume, fromVoxel, toVoxel, fromPos, toPos).All(v => v.empty); }
        catch (PathfindLoopException ex)
        {
            // 遇到路径循环问题，记录日志
            Service.Log.Debug($"LineOfSight 遇到路径循环问题: {ex.Message}");

            // 尝试替代策略：使用更简单的体素遍历方法
            return TryAlternativeLineOfSight(volume, fromVoxel, toVoxel, fromPos, toPos);
        }
    }

    /// <summary>
    ///     当标准 LineOfSight 方法失败时使用的替代方法
    /// </summary>
    private static bool TryAlternativeLineOfSight(VoxelMap volume, ulong fromVoxel, ulong toVoxel, Vector3 fromPos, Vector3 toPos)
    {
        // 简化版的视线检测算法，使用更保守的方法
        // 沿着直线采样多个点，检查每个点所在的体素是否为空
        const int sampleCount = 10;

        for (var i = 1; i < sampleCount; i++)
        {
            var t         = i / (float)sampleCount;
            var samplePos = Vector3.Lerp(fromPos, toPos, t);

            var voxelResult = volume.FindLeafVoxel(samplePos);
            if (!voxelResult.empty)
                return false;
        }

        // 如果所有采样点都通过，认为可以直线通过
        return true;
    }
}
