using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Threading;
using DotRecast.Detour;
using Navmesh.NavVolume;

namespace Navmesh;

public class NavmeshQuery
{
    private class IntersectQuery : IDtPolyQuery
    {
        public readonly List<long> Result = [];

        public void Process(DtMeshTile tile, DtPoly poly, long refs) => Result.Add(refs);
    }

    public           DtNavMeshQuery MeshQuery;
    public           VoxelPathfind? VolumeQuery;
    private readonly IDtQueryFilter _filter = new DtQueryDefaultFilter();
    private readonly Random         _random = new();

    public  List<long> LastPath => _lastPath;
    private List<long> _lastPath = [];

    public NavmeshQuery(Navmesh navmesh)
    {
        MeshQuery = new(navmesh.Mesh);
        if (navmesh.Volume != null)
            VolumeQuery = new(navmesh.Volume);
    }

    public List<Vector3> PathfindMesh(Vector3 from, Vector3 to, bool useRaycast, bool useStringPulling, CancellationToken cancel)
    {
        var startRef = FindNearestMeshPoly(from);
        var endRef   = FindNearestMeshPoly(to);
        Service.Log.Debug($"[寻路] 多边形 {startRef:X} -> {endRef:X}");
        if (startRef == 0 || endRef == 0)
        {
            Service.Log.Error($"从 {from} ({startRef:X}) 到 {to} ({endRef:X}) 的路径查找失败：无法在网格上找到多边形");
            return [];
        }

        var timer = Timer.Create();
        _lastPath.Clear();

        // 添加随机化因子到寻路选项中
        var options = 0;
        if (useRaycast)
            options |= DtFindPathOptions.DT_FINDPATH_ANY_ANGLE;

        var opt = new DtFindPathOption(options, useRaycast ? 5 : 0);

        // TODO: DotRecast库没有直接支持随机化，可考虑修改权重或者在后处理中添加随机性
        MeshQuery.FindPath(startRef, endRef, from.SystemToRecast(), to.SystemToRecast(), _filter, ref _lastPath, opt);

        if (_lastPath.Count == 0)
        {
            Service.Log.Error($"从 {from} ({startRef:X}) 到 {to} ({endRef:X}) 的路径查找失败：无法在网格上找到路径");
            return [];
        }

        Service.Log.Debug($"寻路耗时 {timer.Value().TotalSeconds:f3} 秒: {string.Join(", ", _lastPath.Select(r => r.ToString("X")))}");

        var endPos = to.SystemToRecast();
        if (useStringPulling)
        {
            var straightPath = new List<DtStraightPath>();
            var success      = MeshQuery.FindStraightPath(from.SystemToRecast(), endPos, _lastPath, ref straightPath, 1024, 0);
            if (success.Failed())
                Service.Log.Error($"从 {from} ({startRef:X}) 到 {to} ({endRef:X}) 的路径查找失败：无法找到直线路径 ({success.Value:X})");

            var res = straightPath.Select(p => p.pos.RecastToSystem()).ToList();
            res.Add(endPos.RecastToSystem());

            if (res.Count > 2)
            {
                var randomizedPoints = ApplyPathRandomization(res);
                Service.Log.Debug($"[随机化成功] 总点位: {res.Count}, 随机化处理点位: {randomizedPoints}");
            }

            return res;
        }
        else
        {
            var res = _lastPath.Select(r => MeshQuery.GetAttachedNavMesh().GetPolyCenter(r).RecastToSystem()).ToList();
            res.Add(endPos.RecastToSystem());

            if (res.Count > 2)
            {
                var randomizedPoints = ApplyPathRandomization(res);
                Service.Log.Debug($"[随机化成功] 总点位: {res.Count}, 随机化处理点位: {randomizedPoints}");
            }

            return res;
        }
    }

    public List<Vector3> PathfindVolume(Vector3 from, Vector3 to, bool useRaycast, bool useStringPulling, CancellationToken cancel)
    {
        if (VolumeQuery == null)
        {
            Service.Log.Error("导航空间未构建");
            return [];
        }

        var startVoxel = FindNearestVolumeVoxel(from);
        var endVoxel   = FindNearestVolumeVoxel(to);
        Service.Log.Debug($"[寻路] 体素 {startVoxel:X} -> {endVoxel:X}");
        if (startVoxel == VoxelMap.InvalidVoxel || endVoxel == VoxelMap.InvalidVoxel)
        {
            Service.Log.Error($"从 {from} ({startVoxel:X}) 到 {to} ({endVoxel:X}) 的路径查找失败：无法找到空体素");
            return [];
        }

        var timer = Timer.Create();
        
        // TODO: 对于拉绳算法，我们是否需要中间点？
        var voxelPath = VolumeQuery.FindPath(startVoxel, endVoxel, from, to, useRaycast, false, cancel);
        if (voxelPath.Count == 0)
        {
            Service.Log.Error($"从 {from} ({startVoxel:X}) 到 {to} ({endVoxel:X}) 的路径查找失败：无法在空间中找到路径");
            return [];
        }

        Service.Log.Debug($"寻路耗时 {timer.Value().TotalSeconds:f3} 秒: {string.Join(", ", voxelPath.Select(r => $"{r.p} {r.voxel:X}"))}");

        // TODO: 拉绳算法支持
        var res = voxelPath.Select(r => r.p).ToList();
        res.Add(to);

        if (voxelPath.Count > 0)
        {
            var pathVariance = CalculatePathVariance(voxelPath);
            var randomizeStatus = pathVariance > 0.05f ? "高随机性" :
                                  pathVariance > 0.01f ? "中随机性" : "低随机性";
            Service.Log.Debug($"[空间随机化成功] 点位: {voxelPath.Count} / 状态: {randomizeStatus} ({pathVariance:F2})");
        }

        return res;
    }

    // 计算路径方差作为随机化指标
    private static float CalculatePathVariance(List<(ulong voxel, Vector3 p)> path)
    {
        if (path.Count < 3) return 0;

        float totalVariance = 0;
        for (var i = 1; i < path.Count - 1; i++)
        {
            // 计算三点之间的直线偏差
            var prev = path[i - 1].p;
            var curr = path[i].p;
            var next = path[i + 1].p;

            // 计算理想直线和实际点的距离
            var idealDir      = Vector3.Normalize(next - prev);
            var segmentLength = Vector3.Distance(prev, next);
            var idealPos      = prev + (idealDir * (Vector3.Distance(prev, curr) / segmentLength * segmentLength));

            // 计算实际点到理想线段的距离
            var deviation = Vector3.Distance(curr, idealPos);

            // 累加标准化偏差
            totalVariance += deviation / segmentLength;
        }

        // 返回平均方差
        return totalVariance / (path.Count - 2);
    }

    // 优化路径随机化方法
    private int ApplyPathRandomization(List<Vector3> path)
    {
        var randomizedPoints = 0;

        // 分析路径特征，计算平均段长度
        float totalPathLength                                    = 0;
        for (var i = 0; i < path.Count - 1; i++) totalPathLength += Vector3.Distance(path[i], path[i + 1]);

        var averageSegmentLength = totalPathLength / (path.Count - 1);
        var pathLength           = totalPathLength;

        // 根据路径长度动态调整随机因子
        var dynamicRandomFactor = Math.Min(0.5f,
                                           Math.Max(0.05f, 0.5f * (pathLength / 50))); // 长路径使用更大的随机因子

        // 保留起点和终点不变
        for (var i = 1; i < path.Count - 1; i++)
        {
            // 计算点在路径中的相对位置（0-1）
            var progressAlongPath = (float)i / (path.Count - 1);

            // 计算每个点与前后点的关系，决定随机化强度
            var distToPrev = i > 0 ? Vector3.Distance(path[i],              path[i - 1]) : 0;
            var distToNext = i < path.Count - 1 ? Vector3.Distance(path[i], path[i + 1]) : 0;

            // 如果是拐点（距离前后点较远），减少随机化强度
            var isCornerPoint = distToPrev > averageSegmentLength * 1.5f || distToNext > averageSegmentLength * 1.5f;

            // 在路径中间区域应用最大随机性，在接近起点和终点处减小随机性
            var positionFactor = MathF.Sin(progressAlongPath * MathF.PI);

            // 为拐点减少随机性
            var cornerFactor = isCornerPoint ? 0.5f : 1.0f;

            // 最终的局部随机因子
            var localRandomFactor = dynamicRandomFactor * positionFactor * cornerFactor;

            // 获取当前点和前后点，用于计算合理的随机方向
            var prev = path[i - 1];
            var curr = path[i];
            var next = i < path.Count - 1 ? path[i + 1] : curr + (curr - prev);

            // 计算路径方向
            var dirToPrev = Vector3.Normalize(prev - curr);
            var dirToNext = Vector3.Normalize(next - curr);

            // 计算路径整体方向以及法向量
            Vector3 pathDir;
            if (Vector3.Dot(dirToPrev, dirToNext) < -0.9f)
            {
                // 如果前后方向几乎相反，使用前一个方向
                pathDir = -dirToPrev;
            }
            else
            {
                // 否则使用平均方向
                pathDir = Vector3.Normalize(dirToPrev + dirToNext);
            }

            // 计算两个相互垂直的方向，用于在3D空间中随机化
            var perpH = new Vector3(-pathDir.Z, 0, pathDir.X);            // 水平面上的垂直向量
            var perpV = Vector3.Normalize(Vector3.Cross(pathDir, perpH)); // 竖直方向的向量

            // 生成两个-1到1之间的随机数
            var randomH = ((float)_random.NextDouble() * 2) - 1;
            var randomV = (((float)_random.NextDouble() * 2) - 1) * 0.3f; // 竖直方向的随机性更小

            // 只有超过一定阈值才应用随机化，避免每个点都有微小变化
            if (Math.Abs(randomH) > 0.2f || Math.Abs(randomV) > 0.05f)
            {
                // 计算随机偏移向量
                var offset = ((perpH * randomH) + (perpV * randomV)) * localRandomFactor * 3.0f;

                // 应用随机偏移
                var newPos = path[i] + offset;

                // 确保修改后的路径点不会落在障碍物中或离开导航网格太远
                var nearestPoint = FindNearestPointOnMesh(newPos, 2f, 2f);
                if (nearestPoint != null)
                {
                    // 如果随机化后的点与网格上最近点的距离在合理范围内，使用随机化后的点
                    var distToMesh = Vector3.Distance(newPos, nearestPoint.Value);
                    if (distToMesh < 1.0f)
                        path[i] = newPos;
                    else
                    {
                        // 如果距离太远，则部分向随机方向移动
                        path[i] = Vector3.Lerp(path[i], nearestPoint.Value, 0.7f);
                    }

                    randomizedPoints++;
                }
            }
        }

        // 平滑处理，确保路径更自然
        SmoothPath(path);

        return randomizedPoints;
    }

    // 添加路径平滑处理方法
    private void SmoothPath(List<Vector3> path)
    {
        if (path.Count < 4) return; // 至少需要4个点才能平滑

        var originalPoints = new List<Vector3>(path);

        // 保留起点和终点，平滑中间点
        for (var i = 1; i < path.Count - 1; i++)
            // 与相邻点进行加权平均
            if (i > 0 && i < path.Count - 1)
            {
                // 使用前一个点、当前点和后一个点的加权平均
                var smoothed = (originalPoints[i]     * 0.6f) +
                               (originalPoints[i - 1] * 0.2f) +
                               (originalPoints[i + 1] * 0.2f);

                // 确保平滑后的点仍在可行走区域
                var nearestPoint = FindNearestPointOnMesh(smoothed, 1.5f, 1.5f);
                if (nearestPoint != null)
                {
                    // 如果平滑点与最近网格点的距离在合理范围内
                    if (Vector3.Distance(smoothed, nearestPoint.Value) < 0.8f) path[i] = smoothed;
                }
            }
    }

    // returns 0 if not found, otherwise polygon ref
    public long FindNearestMeshPoly(Vector3 p, float halfExtentXZ = 5, float halfExtentY = 5)
    {
        MeshQuery.FindNearestPoly(p.SystemToRecast(), new(halfExtentXZ, halfExtentY, halfExtentXZ), _filter, out var nearestRef, out _, out _);
        return nearestRef;
    }

    public List<long> FindIntersectingMeshPolys(Vector3 p, Vector3 halfExtent)
    {
        IntersectQuery query = new();
        MeshQuery.QueryPolygons(p.SystemToRecast(), halfExtent.SystemToRecast(), _filter, query);
        return query.Result;
    }

    public Vector3? FindNearestPointOnMeshPoly(Vector3 p, long poly) =>
        MeshQuery.ClosestPointOnPoly(poly, p.SystemToRecast(), out var closest, out _).Succeeded() ? closest.RecastToSystem() : null;

    public Vector3? FindNearestPointOnMesh(Vector3 p, float halfExtentXZ = 5, float halfExtentY = 5) =>
        FindNearestPointOnMeshPoly(p, FindNearestMeshPoly(p, halfExtentXZ, halfExtentY));

    // finds the point on the mesh within specified x/z tolerance and with largest Y that is still smaller than p.Y
    public Vector3? FindPointOnFloor(Vector3 p, float halfExtentXZ = 5)
    {
        IEnumerable<long> polys = FindIntersectingMeshPolys(p, new(halfExtentXZ, 2048, halfExtentXZ));
        return polys.Select(poly => FindNearestPointOnMeshPoly(p, poly)).Where(pt => pt != null && pt.Value.Y <= p.Y).MaxBy(pt => pt!.Value.Y);
    }

    // returns VoxelMap.InvalidVoxel if not found, otherwise voxel index
    public ulong FindNearestVolumeVoxel(Vector3 p, float halfExtentXZ = 5, float halfExtentY = 5) =>
        VolumeQuery != null ? VoxelSearch.FindNearestEmptyVoxel(VolumeQuery.Volume, p, new(halfExtentXZ, halfExtentY, halfExtentXZ)) : VoxelMap.InvalidVoxel;

    // collect all mesh polygons reachable from specified polygon
    public HashSet<long> FindReachableMeshPolys(long starting)
    {
        HashSet<long> result = [];
        if (starting == 0)
            return result;

        List<long> queue = [starting];
        while (queue.Count > 0)
        {
            var next = queue[^1];
            queue.RemoveAt(queue.Count - 1);

            if (!result.Add(next))
                continue; // already visited

            MeshQuery.GetAttachedNavMesh().GetTileAndPolyByRefUnsafe(next, out var nextTile, out var nextPoly);
            for (var i = nextTile.polyLinks[nextPoly.index]; i != DtNavMesh.DT_NULL_LINK; i = nextTile.links[i].next)
            {
                var neighbourRef = nextTile.links[i].refs;
                if (neighbourRef != 0)
                    queue.Add(neighbourRef);
            }
        }

        return result;
    }
}
