using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Threading;
using DotRecast.Core;
using DotRecast.Core.Numerics;
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
    internal static readonly IDtQueryFilter _filter2 = new RandomizedQueryFilter();
    internal readonly PathRandomizer _randomizer;

    private readonly DirectPathGenerator _directPathGen;

    public  List<long> LastPath => _lastPath;
    private List<long> _lastPath = [];

    public NavmeshQuery(Navmesh navmesh)
    {
        MeshQuery = new(navmesh.Mesh);
        if (navmesh.Volume != null)
            VolumeQuery = new(navmesh.Volume);

        _directPathGen = new(MeshQuery, agentRadius: 0.5f);

        _randomizer = new(MeshQuery);
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

        var options = 0;
        if (useRaycast)
            options |= DtFindPathOptions.DT_FINDPATH_ANY_ANGLE;

        var opt = new DtFindPathOption(options, useRaycast ? 5 : 0);

        MeshQuery.FindPath(startRef, endRef, from.SystemToRecast(), to.SystemToRecast(), Service.Config.UseRandomPathGen ? _filter2 : _filter, ref _lastPath, opt);
        if (_lastPath.Count == 0)
        {
            Service.Log.Error($"从 {from} ({startRef:X}) 到 {to} ({endRef:X}) 的路径查找失败：无法在网格上找到路径");
            return [];
        }

        Service.Log.Debug($"寻路耗时 {timer.Value().TotalSeconds:f3} 秒: {string.Join(", ", _lastPath.Select(r => r.ToString("X")))}");

        var endPos = to.SystemToRecast();

        if (useStringPulling)
        {
            if (Service.Config.UseRandomPathGen)
            {
                var pathList = _lastPath.Select(r => MeshQuery.GetAttachedNavMesh().GetPolyCenter(r)).ToList();                
                
                var straightPath = new List<DtStraightPath>();
                var success = MeshQuery.FindStraightPath(from.SystemToRecast(), endPos, _lastPath, ref straightPath, 1024, DtStraightPathOptions.DT_STRAIGHTPATH_AREA_CROSSINGS);
                if (success.Failed())
                    Service.Log.Error($"从 {from} ({startRef:X}) 到 {to} ({endRef:X}) 的路径查找失败：无法找到直线路径 ({success.Value:X})");

                /*Plugin.OriginalPath_Locked = true;
                Plugin.OriginalPath = [.. straightPath.Select(p => p.pos.RecastToSystem())];
                Plugin.OriginalPath_Locked = false;*/

                _randomizer.SetPointGenerationParameters(Service.Config.RandomPath_MinPointsPerSeg, Service.Config.RandomPath_MaxPointsPerSeg);
                _randomizer.SetRandomizationParameters(3f, Service.Config.RandomPath_MaxDeviationRatio, Service.Config.RandomPath_Randomness);
                _randomizer.SetPathCenteringParameters(Service.Config.RandomPath_UsePathCentering, Service.Config.RandomPath_CenteringStrength, Service.Config.RandomPath_MaxCenteringDist, Service.Config.RandomPath_RandomOffsetRatio);
                
                var res = _randomizer.RandomizePath([.. straightPath.Select(p => p.pos.RecastToSystem())], _filter);
                res.Add(endPos.RecastToSystem());

                return res;
            }
            else
            {
                var straightPath = new List<DtStraightPath>();
                var success = MeshQuery.FindStraightPath(from.SystemToRecast(), endPos, _lastPath, ref straightPath, 1024, DtStraightPathOptions.DT_STRAIGHTPATH_AREA_CROSSINGS);
                if (success.Failed())
                    Service.Log.Error($"从 {from} ({startRef:X}) 到 {to} ({endRef:X}) 的路径查找失败：无法找到直线路径 ({success.Value:X})");

                var res = straightPath.Select(p => p.pos.RecastToSystem()).ToList();
                res.Add(endPos.RecastToSystem());

                return res;
            }
        }
        else
        {
            var res = _lastPath.Select(r => MeshQuery.GetAttachedNavMesh().GetPolyCenter(r).RecastToSystem()).ToList();
            res.Add(endPos.RecastToSystem());

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
            totalVariance += deviation / Math.Max(0.001f, segmentLength);
        }

        // 返回平均方差
        return path.Count > 2 ? totalVariance / (path.Count - 2) : 0;
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
