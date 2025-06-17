using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
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

    public readonly DtNavMeshQuery MeshQuery;
    public readonly VoxelPathfind? VolumeQuery;
    
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
                if (Service.Config.RandomPath_Mode == Config.RandomPathMode.RECAST_LINEPULL)
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
                } else
                {
                    var pathList = _lastPath.Select(r => MeshQuery.GetAttachedNavMesh().GetPolyCenter(r)).ToList();
                    var res = _directPathGen.GenerateDirectPath(from.SystemToRecast(), to.SystemToRecast(), pathList, _filter, Service.Config.RandomPath_CPull_Directness, Service.Config.RandomPath_CPull_Natrualness);

                    return [.. res.Select(p => p.RecastToSystem())];
                }
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

    public List<Vector3> PathfindVolume(Vector3 from, Vector3 to, bool useRaycast, bool useStringPulling, Action<float>? progressCallback, CancellationToken cancel)
    {
        if (VolumeQuery == null)
        {
            Service.Log.Error("导航尚未构建");
            return [];
        }

        var startVoxel = FindNearestVolumeVoxel(from);
        var endVoxel   = FindNearestVolumeVoxel(to);
        Service.Log.Debug($"[寻路] 体素 {startVoxel:X} -> {endVoxel:X}");
        if (startVoxel == VoxelMap.InvalidVoxel || endVoxel == VoxelMap.InvalidVoxel)
        {
            Service.Log.Error($"无法找到从 {from} ({startVoxel:X}) 到 {to} ({endVoxel:X}) 的路径：未能找到空体素");
            return [];
        }

        // 如果起点和终点足够近，并且有视线，则直接返回一条直线路径
        if ((from - to).LengthSquared() < 100.0f && VoxelSearch.LineOfSight(VolumeQuery.Volume, startVoxel, endVoxel, from, to))
        {
            Service.Log.Debug($"[寻路] 从 {from} 到 {to} 间存在视线，直接返回直线路径");
            var directPath = new List<Vector3> { from, to };

            return directPath;
        }

        var timer = Timer.Create();

        var voxelPath = VolumeQuery.FindPath(startVoxel, endVoxel, from, to, useRaycast, false, progressCallback, cancel);

        if (voxelPath.Count == 0)
        {
            Service.Log.Error($"无法找到从 {from} ({startVoxel:X}) 到 {to} ({endVoxel:X}) 的路径：未能找到路径");
            return [];
        }

        Service.Log.Debug($"寻路耗时 {timer.Value().TotalSeconds:f3}秒: {string.Join(", ", voxelPath.Select(r => $"{r.p} {r.voxel:X}"))}");

        List<Vector3> res = [.. voxelPath.Select(r => r.p)];
        res.Add(to);

        return res;
    }

    // returns 0 if not found, otherwise polygon ref
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public long FindNearestMeshPoly(Vector3 p, float halfExtentXZ = 5, float halfExtentY = 5)
    {
        MeshQuery.FindNearestPoly(p.SystemToRecast(), new(halfExtentXZ, halfExtentY, halfExtentXZ), _filter, out var nearestRef, out _, out _);
        return nearestRef;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public List<long> FindIntersectingMeshPolys(Vector3 p, Vector3 halfExtent)
    {
        IntersectQuery query = new();
        MeshQuery.QueryPolygons(p.SystemToRecast(), halfExtent.SystemToRecast(), _filter, query);
        return query.Result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3? FindNearestPointOnMeshPoly(Vector3 p, long poly) =>
        MeshQuery.ClosestPointOnPoly(poly, p.SystemToRecast(), out var closest, out _).Succeeded() ? closest.RecastToSystem() : null;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3? FindNearestPointOnMesh(Vector3 p, float halfExtentXZ = 5, float halfExtentY = 5) =>
        FindNearestPointOnMeshPoly(p, FindNearestMeshPoly(p, halfExtentXZ, halfExtentY));

    // finds the point on the mesh within specified x/z tolerance and with largest Y that is still smaller than p.Y
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3? FindPointOnFloor(Vector3 p, float halfExtentXZ = 5)
    {
        IEnumerable<long> polys = FindIntersectingMeshPolys(p, new(halfExtentXZ, 2048, halfExtentXZ));
        return polys.Select(poly => FindNearestPointOnMeshPoly(p, poly)).Where(pt => pt != null && pt.Value.Y <= p.Y).MaxBy(pt => pt!.Value.Y);
    }

    // returns VoxelMap.InvalidVoxel if not found, otherwise voxel index
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
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
