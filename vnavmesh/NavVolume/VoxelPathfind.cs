using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Threading;

namespace Navmesh.NavVolume;

/// <summary>
/// 提供基于体素地图的路径查找功能，使用 A* 算法实现。
/// </summary>
public class VoxelPathfind
{
    #region 公共属性
    
    /// <summary>
    /// 获取当前使用的体素地图
    /// </summary>
    public VoxelMap Volume { get; }

    /// <summary>
    /// 获取节点列表的跨度视图
    /// </summary>
    public Span<Node> NodeSpan => CollectionsMarshal.AsSpan(Nodes);
    
    #endregion

    #region 私有字段
    
    private readonly Random                 Random     = new();
    private readonly List<Node>             Nodes      = [];
    private readonly Dictionary<ulong, int> NodeLookup = new();
    private readonly List<int>              OpenList   = [];
    private          int                    BestNodeIndex;
    private          ulong                  GoalVoxel;
    private          Vector3                GoalPos;
    private          bool                   UseRaycast;
    
    #endregion

    #region 常量
    
    private const bool  AllowReopen           = false;
    private const float RaycastLimitSq        = float.MaxValue;
    private const float VerticalSafetyBuffer  = 3f;
    private const float EdgeDetectionDistance = 10f;
    
    #endregion

    /// <summary>
    /// 初始化新的体素寻路实例
    /// </summary>
    /// <param name="volume">要使用的体素地图</param>
    public VoxelPathfind(VoxelMap volume) => 
        Volume = volume;

    #region 公共方法

    /// <summary>
    /// 在体素地图中查找从起点到终点的路径
    /// </summary>
    /// <param name="fromVoxel">起始体素</param>
    /// <param name="toVoxel">目标体素</param>
    /// <param name="fromPos">起始位置</param>
    /// <param name="toPos">目标位置</param>
    /// <param name="useRaycast">是否使用射线检测进行优化</param>
    /// <param name="returnIntermediatePoints">是否返回中间点</param>
    /// <param name="cancel">取消令牌</param>
    /// <returns>找到的路径点列表</returns>
    public List<(ulong voxel, Vector3 p)> FindPath(
        ulong fromVoxel, ulong toVoxel, Vector3 fromPos, Vector3 toPos, bool useRaycast, bool returnIntermediatePoints, CancellationToken cancel)
    {
        UseRaycast = useRaycast;

        Start(fromVoxel, toVoxel, fromPos, toPos);
        Execute(cancel);

        var path = BuildPathToVisitedNode(BestNodeIndex, returnIntermediatePoints);

        if (path.Count > 2)
            ApplyVoxelPathRandomization(path);

        if (path.Count > 2)
            AdjustPathHeightNearObstacles(path);

        return path;
    }

    /// <summary>
    /// 初始化寻路过程的开始状态
    /// </summary>
    /// <param name="fromVoxel">起始体素</param>
    /// <param name="toVoxel">目标体素</param>
    /// <param name="fromPos">起始位置</param>
    /// <param name="toPos">目标位置</param>
    public void Start(ulong fromVoxel, ulong toVoxel, Vector3 fromPos, Vector3 toPos)
    {
        Nodes.Clear();
        NodeLookup.Clear();
        OpenList.Clear();
        BestNodeIndex = 0;
        if (fromVoxel == VoxelMap.InvalidVoxel || toVoxel == VoxelMap.InvalidVoxel)
        {
            Service.Log.Error($"无效的输入单元格: {fromVoxel:X} -> {toVoxel:X}");
            return;
        }

        GoalVoxel = toVoxel;
        GoalPos = toPos;

        var randomFactor = GenerateRandomFactor();
        Nodes.Add(new()
        {
            HScore = HeuristicDistance(fromVoxel, fromPos),
            Voxel = fromVoxel,
            ParentIndex = 0,
            OpenHeapIndex = -1,
            Position = fromPos,
            RandomFactor = randomFactor
        });

        NodeLookup[fromVoxel] = 0;
        AddToOpen(0);
    }

    /// <summary>
    /// 执行寻路算法，直到找到路径或达到最大步数
    /// </summary>
    /// <param name="cancel">取消令牌</param>
    /// <param name="maxSteps">最大执行步数</param>
    public void Execute(CancellationToken cancel, int maxSteps = 1000000)
    {
        for (var i = 0; i < maxSteps; ++i)
        {
            if (!ExecuteStep())
                return;
            if ((i & 0x3ff) == 0)
                cancel.ThrowIfCancellationRequested();
        }
    }

    /// <summary>
    /// 执行单步寻路搜索
    /// </summary>
    /// <returns>如果搜索应继续则返回true，否则返回false</returns>
    public bool ExecuteStep()
    {
        var nodeSpan = NodeSpan;
        if (OpenList.Count == 0 || nodeSpan[BestNodeIndex].HScore <= 0)
            return false;

        var curNodeIndex = PopMinOpen();
        ref var curNode = ref nodeSpan[curNodeIndex];

        var curVoxel = curNode.Voxel;
        foreach (var dest in EnumerateNeighbours(curVoxel, 0, -1, 0))
            VisitNeighbour(curNodeIndex, dest);
        foreach (var dest in EnumerateNeighbours(curVoxel, 0, +1, 0))
            VisitNeighbour(curNodeIndex, dest);
        foreach (var dest in EnumerateNeighbours(curVoxel, -1, 0, 0))
            VisitNeighbour(curNodeIndex, dest);
        foreach (var dest in EnumerateNeighbours(curVoxel, +1, 0, 0))
            VisitNeighbour(curNodeIndex, dest);
        foreach (var dest in EnumerateNeighbours(curVoxel, 0, 0, -1))
            VisitNeighbour(curNodeIndex, dest);
        foreach (var dest in EnumerateNeighbours(curVoxel, 0, 0, +1))
            VisitNeighbour(curNodeIndex, dest);

        return true;
    }

    #endregion

    #region 路径构建方法

    /// <summary>
    /// 根据访问过的节点构建路径
    /// </summary>
    /// <param name="nodeIndex">结束节点索引</param>
    /// <param name="returnIntermediatePoints">是否返回中间点</param>
    /// <returns>构建的路径</returns>
    private List<(ulong voxel, Vector3 p)> BuildPathToVisitedNode(int nodeIndex, bool returnIntermediatePoints)
    {
        var res = new List<(ulong voxel, Vector3 p)>();
        if (nodeIndex < Nodes.Count)
        {
            var nodeSpan = NodeSpan;
            ref var lastNode = ref nodeSpan[nodeIndex];
            res.Add((lastNode.Voxel, lastNode.Position));
            while (nodeSpan[nodeIndex].ParentIndex != nodeIndex)
            {
                ref var prevNode = ref nodeSpan[nodeIndex];
                var nextIndex = prevNode.ParentIndex;
                ref var nextNode = ref nodeSpan[nextIndex];
                if (returnIntermediatePoints)
                {
                    var delta = nextNode.Position - prevNode.Position;
                    foreach (var v in VoxelSearch.EnumerateVoxelsInLine(Volume, prevNode.Voxel, nextNode.Voxel, prevNode.Position, nextNode.Position))
                        res.Add((v.voxel, prevNode.Position + (v.t * delta)));
                }
                else
                    res.Add((nextNode.Voxel, nextNode.Position));

                nodeIndex = nextIndex;
            }

            res.Reverse();
        }

        return res;
    }

    /// <summary>
    /// 枚举指定体素的相邻体素
    /// </summary>
    private IEnumerable<ulong> EnumerateNeighbours(ulong voxel, int dx, int dy, int dz)
    {
        var l0Desc = Volume.Levels[0];
        var l1Desc = Volume.Levels[1];
        var l2Desc = Volume.Levels[2];
        var l0Index = VoxelMap.DecodeIndex(ref voxel);
        var l1Index = VoxelMap.DecodeIndex(ref voxel);
        var l2Index = VoxelMap.DecodeIndex(ref voxel);
        var l0Coords = l0Desc.IndexToVoxel(l0Index);
        var l1Coords = l1Desc.IndexToVoxel(l1Index);
        var l2Coords = l2Desc.IndexToVoxel(l2Index);

        if (l2Index != VoxelMap.IndexLevelMask)
        {
            // starting from L2 node
            var l2Neighbour = (l2Coords.x + dx, l2Coords.y + dy, l2Coords.z + dz);
            if (l2Desc.InBounds(l2Neighbour))
            {
                // L2->L2 in same L1 tile
                var neighbourVoxel = VoxelMap.EncodeIndex(l2Desc.VoxelToIndex(l2Neighbour));
                neighbourVoxel = VoxelMap.EncodeIndex(l1Index, neighbourVoxel);
                neighbourVoxel = VoxelMap.EncodeIndex(l0Index, neighbourVoxel);
                if (Volume.IsEmpty(neighbourVoxel)) yield return neighbourVoxel;
                yield break;
            }
        }

        if (l1Index != VoxelMap.IndexLevelMask)
        {
            // starting from L1 node -or- L2 node at the boundary
            var l1Neighbour = (l1Coords.x + dx, l1Coords.y + dy, l1Coords.z + dz);
            if (l1Desc.InBounds(l1Neighbour))
            {
                // L1/L2->L1 in same L0 tile
                var neighbourVoxel = VoxelMap.EncodeIndex(l1Desc.VoxelToIndex(l1Neighbour));
                neighbourVoxel = VoxelMap.EncodeIndex(l0Index, neighbourVoxel);
                if (Volume.IsEmpty(neighbourVoxel))
                {
                    // destination L1 is fully empty
                    yield return neighbourVoxel;
                }
                else if (l2Index != VoxelMap.IndexLevelMask)
                {
                    // L2->L2 across L1 border (but in same L0)
                    var l2X = dx == 0 ? l2Coords.x : dx > 0 ? 0 : l2Desc.NumCellsX - 1;
                    var l2Y = dy == 0 ? l2Coords.y : dy > 0 ? 0 : l2Desc.NumCellsY - 1;
                    var l2Z = dz == 0 ? l2Coords.z : dz > 0 ? 0 : l2Desc.NumCellsZ - 1;
                    var l2NeighbourVoxel = VoxelMap.EncodeSubIndex(neighbourVoxel, l2Desc.VoxelToIndex(l2X, l2Y, l2Z), 2);
                    if (Volume.IsEmpty(l2NeighbourVoxel)) yield return l2NeighbourVoxel;
                }
                else
                {
                    // L1->L2 is same L0, enumerate all empty border voxels
                    foreach (var v in EnumerateBorder(neighbourVoxel, 2, dx, dy, dz))
                        if (Volume.IsEmpty(v))
                            yield return v;
                }

                yield break;
            }
        }

        //if (l0Index != VoxelMap.IndexLevelMask) - this is always true
        {
            // starting from L0 node -or- L1/L2 node at the boundary
            var l0Neighbour = (l0Coords.x + dx, l0Coords.y + dy, l0Coords.z + dz);
            if (l0Desc.InBounds(l0Neighbour))
            {
                var neighbourVoxel = VoxelMap.EncodeIndex(l0Desc.VoxelToIndex(l0Neighbour));
                if (Volume.IsEmpty(neighbourVoxel))
                {
                    // destination L0 is fully empty
                    yield return neighbourVoxel;
                }
                else if (l1Index != VoxelMap.IndexLevelMask)
                {
                    // L1/L2 across L0 border
                    var l1X = dx == 0 ? l1Coords.x : dx > 0 ? 0 : l1Desc.NumCellsX - 1;
                    var l1Y = dy == 0 ? l1Coords.y : dy > 0 ? 0 : l1Desc.NumCellsY - 1;
                    var l1Z = dz == 0 ? l1Coords.z : dz > 0 ? 0 : l1Desc.NumCellsZ - 1;
                    var l1NeighbourVoxel = VoxelMap.EncodeSubIndex(neighbourVoxel, l1Desc.VoxelToIndex(l1X, l1Y, l1Z), 1);
                    if (Volume.IsEmpty(l1NeighbourVoxel))
                    {
                        // L1/L2 -> L1
                        yield return l1NeighbourVoxel;
                    }
                    else if (l2Index != VoxelMap.IndexLevelMask)
                    {
                        // L2->L2 across L0 border
                        var l2X = dx == 0 ? l2Coords.x : dx > 0 ? 0 : l2Desc.NumCellsX - 1;
                        var l2Y = dy == 0 ? l2Coords.y : dy > 0 ? 0 : l2Desc.NumCellsY - 1;
                        var l2Z = dz == 0 ? l2Coords.z : dz > 0 ? 0 : l2Desc.NumCellsZ - 1;
                        var l2NeighbourVoxel = VoxelMap.EncodeSubIndex(l1NeighbourVoxel, l2Desc.VoxelToIndex(l2X, l2Y, l2Z), 2);
                        if (Volume.IsEmpty(l2NeighbourVoxel)) yield return l2NeighbourVoxel;
                    }
                    else
                    {
                        // L1->L2 across L0 border
                        foreach (var v in EnumerateBorder(l1NeighbourVoxel, 2, dx, dy, dz))
                            if (Volume.IsEmpty(v))
                                yield return v;
                    }
                }
                else
                {
                    // L0->L1/L2
                    foreach (var v1 in EnumerateBorder(neighbourVoxel, 1, dx, dy, dz))
                        if (Volume.IsEmpty(v1))
                        {
                            // L0->L1
                            yield return v1;
                        }
                        else
                        {
                            foreach (var v2 in EnumerateBorder(v1, 2, dx, dy, dz))
                                if (Volume.IsEmpty(v2))
                                {
                                    // L0->L2
                                    yield return v2;
                                }
                        }
                }
            }
        }
    }

    /// <summary>
    /// 枚举指定等级的边界体素
    /// </summary>
    private IEnumerable<ulong> EnumerateBorder(ulong voxel, int level, int dx, int dy, int dz)
    {
        var ld = Volume.Levels[level];
        var (xmin, xmax) = dx == 0 ? (0, ld.NumCellsX - 1) : dx > 0 ? (0, 0) : (ld.NumCellsX - 1, ld.NumCellsX - 1);
        var (ymin, ymax) = dy == 0 ? (0, ld.NumCellsY - 1) : dy > 0 ? (0, 0) : (ld.NumCellsY - 1, ld.NumCellsY - 1);
        var (zmin, zmax) = dz == 0 ? (0, ld.NumCellsZ - 1) : dz > 0 ? (0, 0) : (ld.NumCellsZ - 1, ld.NumCellsZ - 1);
        for (var z = zmin; z <= zmax; ++z)
        for (var x = xmin; x <= xmax; ++x)
        for (var y = ymin; y <= ymax; ++y)
            yield return VoxelMap.EncodeSubIndex(voxel, ld.VoxelToIndex(x, y, z), level);
    }

    /// <summary>
    /// 访问邻居节点，如果找到更好的路径则更新
    /// </summary>
    private void VisitNeighbour(int parentIndex, ulong nodeVoxel)
    {
        var nodeIndex = NodeLookup.GetValueOrDefault(nodeVoxel, -1);
        if (nodeIndex < 0)
        {
            // first time we're visiting this node, calculate heuristic
            nodeIndex = Nodes.Count;

            var randomFactor = GenerateRandomFactor();

            Nodes.Add(new()
            {
                GScore = float.MaxValue,
                HScore = float.MaxValue,
                Voxel = nodeVoxel,
                ParentIndex = parentIndex,
                OpenHeapIndex = -1,
                RandomFactor = randomFactor
            });

            NodeLookup[nodeVoxel] = nodeIndex;
        }
        else if (!AllowReopen && Nodes[nodeIndex].OpenHeapIndex < 0)
        {
            // in closed list already
            return;
        }

        var nodeSpan = NodeSpan;
        ref var parentNode = ref nodeSpan[parentIndex];
        var enterPos = nodeVoxel == GoalVoxel ? GoalPos : VoxelSearch.FindClosestVoxelPoint(Volume, nodeVoxel, parentNode.Position);
        var nodeG = CalculateGScore(ref parentNode, nodeVoxel, enterPos, ref parentIndex);
        ref var curNode = ref nodeSpan[nodeIndex];
        if (nodeG + 0.00001f < curNode.GScore)
        {
            // new path is better
            curNode.GScore = nodeG;
            curNode.HScore = HeuristicDistance(nodeVoxel, enterPos);
            curNode.ParentIndex = parentIndex;
            curNode.Position = enterPos;
            AddToOpen(nodeIndex);

            if (curNode.HScore < Nodes[BestNodeIndex].HScore)
                BestNodeIndex = nodeIndex;
        }
    }

    #endregion

    #region 路径优化与安全调整

    /// <summary>
    /// 在障碍物附近调整路径高度以确保安全
    /// </summary>
    private void AdjustPathHeightNearObstacles(List<(ulong voxel, Vector3 p)> path)
    {
        if (path.Count < 3) return;

        // 第一次扫描：检测障碍物顶部边缘
        var edgePoints = DetectObstacleEdges(path);

        // 第二次扫描：应用高度偏移
        for (var i = 1; i < path.Count - 1; i++)
            // 检查当前点是否在障碍物边缘附近
            if (IsNearObstacleEdge(i, edgePoints))
            {
                var (voxel, pos) = path[i];

                // 使用检测障碍物下方的射线确定要抬升的高度
                var heightAdjustment = CalculateHeightAdjustment(pos);

                var newPos = pos with { Y = pos.Y + heightAdjustment };

                // 验证新位置是否有效
                if (IsValidPosition(newPos))
                    path[i] = (voxel, newPos);
                else
                {
                    // 尝试应用较小的高度调整
                    newPos = pos with { Y = pos.Y + (heightAdjustment * 0.5f) };
                    if (IsValidPosition(newPos)) path[i] = (voxel, newPos);
                }
            }

        SmoothVoxelPath(path);
    }

    /// <summary>
    /// 对体素路径应用随机化，增加路径多样性
    /// </summary>
    private void ApplyVoxelPathRandomization(List<(ulong voxel, Vector3 p)> path)
    {
        // 只处理中间点，保持起点和终点不变
        for (var i = 1; i < path.Count - 1; i++)
        {
            // 计算在路径中的相对位置
            var progressAlongPath = (float)i / (path.Count - 1);

            // 在路径中间区域应用最大随机性，接近两端应用较小随机性
            var positionFactor = MathF.Sin(progressAlongPath * MathF.PI);

            // 计算随机因子，体素路径的随机化强度比网格路径小一些，以保持稳定性
            var localRandomFactor = Service.Config.VoxelPathfindRandomFactor * 0.7f * positionFactor;

            var currPos = path[i].p;

            // 计算与相邻点的方向，确定合理的随机化方向
            var prevPos = i > 0 ? path[i - 1].p : currPos;
            var nextPos = i < path.Count - 1 ? path[i + 1].p : currPos;

            // 计算路径方向
            var dirToPrev = Vector3.Normalize(prevPos - currPos);
            var dirToNext = Vector3.Normalize(nextPos - currPos);
            Vector3 pathDir;

            if (Vector3.Dot(dirToPrev, dirToNext) < -0.8f)
            {
                // 如果是急转弯，使用单一方向
                pathDir = -dirToPrev;
            }
            else
            {
                // 使用平均方向
                pathDir = Vector3.Normalize(dirToPrev + dirToNext);
            }

            // 计算垂直于路径的方向向量
            var perpH = new Vector3(-pathDir.Z, 0, pathDir.X);            // 水平面上垂直的向量
            var perpV = Vector3.Normalize(Vector3.Cross(pathDir, perpH)); // 垂直方向

            // 增强垂直方向的随机性 - 优先抬高而非降低
            var randomH = (((float)Random.NextDouble() * 2) - 1) * localRandomFactor * 1.5f;
            var randomV = (float)Random.NextDouble() * 0.8f * localRandomFactor * 0.8f; // 垂直方向更倾向于正向偏移

            // 检查下方是否有障碍物
            var hasObstacleBelow = HasObstacleBelow(currPos);

            // 如果下方有障碍物，增加垂直方向的正向偏移
            if (hasObstacleBelow) randomV = MathF.Abs(randomV) * 1.5f; // 确保向上偏移，并增大偏移量

            // 降低阈值，确保更多点被随机化
            if (Math.Abs(randomH) > 0.05f || Math.Abs(randomV) > 0.02f)
            {
                // 计算随机偏移向量
                var offset = ((perpH * randomH) + (perpV * randomV)) * 2.5f;

                // 应用随机偏移
                var newPos = currPos + offset;

                // 检查新位置是否有效
                if (IsValidPosition(newPos))
                    path[i] = (path[i].voxel, newPos);
                else
                {
                    // 如果随机后的位置无效，尝试较小的偏移
                    var smallerOffset = offset * 0.5f;
                    var fallbackPos = currPos + smallerOffset;

                    if (IsValidPosition(fallbackPos))
                        path[i] = (path[i].voxel, fallbackPos);
                }
            }
        }

        // 平滑路径，确保路径连贯
        SmoothVoxelPath(path);
    }

    /// <summary>
    /// 平滑体素路径，使路径更加连贯平滑
    /// </summary>
    private void SmoothVoxelPath(List<(ulong voxel, Vector3 p)> path)
    {
        if (path.Count < 4) return;

        // 保存原始点用于插值计算
        var originalPoints = new List<Vector3>(path.Count);
        foreach (var point in path) originalPoints.Add(point.p);

        // 平滑中间点
        for (var i = 1; i < path.Count - 1; i++)
        {
            // 与相邻点进行加权平均
            var smoothed = (originalPoints[i] * 0.7f) +
                           (originalPoints[i - 1] * 0.15f) +
                           (originalPoints[i + 1] * 0.15f);

            // 确保平滑后的位置有效并且不会降低太多高度
            if (IsValidPosition(smoothed) && smoothed.Y >= originalPoints[i].Y - 0.3f)
                path[i] = (path[i].voxel, smoothed);
            // 如果平滑会导致高度下降过多，保持原始高度并只平滑水平方向
            else if (smoothed.Y < originalPoints[i].Y - 0.3f)
            {
                var horizontalSmoothed = new Vector3(smoothed.X, originalPoints[i].Y, smoothed.Z);
                if (IsValidPosition(horizontalSmoothed)) path[i] = (path[i].voxel, horizontalSmoothed);
            }
        }
    }

    /// <summary>
    /// 检测路径中的障碍物顶部边缘
    /// </summary>
    private List<int> DetectObstacleEdges(List<(ulong voxel, Vector3 p)> path)
    {
        var edgePoints = new List<int>();

        // 扫描路径查找高度变化显著的点
        for (var i = 1; i < path.Count - 1; i++)
        {
            var prevPos = path[i - 1].p;
            var currPos = path[i].p;
            var nextPos = path[i + 1].p;

            // 检查垂直方向的显著变化
            var heightDiffPrev = MathF.Abs(currPos.Y - prevPos.Y);
            var heightDiffNext = MathF.Abs(nextPos.Y - currPos.Y);

            // 检测下方是否有障碍物
            var hasObstacleBelow = HasObstacleBelow(currPos);

            // 如果存在高度变化且下方有障碍物，认为是障碍物边缘
            if ((heightDiffPrev > 0.5f || heightDiffNext > 0.5f) && hasObstacleBelow) edgePoints.Add(i);
        }

        return edgePoints;
    }

    /// <summary>
    /// 检查点是否在障碍物边缘附近
    /// </summary>
    private static bool IsNearObstacleEdge(int pathIndex, List<int> edgePoints)
    {
        // 直接检查是否是边缘点
        if (edgePoints.Contains(pathIndex))
            return true;

        // 检查是否在边缘点附近
        foreach (var edgeIndex in edgePoints)
            if (MathF.Abs(pathIndex - edgeIndex) <= 2)
                return true;

        return false;
    }

    /// <summary>
    /// 检测点下方是否有障碍物
    /// </summary>
    private bool HasObstacleBelow(Vector3 position)
    {
        // 向下发射射线检测是否有障碍物
        var downVector = new Vector3(position.X, position.Y - EdgeDetectionDistance, position.Z);

        var fromVoxelResult = Volume.FindLeafVoxel(position);
        var toVoxelResult = Volume.FindLeafVoxel(downVector);

        // 如果任一点无效，返回 true（假设有障碍物）
        if (!fromVoxelResult.empty || !toVoxelResult.empty)
            return true;

        var fromVoxel = fromVoxelResult.voxel;
        var toVoxel = toVoxelResult.voxel;

        // 如果不能直线访问，说明下方有障碍物
        return !VoxelSearch.LineOfSight(Volume, fromVoxel, toVoxel, position, downVector);
    }

    /// <summary>
    /// 计算需要的高度调整
    /// </summary>
    private float CalculateHeightAdjustment(Vector3 position)
    {
        // 基础安全缓冲区
        var adjustment = VerticalSafetyBuffer;

        // 向下发射射线检测到最近的障碍物表面
        var maxCheckDistance = 5.0f;
        var currPos = position;
        var step = 0.2f;
        var distance = 0.0f;

        while (distance < maxCheckDistance)
        {
            var downPos = new Vector3(currPos.X, currPos.Y - step, currPos.Z);

            var fromVoxelResult = Volume.FindLeafVoxel(currPos);
            var toVoxelResult = Volume.FindLeafVoxel(downPos);

            // 如果任一点无效，返回当前调整值
            if (!fromVoxelResult.empty || !toVoxelResult.empty)
                return adjustment + (0.5f * distance);

            var fromVoxel = fromVoxelResult.voxel;
            var toVoxel = toVoxelResult.voxel;

            if (!VoxelSearch.LineOfSight(Volume, fromVoxel, toVoxel, currPos, downPos))
            {
                // 找到障碍物，根据距离调整高度偏移
                return adjustment + (0.5f * distance); // 距离越远，偏移越大
            }

            currPos = downPos;
            distance += step;
        }

        // 如果没检测到障碍物，返回基础值
        return adjustment;
    }

    /// <summary>
    /// 检查位置是否有效
    /// </summary>
    private bool IsValidPosition(Vector3 pos) =>
        VoxelSearch.FindNearestEmptyVoxel(Volume, pos, new Vector3(3.0f, 5.0f, 3.0f)) != VoxelMap.InvalidVoxel;

    /// <summary>
    /// 检测是否是从障碍物边缘穿越
    /// </summary>
    private bool IsEdgeTransition(Vector3 from, Vector3 to)
    {
        // 检查高度变化
        var heightDiff = MathF.Abs(to.Y - from.Y);

        // 如果高度变化显著，就认为可能是障碍物边缘
        if (heightDiff > 0.5f)
            return true;

        var hasObstacleBelowFrom = HasObstacleBelow(from);
        var hasObstacleBelowTo = HasObstacleBelow(to);

        // 如果路径点下方的障碍物状态发生变化（一个点下方有障碍物而另一个没有）
        // 说明可能是穿过了障碍物边缘
        return hasObstacleBelowFrom != hasObstacleBelowTo;
    }

    #endregion
    
    private float GenerateRandomFactor() =>
        (((float)Random.NextDouble() * 2) - 1) * Service.Config.VoxelPathfindRandomFactor;

    private float CalculateGScore(ref Node parent, ulong destVoxel, Vector3 destPos, ref int parentIndex)
    {
        if (UseRaycast)
        {
            // 检查与祖父节点的视线
            var grandParentIndex = parent.ParentIndex;
            ref var grandParentNode = ref NodeSpan[grandParentIndex];
            var dist = (grandParentNode.Position - destPos).LengthSquared();
            if (dist <= RaycastLimitSq && VoxelSearch.LineOfSight(Volume, grandParentNode.Voxel, destVoxel, grandParentNode.Position, destPos))
            {
                parentIndex = grandParentIndex;
                return grandParentNode.GScore + MathF.Sqrt(dist);
            }
        }

        var distance = (parent.Position - destPos).Length();

        // 检测是否经过障碍物顶部
        var isEdgeTransition = IsEdgeTransition(parent.Position, destPos);
        if (isEdgeTransition)
        {
            // 对接近障碍物顶部的路径增加惩罚，鼓励寻找更远离障碍物顶部的路径
            distance *= 1.2f;
        }

        ref var node = ref NodeSpan[NodeLookup[destVoxel]];
        var randomInfluence = 1.0f + node.RandomFactor;
        distance *= randomInfluence;

        return parent.GScore + distance;
    }

    private float HeuristicDistance(ulong nodeVoxel, Vector3 v)
    {
        if (nodeVoxel == GoalVoxel)
            return 0;

        var baseDistance = (v - GoalPos).Length() * 0.999f;

        if (nodeVoxel != GoalVoxel)
        {
            var nodeIndex = NodeLookup.GetValueOrDefault(nodeVoxel, -1);
            if (nodeIndex >= 0)
            {
                // 使用已存储的随机因子，但动态调整影响程度
                // 距离目标越近，随机因子影响越小，确保能收敛到目标
                var targetDist = (v - GoalPos).Length();
                var targetInfluence = Math.Min(1.0f, targetDist / 10.0f); // 距离目标 10 单位以内，随机因子影响逐渐减小

                // 增加随机因子影响，提高随机性
                var randomInfluence = 1.0f + (NodeSpan[nodeIndex].RandomFactor * 0.7f * targetInfluence);
                return baseDistance * randomInfluence;
            }
        }

        return baseDistance;
    }

    #region 开放列表操作
    
    /// <summary>
    /// 将节点添加到开放列表
    /// </summary>
    private void AddToOpen(int nodeIndex)
    {
        ref var node = ref NodeSpan[nodeIndex];
        if (node.OpenHeapIndex < 0)
        {
            node.OpenHeapIndex = OpenList.Count;
            OpenList.Add(nodeIndex);
        }

        // 更新位置
        PercolateUp(node.OpenHeapIndex);
    }

    /// <summary>
    /// 从开放列表中移除并返回最小评分的节点索引
    /// </summary>
    private int PopMinOpen()
    {
        var nodeSpan = NodeSpan;
        var nodeIndex = OpenList[0];
        OpenList[0] = OpenList[OpenList.Count - 1];
        OpenList.RemoveAt(OpenList.Count - 1);
        nodeSpan[nodeIndex].OpenHeapIndex = -1;
        if (OpenList.Count > 0)
        {
            nodeSpan[OpenList[0]].OpenHeapIndex = 0;
            PercolateDown(0);
        }

        return nodeIndex;
    }

    /// <summary>
    /// 将节点向上调整到堆中合适的位置
    /// </summary>
    private void PercolateUp(int heapIndex)
    {
        var nodeSpan = NodeSpan;
        var nodeIndex = OpenList[heapIndex];
        var parent = (heapIndex - 1) >> 1;
        while (heapIndex > 0 && HeapLess(ref nodeSpan[nodeIndex], ref nodeSpan[OpenList[parent]]))
        {
            OpenList[heapIndex] = OpenList[parent];
            nodeSpan[OpenList[heapIndex]].OpenHeapIndex = heapIndex;
            heapIndex = parent;
            parent = (heapIndex - 1) >> 1;
        }

        OpenList[heapIndex] = nodeIndex;
        nodeSpan[nodeIndex].OpenHeapIndex = heapIndex;
    }

    /// <summary>
    /// 将节点向下调整到堆中合适的位置
    /// </summary>
    private void PercolateDown(int heapIndex)
    {
        var nodeSpan = NodeSpan;
        var nodeIndex = OpenList[heapIndex];
        var maxSize = OpenList.Count;
        while (true)
        {
            var child1 = (heapIndex << 1) + 1;
            if (child1 >= maxSize)
                break;
            var child2 = child1 + 1;
            if (child2 == maxSize || HeapLess(ref nodeSpan[OpenList[child1]], ref nodeSpan[OpenList[child2]]))
            {
                if (HeapLess(ref nodeSpan[OpenList[child1]], ref nodeSpan[nodeIndex]))
                {
                    OpenList[heapIndex] = OpenList[child1];
                    nodeSpan[OpenList[heapIndex]].OpenHeapIndex = heapIndex;
                    heapIndex = child1;
                }
                else
                    break;
            }
            else if (HeapLess(ref nodeSpan[OpenList[child2]], ref nodeSpan[nodeIndex]))
            {
                OpenList[heapIndex] = OpenList[child2];
                nodeSpan[OpenList[heapIndex]].OpenHeapIndex = heapIndex;
                heapIndex = child2;
            }
            else
                break;
        }

        OpenList[heapIndex] = nodeIndex;
        nodeSpan[nodeIndex].OpenHeapIndex = heapIndex;
    }

    /// <summary>
    /// 比较两个节点的评分大小，用于堆排序
    /// </summary>
    private static bool HeapLess(ref Node nodeL, ref Node nodeR)
    {
        var fl = nodeL.GScore + nodeL.HScore;
        var fr = nodeR.GScore + nodeR.HScore;

        fl *= 1.0f + (nodeL.RandomFactor * 0.3f);
        fr *= 1.0f + (nodeR.RandomFactor * 0.3f);

        if (fl + 0.00001f < fr)
            return true;
        if (fr + 0.00001f < fl)
            return false;
        return nodeL.GScore > nodeR.GScore; // 当f值相等时，优先选择g值更大的节点
    }
    
    #endregion
}

/// <summary>
/// 表示寻路过程中的一个节点
/// </summary>
public struct Node
{
    /// <summary>从起点到当前节点的实际代价</summary>
    public float GScore;
    
    /// <summary>从当前节点到目标的估计代价</summary>
    public float HScore;
    
    /// <summary>对应的体素地图索引</summary>
    public ulong Voxel;
    
    /// <summary>父节点在节点列表中的索引</summary>
    public int ParentIndex;
    
    /// <summary>在开放列表中的索引，-1表示在关闭列表中</summary>
    public int OpenHeapIndex;
    
    /// <summary>节点的实际位置</summary>
    public Vector3 Position;
    
    /// <summary>影响路径随机性的随机因子</summary>
    public float RandomFactor;
}
