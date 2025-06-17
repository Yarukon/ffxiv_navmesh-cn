using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Threading;

namespace Navmesh.NavVolume;

public class VoxelPathfind(VoxelMap volume)
{
    private const    int                    DefaultInitialCapacity = 1024;
    private readonly List<Node>             Nodes                  = new(DefaultInitialCapacity);
    private readonly Dictionary<ulong, int> NodeLookup             = new(DefaultInitialCapacity);
    private readonly List<int>              OpenList               = new(DefaultInitialCapacity);
    private          int                    BestNodeIndex;
    private          ulong                  GoalVoxel;
    private          Vector3                GoalPos;
    private          bool                   UseRaycast;

    public static float CurRandomFactor { get; set; }
    
    private const bool  AllowReopen    = false; // this is extremely expensive and doesn't seem to actually improve the result
    private const float RaycastLimitSq = float.MaxValue;

    public VoxelMap Volume { get; } = volume;

    public Span<Node> NodeSpan =>
        CollectionsMarshal.AsSpan(Nodes);

    private readonly Vector3 CharaHalfExtents = new(0.5f, 2f, 0.5f);

    public List<(ulong voxel, Vector3 p)> FindPath(
        ulong fromVoxel, ulong toVoxel, Vector3 fromPos, Vector3 toPos, bool useRaycast, bool returnIntermediatePoints, 
        Action<float>? progressCallback, CancellationToken cancel)
    {
        UseRaycast = useRaycast;
        GenerateRandomThisTime();
        Start(fromVoxel, toVoxel, fromPos, toPos);
        Execute(cancel, progressCallback, CalculateDynamicMaxSteps(fromPos, toPos));

        // --- 修改开始 ---
        // 1. 先构建原始路径
        var rawPath = BuildPathToVisitedNode(BestNodeIndex, returnIntermediatePoints);

        if (Service.Config.VoxelPathfindPostProcess)
        {
            // 2. 对原始路径进行后处理以调整垂直净空
            var adjustedPathPoints = AdjustPathForClearance(rawPath, CharaHalfExtents);

            // 3. 将调整后的坐标点包装成期望的返回类型
            // voxel 信息被丢弃，因为点的位置已经改变，原始voxel不再精确
            var finalPath = adjustedPathPoints.Select(p => (voxel: (ulong)0, p)).ToList();

            return finalPath;
        }
        // --- 修改结束 ---
        return rawPath;
    }

    public List<Vector3> AdjustPathForClearance(List<(ulong voxel, Vector3 p)> rawPath, Vector3 characterHalfExtents)
    {
        if (rawPath.Count < 2)
            return [.. rawPath.Select(node => node.p)];

        var adjustedPath = new List<Vector3>();

        // 定义后处理参数
        const int NumProbes = 5;
        const float ProbeInterval = 0.75f;
        const float MaxAngleDegrees = 56.0f;
        const float MaxAngleRadians = MaxAngleDegrees * (float)(Math.PI / 180.0);

        // 用于从起点到第一个点的最大爬坡角度
        const float MaxClimbAngDeg = 25.0f; // 最大爬坡角度
        const float MaxClimbAngRad = MaxClimbAngDeg * (float)(Math.PI / 180.0);

        const float InterpolationStep = 1.0f;

        // ==================================================================
        // ---- 1. 起点处理：P0 固定不变 ----
        // ==================================================================
        var p0_final = rawPath[0].p;
        adjustedPath.Add(p0_final);
        var previousAdjustedPoint = p0_final;

        // ==================================================================
        // ---- 2. 特殊处理第一个路径点 P1 (索引为 1) ----
        // ==================================================================
        var p1_orig = rawPath[1].p;
        Vector3 p1_final;
        bool p1_resolved = false;

        // 只有在存在 P2 时，才能执行前瞻性修正
        if (rawPath.Count > 2)
        {
            var p2_orig = rawPath[2].p;
            var segment_1_2 = p2_orig - p1_orig;
            var segmentLength = segment_1_2.Length();

            if (segmentLength > 0.1f) // 避免除零
            {
                var direction = segment_1_2 / segmentLength;
                const int lookAheadSteps = 10; // 最多探测10步
                const float lookAheadInterval = 1.0f; // 每步的距离

                // 从 p1_orig 开始，沿着 p1->p2 方向探测
                for (int i = 0; i <= lookAheadSteps; i++)
                {
                    float dist = i * lookAheadInterval;
                    if (dist > segmentLength) break; // 不超出 p2

                    var probePoint = p1_orig + (direction * dist);

                    // 检查从 P0 到探测点的坡度
                    var delta = probePoint - p0_final;
                    var horizontalDist = new Vector2(delta.X, delta.Z).Length();

                    if (horizontalDist < 0.1f) continue;

                    var maxVerticalChange = horizontalDist * MathF.Tan(MaxClimbAngRad);

                    // 如果坡度在允许范围内
                    if (Math.Abs(delta.Y) <= maxVerticalChange)
                    {
                        // 直接检查探测点本身是否是一个有效的、有足够净空间的位置。
                        // 这是最可靠的检查方法。
                        if (VoxelSearch.FindNearestEmptyVoxel(Volume, probePoint, characterHalfExtents) != VoxelMap.InvalidVoxel)
                        {
                            // 找到了一个坡度和净空都合格的候选点！
                            // 使用 AdjustSinglePoint 进行最终微调，找到最佳垂直位置
                            p1_final = AdjustSinglePoint(probePoint, NumProbes, ProbeInterval, characterHalfExtents);
                            adjustedPath.Add(p1_final);
                            previousAdjustedPoint = p1_final;
                            p1_resolved = true;
                            break; // 成功找到，跳出循环
                        }
                    }
                }
            }
        }

        // 如果前瞻性修正失败（或无法进行），则使用标准的回退方案
        if (!p1_resolved)
        {
            var p1_forwardDir = Vector3.Normalize(p1_orig - p0_final);
            p1_final = ProcessPoint(p1_orig, p0_final, p1_forwardDir, MaxAngleRadians, NumProbes, ProbeInterval, characterHalfExtents);
            adjustedPath.Add(p1_final);
            previousAdjustedPoint = p1_final;
        }

        // ==================================================================
        // ---- 3. 循环处理路径的其余部分 (从 P2 开始) ----
        // ==================================================================
        for (int i = 2; i < rawPath.Count - 1; i++)
        {
            var segmentStartPoint = previousAdjustedPoint;
            var segmentEndPointOriginal = rawPath[i].p;
            var segmentVector = segmentEndPointOriginal - segmentStartPoint;
            var segmentLength = segmentVector.Length();

            if (segmentLength < 0.01f) continue;

            var segmentDirection = Vector3.Normalize(segmentVector); // 这是当前段的前进方向

            if (segmentLength < InterpolationStep)
            {
                var finalAdjustedPoint = ProcessPoint(segmentEndPointOriginal, segmentStartPoint, segmentDirection, MaxAngleRadians, NumProbes, ProbeInterval, characterHalfExtents);
                adjustedPath.Add(finalAdjustedPoint);
                previousAdjustedPoint = finalAdjustedPoint;
                continue;
            }

            for (float distance = InterpolationStep; distance < segmentLength; distance += InterpolationStep)
            {
                var currentOriginalPoint = segmentStartPoint + segmentDirection * distance;
                var adjustedIntermediatePoint = ProcessPoint(currentOriginalPoint, previousAdjustedPoint, segmentDirection, MaxAngleRadians, NumProbes, ProbeInterval, characterHalfExtents);
                adjustedPath.Add(adjustedIntermediatePoint);
                previousAdjustedPoint = adjustedIntermediatePoint;
            }

            // *** MODIFIED CALL ***
            var finalSegmentPoint = ProcessPoint(segmentEndPointOriginal, previousAdjustedPoint, segmentDirection, MaxAngleRadians, NumProbes, ProbeInterval, characterHalfExtents);
            adjustedPath.Add(finalSegmentPoint);
            previousAdjustedPoint = finalSegmentPoint;
        }

        // ==================================================================
        // ---- 4. Final Destination Point Processing (Special Handling with Reverse Probing) ----
        // ==================================================================
        // 只有当路径长度大于等于2时，才有终点需要处理
        if (rawPath.Count >= 2)
        {
            var lastOriginalPoint = rawPath[^1].p;
            var lastAdjustedIntermediatePoint = previousAdjustedPoint; // 这是倒数第二个调整好的点

            var segmentVector = lastOriginalPoint - lastAdjustedIntermediatePoint;
            var segmentLength = segmentVector.Length();

            if (segmentLength > 0.01f)
            {
                var segmentDirection = Vector3.Normalize(segmentVector);
                // 我们从目标点(终点)开始，沿着路径段向后回溯探测
                // 寻找第一个既满足坡度要求，又在物理上有效的点
                const float probeStep = 1.0f; // 与InterpolationStep保持一致或自定义

                for (float dist = segmentLength; dist >= 0; dist -= probeStep)
                {
                    // dist=segmentLength是终点本身, dist=0是上一个点本身
                    var probePoint = lastAdjustedIntermediatePoint + (segmentDirection * dist);

                    // a. 检查从上一个点到当前探测点的坡度是否允许
                    var delta = probePoint - lastAdjustedIntermediatePoint;
                    var horizontalDist = new Vector2(delta.X, delta.Z).Length();

                    // 避免除以零，并确保有足够的水平距离来进行有意义的坡度检查
                    if (horizontalDist < 0.1f && Math.Abs(delta.Y) > 0.1f)
                    {
                        // 几乎是垂直的墙，跳过这个探测点
                        continue;
                    }

                    var maxVerticalChange = horizontalDist * MathF.Tan(MaxClimbAngRad);

                    if (Math.Abs(delta.Y) <= maxVerticalChange)
                    {
                        // b. 坡度允许，再检查该点是否有足够的站立空间
                        if (VoxelSearch.FindNearestEmptyVoxel(Volume, probePoint, characterHalfExtents) != VoxelMap.InvalidVoxel)
                        {
                            // c. 找到了一个完全有效的点！进行最终的垂直调整并添加它
                            var finalAdjustedPoint = AdjustSinglePoint(probePoint, NumProbes, ProbeInterval, characterHalfExtents);
                            adjustedPath.Add(finalAdjustedPoint);
                            break; // 找到后立即退出循环
                        }
                    }

                    // 如果dist已经非常小，再继续探测没有意义
                    if (dist < probeStep) break;
                }
            }
        }

        return adjustedPath;
    }

    /// <summary>
    /// 处理单个路径点：进行垂直调整、角度限制和最终安全检查。
    /// </summary>
    private Vector3 ProcessPoint(Vector3 currentPoint, Vector3 previousPoint, Vector3 forwardDirection, float maxAngleRadians, int numProbes, float probeInterval, Vector3 characterHalfExtents)
    {
        // 1. 坡度限制 (与之前相同)
        var delta = currentPoint - previousPoint;
        var horizontalDist = new Vector2(delta.X, delta.Z).Length();
        if (horizontalDist > 0.01f)
        {
            var maxVerticalChange = horizontalDist * MathF.Tan(maxAngleRadians);
            if (Math.Abs(delta.Y) > maxVerticalChange)
            {
                delta.Y = maxVerticalChange * Math.Sign(delta.Y);
                currentPoint = previousPoint + delta;
            }
        }

        // 2. *** 新增：水平居中调整 ***
        // 在进行最终垂直定位前，先找到水平方向上的最佳位置
        // 使用角色宽度的一半作为探测距离是一个合理的选择
        float horizontalProbeDist = characterHalfExtents.X;
        var horizontallyAdjustedPoint = FindHorizontalCenter(currentPoint, forwardDirection, horizontalProbeDist, characterHalfExtents);

        // 3. 最终垂直位置微调 (使用水平调整后的点)
        return AdjustSinglePoint(horizontallyAdjustedPoint, numProbes, probeInterval, characterHalfExtents);
    }

    /// <summary>
    /// 对单个点进行垂直净空检查和调整，返回一个理想的垂直位置。
    /// </summary>
    private Vector3 AdjustSinglePoint(Vector3 point, int numProbes, float probeInterval, Vector3 characterHalfExtents)
    {
        var (upClearance, downClearance) = CheckVerticalClearance(point, numProbes, probeInterval, characterHalfExtents);

        float offsetY = 0;
        int totalAvailableProbes = upClearance + downClearance;

        if (upClearance < numProbes && downClearance < numProbes)
        {
            float totalHeight = totalAvailableProbes * probeInterval;
            float targetYFromBottom = totalHeight * 0.4f;
            float currentYFromBottom = downClearance * probeInterval;
            offsetY = targetYFromBottom - currentYFromBottom;
        }
        else if (upClearance < numProbes)
        {
            offsetY = -(numProbes - upClearance) * probeInterval;
        }
        else if (downClearance < numProbes)
        {
            offsetY = (numProbes - downClearance) * probeInterval;
        }

        return new Vector3(point.X, point.Y + offsetY, point.Z);
    }

    /// <summary>
    /// 使用探针和 FindNearestEmptyVoxel 检查给定位置的垂直净空。
    /// </summary>
    private (int upwardClearance, int downwardClearance) CheckVerticalClearance(Vector3 position, int numProbes, float probeInterval, Vector3 characterHalfExtents)
    {
        int upClear = 0;
        int downClear = 0;

        // 向上检查
        for (int i = 1; i <= numProbes; i++)
        {
            var probePos = position + new Vector3(0, i * probeInterval, 0);
            // 检查返回的ID是否有效
            if (VoxelSearch.FindNearestEmptyVoxel(Volume, probePos, characterHalfExtents) == VoxelMap.InvalidVoxel)
                break;
            upClear++;
        }

        // 向下检查
        for (int i = 1; i <= numProbes; i++)
        {
            var probePos = position - new Vector3(0, i * probeInterval, 0);
            if (VoxelSearch.FindNearestEmptyVoxel(Volume, probePos, characterHalfExtents) == VoxelMap.InvalidVoxel)
                break;
            downClear++;
        }

        return (upClear, downClear);
    }

    /// <summary>
    /// 探测一个点的左右空间，并尝试将其移动到水平空间的中心。
    /// "左右"是相对于前进方向而言的。
    /// </summary>
    /// <param name="centerPoint">要检查和调整的候选点。</param>
    /// <param name="forwardDirection">角色当前的前进方向，用于确定左右。</param>
    /// <param name="probeDistance">向左右探测的距离。一个好的默认值是 characterHalfExtents.X。</param>
    /// <param name="characterHalfExtents">角色的半尺寸。</param>
    /// <returns>经过水平调整后的新位置。</returns>
    private Vector3 FindHorizontalCenter(Vector3 centerPoint, Vector3 forwardDirection, float probeDistance, Vector3 characterHalfExtents)
    {
        // 如果前进方向很小，无法判断左右，则直接返回原点
        if (forwardDirection.LengthSquared() < 0.01f)
        {
            return centerPoint;
        }

        // 1. 计算相对右方向的向量 (Y-up 坐标系)
        // 通过前进方向和世界向上方向(Y轴)的叉积得到
        var right = Vector3.Normalize(Vector3.Cross(forwardDirection, Vector3.UnitY));

        // 2. 定义左右探测点
        var probeLeft = centerPoint - right * probeDistance;
        var probeRight = centerPoint + right * probeDistance;

        // 3. 检查三个点的有效性
        // 注意：我们只检查水平调整，不改变Y坐标
        bool isCenterValid = VoxelSearch.FindNearestEmptyVoxel(Volume, centerPoint, characterHalfExtents) != VoxelMap.InvalidVoxel;
        bool isLeftValid = VoxelSearch.FindNearestEmptyVoxel(Volume, probeLeft, characterHalfExtents) != VoxelMap.InvalidVoxel;
        bool isRightValid = VoxelSearch.FindNearestEmptyVoxel(Volume, probeRight, characterHalfExtents) != VoxelMap.InvalidVoxel;

        // 如果中心点本身就无效，我们无能为力，返回原点让上层逻辑处理
        if (!isCenterValid)
        {
            return centerPoint;
        }

        // 4. 根据探测结果进行调整
        if (isLeftValid && !isRightValid)
        {
            // 右边是墙，左边是空地 -> 向左移动一点以远离墙壁
            return centerPoint - right * (probeDistance / 2.0f);
        }
        else if (!isLeftValid && isRightValid)
        {
            // 左边是墙，右边是空地 -> 向右移动一点以远离墙壁
            return centerPoint + right * (probeDistance / 2.0f);
        }

        // 如果两边都有效（开阔地）或两边都无效（狭窄通道），则保持在中间是最佳选择
        return centerPoint;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Start(ulong fromVoxel, ulong toVoxel, Vector3 fromPos, Vector3 toPos)
    {
        Nodes.Clear();
        NodeLookup.Clear();
        OpenList.Clear();
        BestNodeIndex = 0;
        if (fromVoxel == VoxelMap.InvalidVoxel || toVoxel == VoxelMap.InvalidVoxel)
        {
            Service.Log.Error($"Bad input cells: {fromVoxel:X} -> {toVoxel:X}");
            return;
        }

        GoalVoxel = toVoxel;
        GoalPos   = toPos;

        // 预分配容量以减少重新分配
        if (Nodes.Capacity < DefaultInitialCapacity)
            Nodes.Capacity = DefaultInitialCapacity;
        if (NodeLookup.EnsureCapacity(DefaultInitialCapacity) < DefaultInitialCapacity)
            NodeLookup.EnsureCapacity(DefaultInitialCapacity);
        if (OpenList.Capacity < DefaultInitialCapacity)
            OpenList.Capacity = DefaultInitialCapacity;

        Nodes.Add(new()
        {
            HScore = HeuristicDistance(fromVoxel, fromPos), Voxel = fromVoxel, ParentIndex = 0, OpenHeapIndex = -1, Position = fromPos
        }); // start's parent is self
        NodeLookup[fromVoxel] = 0;
        AddToOpen(0);
    }

    public void Execute(CancellationToken cancel, Action<float>? progressCallback = null, int maxSteps = 1000000)
    {
        for (var i = 0; i < maxSteps; ++i)
        {
            if (!ExecuteStep())
                return;
            if ((i & 0x3ff) == 0)
            {
                cancel.ThrowIfCancellationRequested();
                progressCallback?.Invoke((float)i / maxSteps);
            }
        }
    }

    // returns whether search is to be terminated; on success, first node of the open list would contain found goal
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool ExecuteStep()
    {
        var nodeSpan = NodeSpan;
        if (OpenList.Count == 0 || nodeSpan[BestNodeIndex].HScore <= 0)
            return false;

        var     curNodeIndex = PopMinOpen();
        ref var curNode      = ref nodeSpan[curNodeIndex];

        // 早期终止检查：如果当前最佳节点足够接近目标，提前结束搜索
        if (IsCloseEnoughToGoal(nodeSpan[BestNodeIndex].Position))
            return false;

        var curVoxel = curNode.Voxel;
        VisitNeighboursInAllDirections(curNodeIndex, curVoxel);
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void VisitNeighboursInAllDirections(int curNodeIndex, ulong curVoxel)
    {
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
    }

    private List<(ulong voxel, Vector3 p)> BuildPathToVisitedNode(int nodeIndex, bool returnIntermediatePoints)
    {
        var res = new List<(ulong voxel, Vector3 p)>();
        if (nodeIndex < Nodes.Count)
        {
            var     nodeSpan = NodeSpan;
            ref var lastNode = ref nodeSpan[nodeIndex];
            res.Add((lastNode.Voxel, lastNode.Position));
            while (nodeSpan[nodeIndex].ParentIndex != nodeIndex)
            {
                ref var prevNode  = ref nodeSpan[nodeIndex];
                var     nextIndex = prevNode.ParentIndex;
                ref var nextNode  = ref nodeSpan[nextIndex];
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

    private IEnumerable<ulong> EnumerateNeighbours(ulong voxel, int dx, int dy, int dz)
    {
        var l0Desc   = Volume.Levels[0];
        var l1Desc   = Volume.Levels[1];
        var l2Desc   = Volume.Levels[2];
        var l0Index  = VoxelMap.DecodeIndex(ref voxel); // should always be valid
        var l1Index  = VoxelMap.DecodeIndex(ref voxel);
        var l2Index  = VoxelMap.DecodeIndex(ref voxel);
        var l0Coords = l0Desc.IndexToVoxel(l0Index);
        var l1Coords = l1Desc.IndexToVoxel(l1Index); // not valid if l1 is invalid
        var l2Coords = l2Desc.IndexToVoxel(l2Index); // not valid if l2 is invalid

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
                // else: L2 is occupied, so we can't go there
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
                    var l2X              = dx == 0 ? l2Coords.x : dx > 0 ? 0 : l2Desc.NumCellsX - 1;
                    var l2Y              = dy == 0 ? l2Coords.y : dy > 0 ? 0 : l2Desc.NumCellsY - 1;
                    var l2Z              = dz == 0 ? l2Coords.z : dz > 0 ? 0 : l2Desc.NumCellsZ - 1;
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
                    var l1X              = dx == 0 ? l1Coords.x : dx > 0 ? 0 : l1Desc.NumCellsX - 1;
                    var l1Y              = dy == 0 ? l1Coords.y : dy > 0 ? 0 : l1Desc.NumCellsY - 1;
                    var l1Z              = dz == 0 ? l1Coords.z : dz > 0 ? 0 : l1Desc.NumCellsZ - 1;
                    var l1NeighbourVoxel = VoxelMap.EncodeSubIndex(neighbourVoxel, l1Desc.VoxelToIndex(l1X, l1Y, l1Z), 1);
                    if (Volume.IsEmpty(l1NeighbourVoxel))
                    {
                        // L1/L2 -> L1
                        yield return l1NeighbourVoxel;
                    }
                    else if (l2Index != VoxelMap.IndexLevelMask)
                    {
                        // L2->L2 across L0 border
                        var l2X              = dx == 0 ? l2Coords.x : dx > 0 ? 0 : l2Desc.NumCellsX - 1;
                        var l2Y              = dy == 0 ? l2Coords.y : dy > 0 ? 0 : l2Desc.NumCellsY - 1;
                        var l2Z              = dz == 0 ? l2Coords.z : dz > 0 ? 0 : l2Desc.NumCellsZ - 1;
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void VisitNeighbour(int parentIndex, ulong nodeVoxel)
    {
        var nodeIndex = NodeLookup.GetValueOrDefault(nodeVoxel, -1);
        if (nodeIndex < 0)
        {
            // first time we're visiting this node, calculate heuristic
            nodeIndex = Nodes.Count;
            Nodes.Add(new() { GScore = float.MaxValue, HScore = float.MaxValue, Voxel = nodeVoxel, ParentIndex = parentIndex, OpenHeapIndex = -1 });
            NodeLookup[nodeVoxel] = nodeIndex;
        }
        else if (!AllowReopen && Nodes[nodeIndex].OpenHeapIndex < 0)
        {
            // in closed list already - TODO: is it possible to visit again with lower cost?..
            return;
        }

        var     nodeSpan   = NodeSpan;
        ref var parentNode = ref nodeSpan[parentIndex];
        var     enterPos   = nodeVoxel == GoalVoxel ? GoalPos : VoxelSearch.FindClosestVoxelPoint(Volume, nodeVoxel, parentNode.Position);
        var     nodeG      = CalculateGScore(ref parentNode, nodeVoxel, enterPos, ref parentIndex);
        ref var curNode    = ref nodeSpan[nodeIndex];
        if (nodeG + 0.00001f < curNode.GScore)
        {
            // new path is better
            curNode.GScore      = nodeG;
            curNode.HScore      = HeuristicDistance(nodeVoxel, enterPos);
            curNode.ParentIndex = parentIndex;
            curNode.Position    = enterPos;
            AddToOpen(nodeIndex);

            if (curNode.HScore < Nodes[BestNodeIndex].HScore)
                BestNodeIndex = nodeIndex;
        }
    }

    // 惩罚乘数：一个足够大的值，让算法“害怕”选择陡峭的路径。
    // 10.0f 意味着走一段陡峭路径的成本是其物理长度的20倍。
    private const float SteepSlopePenalty = 20.0f;

    // 陡峭角度阈值：超过这个角度的移动将被施加惩罚。
    // 建议值在 60 到 80 度之间。
    private const float MaxSlopeAngleDegrees = 70.0f;
    private const float MaxSlopeAngleRadians = MaxSlopeAngleDegrees * (float)(Math.PI / 180.0);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private float CalculateGScore(ref Node parent, ulong destVoxel, Vector3 destPos, ref int parentIndex)
    {
        float   baseDistance;
        float   parentBaseG;
        Vector3 fromPos;

        if (UseRaycast)
        {
            // check LoS from grandparent
            var     grandParentIndex = parent.ParentIndex;
            ref var grandParentNode  = ref NodeSpan[grandParentIndex];
            var     distanceSquared  = (grandParentNode.Position - destPos).LengthSquared();
            if (distanceSquared <= RaycastLimitSq && VoxelSearch.LineOfSight(Volume, grandParentNode.Voxel, destVoxel, grandParentNode.Position, destPos))
            {
                parentIndex  = grandParentIndex;
                baseDistance = MathF.Sqrt(distanceSquared);
                parentBaseG  = grandParentNode.GScore;
                fromPos      = grandParentNode.Position;
            }
            else
            {
                baseDistance = (parent.Position - destPos).Length();
                parentBaseG  = parent.GScore;
                fromPos      = parent.Position;
            }
        }
        else
        {
            baseDistance = (parent.Position - destPos).Length();
            parentBaseG  = parent.GScore;
            fromPos      = parent.Position;
        }

        // 1. 计算移动的水平和垂直分量
        var moveVector = destPos - fromPos;
        var horizontalDistance = new Vector2(moveVector.X, moveVector.Z).Length();
        var verticalDistance = Math.Abs(moveVector.Y);

        // 2. 计算惩罚乘数
        float slopePenaltyMultiplier = 1.0f; // 默认为1.0 (无惩罚)

        // 检查是否为近乎垂直的移动
        if (horizontalDistance < 0.01f) // 几乎没有水平移动
        {
            if (verticalDistance > 0.1f) // 但有明显的垂直移动
            {
                // 这是纯粹的垂直攀爬/下落，给予最大惩罚
                slopePenaltyMultiplier = SteepSlopePenalty;
            }
        }
        else
        {
            // 计算实际的移动坡度角度
            float slopeAngle = MathF.Atan(verticalDistance / horizontalDistance);

            // 如果坡度超过我们设定的阈值，则施加惩罚
            if (slopeAngle > MaxSlopeAngleRadians)
            {
                slopePenaltyMultiplier = SteepSlopePenalty;
            }
        }

        // 3. 将惩罚应用到基础移动成本上
        float finalMoveCost = baseDistance * slopePenaltyMultiplier;

        return parentBaseG + finalMoveCost;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private float HeuristicDistance(ulong nodeVoxel, Vector3 v)
    {
        if (nodeVoxel == GoalVoxel)
            return 0;

        // Apply random variation to heuristic while keeping it admissible
        // RandomThisTime is between 0 and VoxelPathfindRandomFactor
        // Multiply by small factor (0.99) to ensure heuristic remains admissible
        var baseHeuristic = (v - GoalPos).Length();
        var randomizedHeuristic = baseHeuristic * (0.999f - (CurRandomFactor * 0.01f));
        return randomizedHeuristic;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int CalculateDynamicMaxSteps(Vector3 fromPos, Vector3 toPos)
    {
        var distance = (fromPos - toPos).Length();
        var config = Service.Config;
        
        var dynamicSteps = (int)(config.VoxelPathfindMinSteps + 
                                (distance * config.VoxelPathfindMaxStepsMultiplier * config.VoxelPathfindMaxStepsBaseFactor));
        
        // 最多 100 万步
        return Math.Min(Math.Max(dynamicSteps, config.VoxelPathfindMinSteps), 100_0000);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool IsCloseEnoughToGoal(Vector3 currentPos)
    {
        var distanceToGoal = (currentPos - GoalPos).Length();
        return distanceToGoal <= Service.Config.VoxelPathfindEarlyTerminationDistance;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void AddToOpen(int nodeIndex)
    {
        ref var node = ref NodeSpan[nodeIndex];
        if (node.OpenHeapIndex < 0)
        {
            node.OpenHeapIndex = OpenList.Count;
            OpenList.Add(nodeIndex);
        }

        // update location
        PercolateUp(node.OpenHeapIndex);
    }

    // remove first (minimal) node from open heap and mark as closed
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int PopMinOpen()
    {
        var nodeSpan  = NodeSpan;
        var nodeIndex = OpenList[0];
        OpenList[0] = OpenList[^1];
        OpenList.RemoveAt(OpenList.Count - 1);
        nodeSpan[nodeIndex].OpenHeapIndex = -1;
        if (OpenList.Count > 0)
        {
            nodeSpan[OpenList[0]].OpenHeapIndex = 0;
            PercolateDown(0);
        }

        return nodeIndex;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void PercolateUp(int heapIndex)
    {
        var nodeSpan  = NodeSpan;
        var nodeIndex = OpenList[heapIndex];
        var parent    = (heapIndex - 1) >> 1;
        while (heapIndex > 0 && HeapLess(ref nodeSpan[nodeIndex], ref nodeSpan[OpenList[parent]]))
        {
            OpenList[heapIndex]                         = OpenList[parent];
            nodeSpan[OpenList[heapIndex]].OpenHeapIndex = heapIndex;
            heapIndex                                   = parent;
            parent                                      = (heapIndex - 1) >> 1;
        }

        OpenList[heapIndex]               = nodeIndex;
        nodeSpan[nodeIndex].OpenHeapIndex = heapIndex;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void PercolateDown(int heapIndex)
    {
        var nodeSpan  = NodeSpan;
        var nodeIndex = OpenList[heapIndex];
        var maxSize   = OpenList.Count;
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
                    OpenList[heapIndex]                         = OpenList[child1];
                    nodeSpan[OpenList[heapIndex]].OpenHeapIndex = heapIndex;
                    heapIndex                                   = child1;
                }
                else
                    break;
            }
            else if (HeapLess(ref nodeSpan[OpenList[child2]], ref nodeSpan[nodeIndex]))
            {
                OpenList[heapIndex]                         = OpenList[child2];
                nodeSpan[OpenList[heapIndex]].OpenHeapIndex = heapIndex;
                heapIndex                                   = child2;
            }
            else
                break;
        }

        OpenList[heapIndex]               = nodeIndex;
        nodeSpan[nodeIndex].OpenHeapIndex = heapIndex;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool HeapLess(ref Node nodeL, ref Node nodeR)
    {
        var fl = nodeL.GScore + nodeL.HScore;
        var fr = nodeR.GScore + nodeR.HScore;

        if (fl + 0.00001f < fr)
            return true;
        if (fr + 0.00001f < fl)
            return false;

        return nodeL.GScore > nodeR.GScore; // tie-break towards larger g-values
    }

    public static void GenerateRandomThisTime()
    {
        // Scale random factor to be a small value that influences path selection
        // but doesn't significantly impact pathfinding accuracy
        CurRandomFactor = Service.Config.VoxelPathfindRandomFactor > 0f ? (float)new Random().NextDouble() * Service.Config.VoxelPathfindRandomFactor : 0;
    }

    public struct Node
    {
        public float   GScore;
        public float   HScore;
        public ulong   Voxel;         // voxel map index corresponding to this node
        public int     ParentIndex;   // index in the node list of the node we entered from
        public int     OpenHeapIndex; // -1 if in closed list, otherwise index in open list
        public Vector3 Position;
    }
}
