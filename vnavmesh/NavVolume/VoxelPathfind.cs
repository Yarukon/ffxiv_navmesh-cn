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

    private readonly Vector3 CharaHalfExtents = new(5f, 5f, 5f);

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
        const float ProbeInterval = 1.5f;
        const float MaxAngleDegrees = 56.0f;
        const float MaxAngleRadians = MaxAngleDegrees * (float)(Math.PI / 180.0);

        // 用于从起点到第一个点的最大爬坡角度
        const float MaxClimbAngDeg = 35.0f; // 最大爬坡角度
        const float MaxClimbAngRad = MaxClimbAngDeg * (float)(Math.PI / 180.0);

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

                // 从 p1_orig 开始，沿着 p1->p2 方向探测
                for (int i = 0; i <= lookAheadSteps; i++)
                {
                    float dist = i * ProbeInterval;
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
                            p1_final = AdjustSinglePoint(probePoint, p0_final, NumProbes, ProbeInterval, characterHalfExtents);
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
            p1_final = ProcessPoint(p1_orig, p0_final, MaxAngleRadians, characterHalfExtents);
            adjustedPath.Add(p1_final);
            previousAdjustedPoint = p1_final;
        }

        // ==================================================================
        // ---- 3. 循环处理路径的其余部分 (从 P2 开始) ----
        // ==================================================================
        for (int i = 2; i < rawPath.Count - 1; i++) // 循环到最后一个点
        {
            var segmentStartPoint = previousAdjustedPoint; // 这是上一个调整好的点
            var segmentEndPointOriginal = rawPath[i].p;   // 这是当前要处理的原始目标点

            // 检查 segmentStartPoint 到更远的点，实现“拉绳”效果。
            // 我们需要找到从 segmentStartPoint 出发，能直接到达的最远的原始路径点。
            int furthestVisibleIndex = i; // 假设当前点 i 是最远能到达的
            for (int j = i + 1; j < rawPath.Count; j++)
            {
                // 尝试从 segmentStartPoint 直接连接到更远的点 j
                if (IsPathSegmentSafe(segmentStartPoint, rawPath[j].p, characterHalfExtents, MaxAngleRadians))
                {
                    // 如果可以安全到达，更新最远可见索引，并继续尝试更远的点
                    furthestVisibleIndex = j;
                }
                else
                {
                    // 如果无法安全到达 j，说明 j-1 是极限了，停止搜索
                    break;
                }
            }

            // 循环结束后，furthestVisibleIndex 就是从 segmentStartPoint 能安全到达的最远点的索引。
            // 这个点就是我们下一个要添加的“关键拐点”。
            var nextKeyPointOriginal = rawPath[furthestVisibleIndex].p;

            // 【修改点 2】: 我们只对这个找到的关键拐点进行一次处理，而不是沿途插值。
            // 使用您现有的 ProcessPoint 函数来对这个关键点进行最终的精细调整。
            var finalAdjustedPoint = ProcessPoint(nextKeyPointOriginal, segmentStartPoint,  MaxAngleRadians, characterHalfExtents);

            adjustedPath.Add(finalAdjustedPoint);
            previousAdjustedPoint = finalAdjustedPoint;

            // 【修改点 3】: 跳过所有被“拉直”的中间点。
            // 下一轮循环将从这个新的关键拐点之后开始。
            i = furthestVisibleIndex;
        }

        // ==================================================================
        // ---- 4. Final Destination Point Processing (Special Handling with Reverse Probing) ----
        // ==================================================================
        // 只有当路径长度大于等于2时，才有终点需要处理
        if (rawPath.Count >= 2)
        {
            var endPoint = rawPath[^1].p;
            var lastAdjustedIntermediatePoint = previousAdjustedPoint;
            bool finalPointFoundAndAdded = false;

            // --- 策略 1: 优先尝试直接到达 ---
            // 检查从最后一个调整点直接到原始终点的路径是否可行
            if (IsPathToPointValid(lastAdjustedIntermediatePoint, endPoint, MaxClimbAngRad, Volume, characterHalfExtents))
            {
                // 路径可行，直接添加原始终点并完成
                adjustedPath.Add(endPoint);
                finalPointFoundAndAdded = true;
            }
            // --- 策略 2: 如果直接到达不可行，并且路径足够长，则执行“后退探测” ---
            else if (adjustedPath.Count >= 2) // 必须至少有2个点才能“后退”
            {
                var secondToLastAdjustedPoint = adjustedPath[^2]; // 后退的基准点

                var backupVector = secondToLastAdjustedPoint - lastAdjustedIntermediatePoint;
                var backupDistance = backupVector.Length();

                if (backupDistance > 0.5f)
                {
                    var backupDirection = Vector3.Normalize(backupVector);
                    const float probeStep = 0.5f; // 后退探测的步长

                    // 从当前位置开始，沿着来路向后探测
                    for (float dist = probeStep; dist <= backupDistance; dist += probeStep)
                    {
                        // 计算一个新的“起跳点”
                        var newTakeoffPoint = lastAdjustedIntermediatePoint + backupDirection * dist;

                        // 检查从这个新的起跳点到原始终点的路径是否可行
                        if (IsPathToPointValid(newTakeoffPoint, endPoint, MaxClimbAngRad, Volume, characterHalfExtents))
                        {
                            // 找到了一个可行的起跳点！
                            // 1. 用这个新的、后退后的起跳点，替换掉原来那个不好的点
                            adjustedPath[^1] = newTakeoffPoint;

                            // 2. 添加原始终点
                            adjustedPath.Add(endPoint);

                            finalPointFoundAndAdded = true;
                            break; // 找到后立即退出
                        }
                    }
                }
            }

            if (!finalPointFoundAndAdded)
            {
                adjustedPath.Add(endPoint); // 如果所有尝试都失败，至少添加原始终点
            }
        }

        return adjustedPath;
    }

    // 辅助函数：检查从一个起点到终点的路径是否坡度平缓且终点可站立
    private bool IsPathToPointValid(Vector3 start, Vector3 end, float maxAngleRad, VoxelMap volume, Vector3 charHalfExtents)
    {
        // a. 检查坡度
        var delta = end - start;
        var horizontalDist = new Vector2(delta.X, delta.Z).Length();

        // 如果水平距离很小但垂直距离很大，视为无效（避免爬墙）
        if (horizontalDist < 0.1f && Math.Abs(delta.Y) > 0.1f)
        {
            return false;
        }

        // 允许的最大垂直变化
        var maxVerticalChange = horizontalDist * MathF.Tan(maxAngleRad);
        if (Math.Abs(delta.Y) > maxVerticalChange)
        {
            return false; // 坡度太陡
        }

        // b. 检查终点是否有站立空间
        if (VoxelSearch.FindNearestEmptyVoxel(volume, end, charHalfExtents) == VoxelMap.InvalidVoxel)
        {
            return false; // 终点被阻挡或无空间
        }

        // c. 所有检查通过
        return true;
    }

    /// <summary>
    /// 处理单个路径点：进行垂直调整、角度限制和最终安全检查。
    /// </summary>
    private Vector3 ProcessPoint(Vector3 currentPoint, Vector3 previousPoint, float maxAngleRadians, Vector3 characterHalfExtents)
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

        var finalPoint = Find3DCenter(Volume, currentPoint, characterHalfExtents);
        return finalPoint;
    }

    /// <summary>
    /// 对单个点进行垂直净空检查和调整，返回一个理想的垂直位置。
    /// </summary>
    private Vector3 AdjustSinglePoint(Vector3 point, Vector3 previousPoint, int numProbes, float probeInterval, Vector3 characterHalfExtents)
    {
        Service.Log.Debug($"[AdjustSinglePoint] START for point {point:F2} (PrevY: {previousPoint.Y:F2}). Probing +/- {numProbes * probeInterval:F1}m...");

        var (upClearance, downClearance) = CheckVerticalClearance(point, numProbes, probeInterval, characterHalfExtents);
        Service.Log.Debug($"  -> Clearance Check: Up Probes = {upClearance}, Down Probes = {downClearance}");

        float offsetY = 0;
        string decision = "None";

        // 计算路径的垂直意图
        float intendedVerticalChange = point.Y - previousPoint.Y;

        // Case 1: 上下空间都受限 (狭窄通道)
        if (upClearance < numProbes && downClearance < numProbes)
        {
            // 这部分逻辑不变，在狭窄空间里找中心是正确的
            int totalAvailableProbes = upClearance + downClearance;
            float totalHeight = totalAvailableProbes * probeInterval;
            float targetYFromBottom = totalHeight * 0.4f;
            float currentYFromBottom = downClearance * probeInterval;
            offsetY = targetYFromBottom - currentYFromBottom;
            decision = $"CRAMPED SPACE. Total height {totalHeight:F2}m. Adjusting to center.";
        }
        // Case 2: 仅上方空间受限 (接近天花板)
        else if (upClearance < numProbes)
        {
            // 【核心判断】根据路径意图决定是“钻过去”还是“飞越”
            // 使用一个小的阈值避免浮点数误差
            if (intendedVerticalChange < -0.1f)
            {
                // 意图是下降：执行“钻过去”逻辑 -> 向下移动
                offsetY = -(numProbes - upClearance) * probeInterval;
                decision = $"CEILING NEAR (Path Descending). Ducking UNDER. Pushing DOWN by {Math.Abs(offsetY):F2}m.";
            }
            else
            {
                // 意图是水平或上升：执行“飞越”逻辑 -> 向上移动以避免撞头
                // 向上推，直到角色顶部和天花板之间有一个小的安全距离
                float requiredClearance = characterHalfExtents.Y;
                float availableUpSpace = upClearance * probeInterval;
                offsetY = availableUpSpace - requiredClearance - 0.1f; // 0.1f 是安全缓冲
                                                                       // 确保我们不会因为过度补偿而向上推得太多
                offsetY = Math.Max(0, offsetY);
                decision = $"CEILING NEAR (Path Level/Ascending). Clearing OVER. Pushing UP by {offsetY:F2}m.";
            }
        }
        // Case 3: 仅下方空间受限 (接近地面)
        else if (downClearance < numProbes)
        {
            // 这个逻辑总是正确的：远离地面/下方障碍物 -> 向上移动
            offsetY = (numProbes - downClearance) * probeInterval;
            decision = $"FLOOR NEAR. Pushing UP by {offsetY:F2}m.";
        }
        // Case 4: 上下空间都开阔
        else
        {
            decision = "OPEN SPACE. No vertical adjustment needed.";
            offsetY = 0;
        }

        var finalPoint = new Vector3(point.X, point.Y + offsetY, point.Z);
        Service.Log.Debug($"  -> Decision: {decision}");
        Service.Log.Debug($"  -> Final OffsetY: {offsetY:F2}. Final Point: {finalPoint:F2}");
        Service.Log.Debug("----------------------------------------------------");

        return finalPoint;
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

    private static readonly Vector3[] ProbeDirections = new Vector3[]
    {
        // 轴向 (6)
        new Vector3(1, 0, 0), new Vector3(-1, 0, 0),
        new Vector3(0, 1, 0), new Vector3(0, -1, 0),
        new Vector3(0, 0, 1), new Vector3(0, 0, -1),
        // 面-对角线 (12)
        Vector3.Normalize(new Vector3(1, 1, 0)), Vector3.Normalize(new Vector3(1, -1, 0)), Vector3.Normalize(new Vector3(-1, 1, 0)), Vector3.Normalize(new Vector3(-1, -1, 0)),
        Vector3.Normalize(new Vector3(1, 0, 1)), Vector3.Normalize(new Vector3(1, 0, -1)), Vector3.Normalize(new Vector3(-1, 0, 1)), Vector3.Normalize(new Vector3(-1, 0, -1)),
        Vector3.Normalize(new Vector3(0, 1, 1)), Vector3.Normalize(new Vector3(0, 1, -1)), Vector3.Normalize(new Vector3(0, -1, 1)), Vector3.Normalize(new Vector3(0, -1, -1)),
        // 角-对角线 (8)
        Vector3.Normalize(new Vector3(1, 1, 1)), Vector3.Normalize(new Vector3(1, 1, -1)), Vector3.Normalize(new Vector3(1, -1, 1)), Vector3.Normalize(new Vector3(1, -1, -1)),
        Vector3.Normalize(new Vector3(-1, 1, 1)), Vector3.Normalize(new Vector3(-1, 1, -1)), Vector3.Normalize(new Vector3(-1, -1, 1)), Vector3.Normalize(new Vector3(-1, -1, -1))
    };

    private Vector3 Find3DCenter(VoxelMap volume, Vector3 position, Vector3 characterHalfExtents, float maxProbeDistance = 5.0f)
    {
        var totalOffset = Vector3.Zero;
        int validPairs = 0;

        // 我们将26个方向视为13对相反的方向
        for (int i = 0; i < ProbeDirections.Length / 2; i++)
        {
            var dir = ProbeDirections[i * 2];      // 正方向
            var antiDir = ProbeDirections[(i * 2) + 1]; // 反方向

            float distPositive = ProbeDistance(volume, position, dir, maxProbeDistance, characterHalfExtents);
            float distNegative = ProbeDistance(volume, position, antiDir, maxProbeDistance, characterHalfExtents);

            var axisOffset = dir * (distPositive - distNegative) / 2.0f;

            totalOffset += axisOffset;
            validPairs++;
        }

        if (validPairs == 0)
        {
            return position;
        }

        var averageOffset = totalOffset / validPairs;
        var optimalPosition = position + averageOffset;

        // 安全校验
        if (VoxelSearch.FindNearestEmptyVoxel(volume, optimalPosition, characterHalfExtents) == VoxelMap.InvalidVoxel)
        {
            return position;
        }

        return optimalPosition;
    }

    /// <summary>
    /// 辅助函数：沿一个方向探测到第一个非空体素的距离。
    /// </summary>
    private float ProbeDistance(VoxelMap volume, Vector3 startPos, Vector3 direction, float maxDistance, Vector3 characterHalfExtents)
    {
        const float step = 1.5f; // 步进距离
        for (float d = step; d <= maxDistance; d += step)
        {
            var probePos = startPos + (direction * d);
            // 检查这个探测点是否能容纳角色（考虑了角色宽度）
            if (VoxelSearch.FindNearestEmptyVoxel(volume, probePos, characterHalfExtents) == VoxelMap.InvalidVoxel)
            {
                return d; // 撞墙了
            }
        }
        return maxDistance;
    }

    /// <summary>
    /// 检查从 fromPoint 到 toPoint 的直线路径是否对角色来说是安全的。
    /// 使用 EnumerateVoxelsInLine 作为核心。
    /// </summary>
    private bool IsPathSegmentSafe(Vector3 fromPoint, Vector3 toPoint, Vector3 characterHalfExtents, float maxSlopeAngleRadians, float requiredClearance = 5.0f)
    {
        // 步骤1: 将世界坐标转换为体素ID
        ulong fromVoxel = VoxelSearch.FindNearestEmptyVoxel(Volume, fromPoint, characterHalfExtents);
        ulong toVoxel = VoxelSearch.FindNearestEmptyVoxel(Volume, toPoint, characterHalfExtents);

        // 如果起点或终点在地图外或无效，则路径不安全
        if (fromVoxel == VoxelMap.InvalidVoxel || toVoxel == VoxelMap.InvalidVoxel)
        {
            return false;
        }

        var voxelsOnPath = VoxelSearch.EnumerateVoxelsInLinePooled(Volume, fromVoxel, toVoxel, fromPoint, toPoint);

        Vector3? previousVoxelCenter = null;
        var clearanceCheckHalfExtents = characterHalfExtents + new Vector3(requiredClearance);

        // 步骤3: 遍历路径上的每个体素并进行检查
        foreach (var (voxelId, t, isEmpty) in voxelsOnPath)
        {
            // 检查A: 体素本身是否是实体？
            // EnumerateVoxelsInLine 直接告诉我们体素是否为空，这是一个很好的快速失败路径。
            if (!isEmpty)
            {
                return false; // 路径直接穿过了一个实体方块
            }

            // 获取当前体素的中心点，用于更精确的检查
            // 同样，假设你有这个辅助函数
            Vector3 currentVoxelCenter = Volume.GetVoxelCenter(voxelId);

            // 检查B: 周围是否有足够的安全空间？
            // 这个检查比检查A更严格，因为它要求体素周围有额外的clearance。
            if (VoxelSearch.FindNearestEmptyVoxel(Volume, currentVoxelCenter, clearanceCheckHalfExtents) == VoxelMap.InvalidVoxel)
            {
                // 路径点周围空间不足
                return false;
            }

            // 检查C: 局部坡度是否过大？
            if (previousVoxelCenter.HasValue)
            {
                var step = currentVoxelCenter - previousVoxelCenter.Value;
                var horizontalDist = new Vector2(step.X, step.Z).Length();
                if (horizontalDist > 0.01f)
                {
                    var slopeAngle = MathF.Atan(Math.Abs(step.Y) / horizontalDist);
                    if (slopeAngle > maxSlopeAngleRadians)
                    {
                        return false; // 路径中有过于陡峭的台阶
                    }
                }
            }

            previousVoxelCenter = currentVoxelCenter;
        }

        return true;
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private float CalculateGScore(ref Node parent, ulong destVoxel, Vector3 destPos, ref int parentIndex)
    {
        // --- 1. 动态确定当前转换的层级 ---
        var parentLevel = GetLevelFromVoxel(parent.Voxel);
        var destLevel = GetLevelFromVoxel(destVoxel);

        // 使用两个节点中更精细的那个层级的规则。
        // (假设 L2_Fine=2, L1_Medium=1, L0_Coarse=0，所以取最大值)
        var transitionLevel = (VoxelLevel) Math.Max((int)parentLevel, (int)destLevel);

        // --- 2. 根据转换层级选择参数 ---
        float maxSlopeAngleDegrees;
        float steepSlopePenalty;

        switch (transitionLevel)
        {
            case VoxelLevel.L2_Fine:
                maxSlopeAngleDegrees = 75.0f;
                steepSlopePenalty = 6.0f;
                break;

            case VoxelLevel.L1_Medium:
                maxSlopeAngleDegrees = 80.0f;
                steepSlopePenalty = 3.0f;
                break;

            case VoxelLevel.L0_Coarse:
            default:
                maxSlopeAngleDegrees = 85.0f;
                steepSlopePenalty = 1.5f;
                break;
        }

        float maxSlopeAngleRadians = maxSlopeAngleDegrees * (float)(Math.PI / 180.0);

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
                slopePenaltyMultiplier = steepSlopePenalty;
            }
        }
        else
        {
            // 计算实际的移动坡度角度
            float slopeAngle = MathF.Atan(verticalDistance / horizontalDistance);

            // 如果坡度超过我们设定的阈值，则施加惩罚
            if (slopeAngle > maxSlopeAngleRadians)
            {
                // 从 1.0 (无惩罚) 到 steepSlopePenalty (最大惩罚) 的线性插值
                // 90度 (Math.PI / 2) 是最大坡度
                const float fullSteepAngle = (float)(Math.PI / 2.0);

                // 计算当前坡度在惩罚区间内的位置 (0 to 1)
                float t = (slopeAngle - maxSlopeAngleRadians) / (fullSteepAngle - maxSlopeAngleRadians);
                t = Math.Clamp(t, 0.0f, 1.0f); // 确保 t 在 [0, 1] 区间

                // 使用 t 进行线性插值
                slopePenaltyMultiplier = 1.0f + (t * (steepSlopePenalty - 1.0f));
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

    /// <summary>
    /// 根据 Voxel ID 的编码，确定其所属的最精细层级。
    /// </summary>
    /// <param name="voxel">要检查的 Voxel ID。</param>
    /// <returns>该 Voxel 所属的层级。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private VoxelLevel GetLevelFromVoxel(ulong voxel)
    {
        // 创建一个副本，因为 DecodeIndex 会修改它
        var tempVoxel = voxel;

        // 按照 L0 -> L1 -> L2 的顺序“剥离”索引
        // 我们不关心索引的值，只关心它的存在性
        _ = VoxelMap.DecodeIndex(ref tempVoxel); // 解码 L0
        var l1Index = VoxelMap.DecodeIndex(ref tempVoxel); // 解码 L1
        var l2Index = VoxelMap.DecodeIndex(ref tempVoxel); // 解码 L2

        // 从最精细的层级开始检查
        if (l2Index != VoxelMap.IndexLevelMask)
        {
            return VoxelLevel.L2_Fine;
        }

        if (l1Index != VoxelMap.IndexLevelMask)
        {
            return VoxelLevel.L1_Medium;
        }

        return VoxelLevel.L0_Coarse;
    }

    private static readonly Random Rnd = new();
    public static void GenerateRandomThisTime()
    {
        CurRandomFactor = Service.Config.VoxelPathfindRandomFactor > 0f ? (float)Rnd.NextDouble() * Service.Config.VoxelPathfindRandomFactor : 0;
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

    public enum VoxelLevel
    {
        L2_Fine = 2,
        L1_Medium = 1,
        L0_Coarse = 0
    }
}
