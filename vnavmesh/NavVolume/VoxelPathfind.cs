using FFXIVClientStructs.FFXIV.Client.Game.Control;
using FFXIVClientStructs.FFXIV.Client.System.Framework;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision;
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

    private static Vector3 CurStartPos { get; set; } = new();
    
    private const bool  AllowReopen    = false; // this is extremely expensive and doesn't seem to actually improve the result
    private const float RaycastLimitSq = float.MaxValue;

    public VoxelMap Volume { get; } = volume;

    public Span<Node> NodeSpan =>
        CollectionsMarshal.AsSpan(Nodes);

    private readonly Vector3 CharaHalfExtents = new(2f, 2f, 2f);

    public List<(ulong voxel, Vector3 p)> FindPath(
        ulong fromVoxel, ulong toVoxel, Vector3 fromPos, Vector3 toPos, bool useRaycast, bool returnIntermediatePoints, 
        Action<float>? progressCallback, CancellationToken cancel)
    {
        CurStartPos = fromPos;

        UseRaycast = useRaycast;
        GenerateRandomThisTime();
        Start(fromVoxel, toVoxel, fromPos, toPos);
        Execute(cancel, progressCallback, CalculateDynamicMaxSteps(fromPos, toPos));

        // --- 修改开始 ---
        // 1. 先构建原始路径
        var rawPath = BuildPathToVisitedNode(BestNodeIndex, returnIntermediatePoints);

        Plugin.OriginalPath_Locked = true;
        Plugin.OriginalPath = [.. rawPath.Select(p => p.p)];
        Plugin.OriginalPath_Locked = false;

        if (Service.Config.VoxelPathfindPostProcess)
        {
            // 2. 对原始路径进行后处理以调整垂直净空
            var adjustedPathPoints = AdjustPathForClearance(rawPath, CharaHalfExtents);

            // 3. 将调整后的坐标点包装成期望的返回类型
            // voxel 信息被丢弃，因为点的位置已经改变，原始voxel不再精确
            var finalPath = adjustedPathPoints.Select(p => (voxel: (ulong)0, p)).ToList();

            return finalPath;
        } else
        {
            rawPath.Add((toVoxel, toPos)); // 确保终点被包含在路径中
        }
        // --- 修改结束 ---
        return rawPath;
    }

    public unsafe static delegate* unmanaged<BGCollisionModule*, RaycastHit*, Vector4*, Vector3*, float, int, int*, bool> _SweepSphereMaterialFilter => (delegate* unmanaged<BGCollisionModule*, RaycastHit*, Vector4*, Vector3*, float, int, int*, bool>)BGCollisionModule.Addresses.SweepSphereMaterialFilter.Value;

    public unsafe static bool SweepSphereMaterialFilter(Vector4 origin, Vector3 direction, out RaycastHit hitInfo, float maxDistance = 1000000f)
    {
        var framework = Framework.Instance();
        if (framework == null)
        {
            hitInfo = default;
            return false;
        }
        var bgCollisionModule = framework->BGCollisionModule;
        if (bgCollisionModule == null)
        {
            hitInfo = default;
            return false;
        }
        var flags = stackalloc int[] { 0x4000, 0, 0x4000, 0 };
        var hit = new RaycastHit();
        var result = _SweepSphereMaterialFilter(bgCollisionModule, &hit, &origin, &direction, maxDistance, 1, flags);
        hitInfo = hit;
        return result;
    }

    public List<Vector3> AdjustPathForClearance(List<(ulong voxel, Vector3 p)> rawPath, Vector3 characterHalfExtents)
    {
        if (rawPath.Count <= 2)
            return [.. rawPath.Select(node => node.p)];

        const float ProbeStepSize = 3f; // 在原始路径上放置探针的密度
        const float MaxAngleRadians = 65.0f * (float)(Math.PI / 180.0);

        // ==================================================================
        // ---- PASS 1: 生成高精度中心路径 (Generate Centered High-Res Path) ----
        // 这是最关键的新增步骤
        // ==================================================================
        var centeredHighResPath = new List<Vector3>
        {
            rawPath[0].p
        };

        for (int i = 0; i < rawPath.Count - 1; i++)
        {
            var startNode = rawPath[i];
            var endNode = rawPath[i + 1];
            var segment = endNode.p - startNode.p;
            float distance = segment.Length();

            if (distance < 0.1f) continue;

            var segmentDirection = Vector3.Normalize(segment);

            // 在原始路径段上插值生成探针点
            for (float traveled = ProbeStepSize; traveled < distance; traveled += ProbeStepSize)
            {
                var probePoint = startNode.p + (segment * (traveled / distance));
                // 对每个探针点进行置中，并使用前一个已置中的点作为参考
                var centeredProbe = ProcessPoint(probePoint, centeredHighResPath[^1], segmentDirection, MaxAngleRadians, characterHalfExtents);
                centeredHighResPath.Add(centeredProbe);
            }

            // 处理段的终点
            var lastCenteredPoint = ProcessPoint(endNode.p, centeredHighResPath[^1], segmentDirection, MaxAngleRadians, characterHalfExtents);
            centeredHighResPath.Add(lastCenteredPoint);
        }

        // ==================================================================
        // ---- PASS 2: 简化中心路径 (String Pulling on Centered Path) ----
        // ==================================================================
        if (centeredHighResPath.Count < 2)
        {
            return [rawPath[0].p, rawPath[^1].p];
        }

        centeredHighResPath[0] = rawPath[0].p;
        centeredHighResPath[^1] = rawPath[^1].p;

        var keyWaypoints = new List<Vector3>
        {
            centeredHighResPath[0] // 添加起点
        };
        int currentIndex = 0;
        while (currentIndex < centeredHighResPath.Count - 1)
        {
            int furthestVisibleIndex = currentIndex + 1;
            for (int j = currentIndex + 2; j < centeredHighResPath.Count; j++)
            {
                // 在已经置中过的路径上进行安全检查
                if (IsPathSegmentSafe(keyWaypoints[^1], centeredHighResPath[j], characterHalfExtents, MaxAngleRadians))
                {
                    furthestVisibleIndex = j;
                }
                else
                {
                    break;
                }
            }
            keyWaypoints.Add(centeredHighResPath[furthestVisibleIndex]);
            currentIndex = furthestVisibleIndex;
        }

        // 确保终点被添加
        if (keyWaypoints[^1] != rawPath[^1].p)
        {
            // 检查最后一个关键路点和最终目标点之间是否安全
            if (IsPathSegmentSafe(keyWaypoints[^1], rawPath[^1].p, characterHalfExtents, MaxAngleRadians))
            {
                keyWaypoints.Add(rawPath[^1].p);
            }
        }

        // 路径平滑
        var smoothedWaypoints = SmoothPathChaikin(keyWaypoints, 2, .1f);

        return smoothedWaypoints;
    }

    public static List<Vector3> SmoothPathChaikin(List<Vector3> path, int iterations = 1, float smoothingFactor = 0.2f)
    {
        if (path.Count < 3 || iterations < 1)
        {
            return path;
        }

        // 确保因子在 (0, 0.5) 范围内，通常用 0.25
        float p1Factor = Math.Clamp(smoothingFactor, 0.01f, 0.49f);
        float p2Factor = 1.0f - p1Factor;

        for (int i = 0; i < iterations; i++)
        {
            var smoothedPath = new List<Vector3> { path[0] }; // 保留原始起点

            for (int j = 0; j < path.Count - 1; j++)
            {
                Vector3 p0 = path[j];
                Vector3 p1 = path[j + 1];

                // 在原线段的 25% 和 75% 处创建新点
                Vector3 newPoint1 = Vector3.Lerp(p0, p1, p1Factor);
                Vector3 newPoint2 = Vector3.Lerp(p0, p1, p2Factor);

                smoothedPath.Add(newPoint1);
                smoothedPath.Add(newPoint2);
            }

            smoothedPath.Add(path[^1]); // 保留原始终点
            path = smoothedPath; // 将平滑后的路径作为下一次迭代的输入
        }

        return path;
    }

    /// <summary>
    /// 处理单个路径点：进行垂直调整、角度限制和最终安全检查。
    /// </summary>
    private Vector3 ProcessPoint(Vector3 currentPoint, Vector3 previousPoint, Vector3 pathDirection, float maxAngleRadians, Vector3 characterHalfExtents)
    {
        // 坡度限制
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

        // --- 2. 找到水平中心 ---
        // 使用经过坡度修正的点作为输入
        Vector3 horizontallyCenteredPos = FindChannelCenter(currentPoint, pathDirection, characterHalfExtents);

        // --- 3. 受限的垂直搜索 (核心修正) ---
        float sweepRadius = Math.Max(characterHalfExtents.X, characterHalfExtents.Z);
        // 定义一个严格的垂直搜索范围，这是防止“掉下悬崖”的关键
        const float verticalSearchRange = 3.0f;

        // 从我们坡度安全的Y坐标稍高一点的位置开始向下搜索
        Vector3 downProbeOrigin = new Vector3(
            horizontallyCenteredPos.X,
            currentPoint.Y + characterHalfExtents.Y, // 使用坡度修正后的Y值！
            horizontallyCenteredPos.Z
        );

        bool usePhysics = IsInPhysicsRange(downProbeOrigin);
        bool groundFound = false;
        Vector3 groundHitPoint = Vector3.Zero;

        if (usePhysics)
        {
            // --- 原始逻辑: 使用精确的物理扫描 ---
            Vector4 sweepOrigin = new Vector4(downProbeOrigin, sweepRadius);
            if (SweepSphereMaterialFilter(sweepOrigin, -Vector3.UnitY, out RaycastHit groundHit, verticalSearchRange))
            {
                groundFound = true;
                groundHitPoint = groundHit.Point;
            }
        }
        else
        {
            // --- 新逻辑: 使用体素扫描 ---
            if (VoxelSweepSphere(Volume, downProbeOrigin, -Vector3.UnitY, sweepRadius, verticalSearchRange, out Vector3 voxelHitPoint))
            {
                groundFound = true;
                // 对于向下的扫描，我们关心的是Y坐标。体素扫描返回的球心Y值可以近似作为地面位置。
                groundHitPoint = voxelHitPoint;
            }
        }

        // --- 根据探测结果决定最终位置 (逻辑不变) ---
        if (groundFound)
        {
            return new Vector3(horizontallyCenteredPos.X, groundHitPoint.Y + characterHalfExtents.Y, horizontallyCenteredPos.Z);
        }
        else
        {
            // Fallback: 附近没有找到地面
            return horizontallyCenteredPos;
        }
    }

    private static readonly Vector3[] HorizontalProbeDirs = [
        new Vector3(1, 0, 0), new Vector3(-1, 0, 0),
        new Vector3(0, 0, 1), new Vector3(0, 0, -1),
        Vector3.Normalize(new Vector3(1, 0, 1)), Vector3.Normalize(new Vector3(1, 0, -1)),
        Vector3.Normalize(new Vector3(-1, 0, 1)), Vector3.Normalize(new Vector3(-1, 0, -1))
    ];

    private Vector3 FindChannelCenter(Vector3 position, Vector3 pathDirection, Vector3 characterHalfExtents, float maxProbeDistance = 10f)
    {
        bool usePhysics = IsInPhysicsRange(position);

        // 1. 计算与路径方向垂直的“右”方向 (只考虑水平面)
        Vector3 rightDir = Vector3.Normalize(new Vector3(pathDirection.Z, 0, -pathDirection.X));
        Vector3 leftDir = -rightDir;

        float sweepRadius = Math.Max(characterHalfExtents.X, characterHalfExtents.Z);
        Vector3 rightHitPoint = position + rightDir * maxProbeDistance; // 默认值
        Vector3 leftHitPoint = position + leftDir * maxProbeDistance;  // 默认值
        bool rightHit = false;
        bool leftHit = false;

        // 2.向右探测
        if (usePhysics)
        {
            // --- 原始逻辑: 使用精确的物理扫描 ---
            Vector4 sweepOrigin = new Vector4(position, sweepRadius);
            if (SweepSphereMaterialFilter(sweepOrigin, rightDir, out RaycastHit hitInfoRight, maxProbeDistance))
            {
                rightHitPoint = hitInfoRight.Point;
                rightHit = true;
            }
        }
        else
        {
            // --- 新逻辑: 使用体素扫描 ---
            if (VoxelSweepSphere(Volume, position, rightDir, sweepRadius, maxProbeDistance, out Vector3 voxelHitPoint))
            {
                // 注意：体素扫描返回的是球体中心的位置，这对于计算已经足够精确
                rightHitPoint = voxelHitPoint;
                rightHit = true;
            }
        }

        // 3. 向左探测
        if (usePhysics)
        {
            // --- 原始逻辑: 使用精确的物理扫描 ---
            Vector4 sweepOrigin = new Vector4(position, sweepRadius);
            if (SweepSphereMaterialFilter(sweepOrigin, leftDir, out RaycastHit hitInfoLeft, maxProbeDistance))
            {
                leftHitPoint = hitInfoLeft.Point;
                leftHit = true;
            }
        }
        else
        {
            // --- 新逻辑: 使用体素扫描 ---
            if (VoxelSweepSphere(Volume, position, leftDir, sweepRadius, maxProbeDistance, out Vector3 voxelHitPoint))
            {
                leftHitPoint = voxelHitPoint;
                leftHit = true;
            }
        }

        Vector3 centeredPosition;

        // 4. 根据探测结果计算中心点
        if (rightHit && leftHit)
        {
            // 两边都碰到墙，完美情况，取中点
            centeredPosition = (rightHitPoint + leftHitPoint) * 0.5f;
        }
        else if (rightHit)
        {
            // 只碰到右边的墙，从右墙向左推一个角色宽度
            centeredPosition = rightHitPoint + leftDir * characterHalfExtents.X; // 假设X是宽度
        }
        else if (leftHit)
        {
            // 只碰到左边的墙，从左墙向右推一个角色宽度
            centeredPosition = leftHitPoint + rightDir * characterHalfExtents.X;
        }
        else
        {
            // 两边都是开阔地，保持原位
            centeredPosition = position;
        }

        // 关键：只修改水平位置，保留原始的Y坐标
        centeredPosition.Y = position.Y;
        return centeredPosition;
    }

    private const float PHYSICS_RELIABLE_DISTANCE = 100.0f;

    /// <summary>
    /// 检查从 fromPoint 到 toPoint 的直线路径是否对角色来说是安全的。
    /// 使用 EnumerateVoxelsInLine 作为核心。
    /// </summary>
    private bool IsPathSegmentSafe(Vector3 fromPoint, Vector3 toPoint, Vector3 characterHalfExtents, float maxSlopeAngleRadians, float requiredClearance = 5.0f)
    {
        var segment = toPoint - fromPoint;
        float distance = segment.Length();

        // 如果距离太近，可以认为路径是安全的，避免除零等问题
        if (distance < 0.01f)
        {
            return true;
        }

        // --- 1. 检查整体坡度 (依然重要) ---
        // 物理扫描无法判断坡度是否适合角色行走，所以这个检查必须保留。
        var horizontalDist = new Vector2(segment.X, segment.Z).Length();
        if (horizontalDist > 0.01f)
        {
            var slopeAngle = MathF.Atan(Math.Abs(segment.Y) / horizontalDist);
            if (slopeAngle > maxSlopeAngleRadians)
            {
                // 整个路径段对于角色来说太陡峭了
                return false;
            }
        }

        // --- 2. 检查物理遮挡 (核心) ---
        // 使用 SweepSphereMaterialFilter 来检查角色体积是否会与障碍物碰撞。
        if (distance <= PHYSICS_RELIABLE_DISTANCE)
        {
            // --- 近距离：使用精确的物理扫描 ---
            float sweepRadius = Math.Max(characterHalfExtents.X, Math.Max(characterHalfExtents.Y, characterHalfExtents.Z));
            Vector4 sweepOrigin = new Vector4(fromPoint, sweepRadius);
            Vector3 direction = segment / distance;
            bool hitSomething = SweepSphereMaterialFilter(sweepOrigin, direction, out _, distance);
            return !hitSomething;
        }
        else
        {
            // --- 远距离：回退到我们新创建的、带体积的体素检查 ---
            return IsPathSegmentSafeWithVoxelVolume(Volume, fromPoint, toPoint, characterHalfExtents);
        }
    }

    /// <summary>
    /// 检查以 centerVoxelId 为中心的、指定半径范围内的所有体素是否都为空。
    /// 这个版本专为您的 VoxelMap API 设计。
    /// </summary>
    /// <param name="volume">体素地图</param>
    /// <param name="centerVoxelId">要检查的中心体素ID</param>
    /// <param name="radiusInVoxels">检查半径（以最精细的体素为单位）</param>
    /// <returns>如果区域内所有体素都为空，则返回 true</returns>
    private bool IsVoxelNeighborhoodEmpty(VoxelMap volume, ulong centerVoxelId, int radiusInVoxels)
    {
        // 如果半径为0或更小，我们不需要检查邻居，只需检查中心点。
        // 但调用此函数的前提是中心点已经检查过了，所以可以直接返回true。
        if (radiusInVoxels <= 0)
        {
            return true;
        }

        // 1. 获取最精细层级（L2）的体素大小。这是我们计算偏移量的单位。
        //    假设 Levels[2] 是最精细的层级。
        float leafVoxelSize = volume.Levels[2].CellSize.X; // 假设体素是立方体

        // 2. 获取中心体素的世界坐标包围盒，以此作为计算的“锚点”。
        var (vmin, vmax) = volume.VoxelBounds(centerVoxelId, 0);
        var centerWorldPos = (vmin + vmax) * 0.5f;

        // 3. 在体素空间中，遍历以中心点为原点的、边长为 (2*radius+1) 的立方体。
        for (int dz = -radiusInVoxels; dz <= radiusInVoxels; dz++)
        {
            for (int dy = -radiusInVoxels; dy <= radiusInVoxels; dy++)
            {
                for (int dx = -radiusInVoxels; dx <= radiusInVoxels; dx++)
                {
                    // 跳过中心点本身，因为它已经在主循环中检查过了
                    if (dx == 0 && dy == 0 && dz == 0)
                    {
                        continue;
                    }

                    // 4. 计算邻居体素中心点的近似世界坐标。
                    var offset = new Vector3(dx, dy, dz) * leafVoxelSize;
                    var neighborWorldPos = centerWorldPos + offset;

                    // 5. 使用 FindLeafVoxel 检查这个世界坐标位置的体素状态。
                    //    这是最可靠的方法，因为它利用了您系统内建的世界到体素的转换逻辑。
                    var (_, neighborIsEmpty) = volume.FindLeafVoxel(neighborWorldPos);

                    if (!neighborIsEmpty)
                    {
                        // 发现障碍物，路径不安全
                        return false;
                    }
                }
            }
        }

        // 遍历完所有邻居都没发现障碍物
        return true;
    }

    /// <summary>
    /// 使用体素数据检查一个带体积的路径段是否安全（为八叉树优化）。
    /// </summary>
    private bool IsPathSegmentSafeWithVoxelVolume(VoxelMap volume, Vector3 fromPos, Vector3 toPos, Vector3 characterHalfExtents)
    {
        // 1. 计算检查半径（以最精细的体素为单位）
        float characterRadius = Math.Max(characterHalfExtents.X, Math.Max(characterHalfExtents.Y, characterHalfExtents.Z));
        float leafVoxelSize = volume.Levels[2].CellSize.X; // 假设体素是立方体
        int radiusInVoxels = (int)Math.Ceiling(characterRadius / leafVoxelSize);

        // 2. 获取路径中心线上的所有体素
        ulong fromVoxel = VoxelSearch.FindNearestEmptyVoxel(volume, fromPos, characterHalfExtents);
        ulong toVoxel = VoxelSearch.FindNearestEmptyVoxel(volume, toPos, characterHalfExtents);

        List<(ulong voxel, float t, bool empty)> voxelsOnLine;
        try
        {
            voxelsOnLine = VoxelSearch.EnumerateVoxelsInLinePooled(volume, fromVoxel, toVoxel, fromPos, toPos);
        }
        catch (PathfindLoopException ex)
        {
            Service.Log.Debug($"远距离体素直线遍历失败: {ex.Message}");
            return false; // 直线遍历失败，视为不安全
        }

        // 3. 遍历中心线上的每个体素，并使用我们新的辅助函数检查其体积
        foreach (var (centerVoxelId, t, centerIsEmpty) in voxelsOnLine)
        {
            // 快速失败路径：如果中心线直接撞上非空体素，立即返回false
            if (!centerIsEmpty)
            {
                return false;
            }

            // 对中心体素进行邻域体积检查
            if (!IsVoxelNeighborhoodEmpty(volume, centerVoxelId, radiusInVoxels))
            {
                // 体积检查失败，意味着角色的边缘会碰到障碍物
                return false;
            }
        }

        // 所有检查都通过了，路径是安全的
        return true;
    }

    /// <summary>
    /// 模拟 SweepSphere 的体素版本。沿着一个方向步进，检查每一步的体积。
    /// </summary>
    /// <param name="volume">体素地图</param>
    /// <param name="origin">扫描的起始世界坐标</param>
    /// <param name="direction">扫描方向（应为单位向量）</param>
    /// <param name="radius">扫描球体的半径</param>
    /// <param name="maxDistance">最大扫描距离</param>
    /// <param name="hitPoint">如果碰到障碍物，返回碰撞发生时球体中心的位置</param>
    /// <returns>如果碰到非空体素，则为 true</returns>
    private bool VoxelSweepSphere(VoxelMap volume, Vector3 origin, Vector3 direction, float radius, float maxDistance, out Vector3 hitPoint)
    {
        float leafVoxelSize = volume.Levels[2].CellSize.X;
        int radiusInVoxels = (int)Math.Ceiling(radius / leafVoxelSize);

        // 我们以体素大小的一半作为步长，以确保不会跳过薄墙
        float stepSize = leafVoxelSize * 0.5f;
        int numSteps = (int)(maxDistance / stepSize);

        for (int i = 1; i <= numSteps; i++) // 从第一步开始，避免立即检测起点
        {
            Vector3 currentCenter = origin + direction * (i * stepSize);

            // 1. 检查球体中心点所在的体素
            var (centerVoxelId, centerIsEmpty) = volume.FindLeafVoxel(currentCenter);
            if (!centerIsEmpty)
            {
                hitPoint = currentCenter;
                return true; // 中心直接命中
            }

            // 2. 检查该点周围的体积
            if (!IsVoxelNeighborhoodEmpty(volume, centerVoxelId, radiusInVoxels))
            {
                hitPoint = currentCenter; // 体积碰撞
                return true;
            }
        }

        hitPoint = Vector3.Zero;
        return false;
    }

    private bool IsInPhysicsRange(Vector3 point)
    {
        return Vector3.Distance(point, CurStartPos) <= PHYSICS_RELIABLE_DISTANCE;
    }

    /// <summary>
    /// 一个辅助结构体，用于非破坏性地解码一个 Voxel ulong ID。
    /// </summary>
    public readonly struct VoxelIDDecoder
    {
        public readonly ulong L0Index;
        public readonly ulong L1Index;
        public readonly ulong L2Index;

        public VoxelIDDecoder(ulong voxelId)
        {
            // 复制 voxelId 以免破坏原始值
            var tempVoxel = voxelId;
            // 解码顺序根据 EnumerateNeighbours 的实现推断
            L2Index = VoxelMap.DecodeIndex(ref tempVoxel);
            L1Index = VoxelMap.DecodeIndex(ref tempVoxel);
            L0Index = VoxelMap.DecodeIndex(ref tempVoxel);
        }
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
                maxSlopeAngleDegrees = 65.0f;
                steepSlopePenalty = 5.0f;
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
        finalMoveCost *= GetRandomCostMultiplier();

        return parentBaseG + finalMoveCost;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private float HeuristicDistance(ulong nodeVoxel, Vector3 v)
    {
        if (nodeVoxel == GoalVoxel)
            return 0;

        var baseHeuristic = (v - GoalPos).Length();
        float minCostMultiplier = 1.0f - CurRandomFactor;

        return baseHeuristic * minCostMultiplier;
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

    private const float MAX_RANDOM_DEVIATION = 0.1f; // ±10%
    private static readonly Random Rnd = new();
    public static void GenerateRandomThisTime()
    {
        float userStrengthFactor = Service.Config.VoxelPathfindRandomFactor;

        CurRandomFactor = MAX_RANDOM_DEVIATION * userStrengthFactor;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private float GetRandomCostMultiplier()
    {
        if (CurRandomFactor <= 0f)
        {
            return 1.0f;
        }

        float randomSample = ((float)Rnd.NextDouble() * 2.0f) - 1.0f;

        // 使用为本次寻路计算好的最大偏移量 CurRandomFactor 来计算最终乘数
        // 范围是 [1.0 - CurRandomFactor, 1.0 + CurRandomFactor]
        return 1.0f + (randomSample * CurRandomFactor);
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
