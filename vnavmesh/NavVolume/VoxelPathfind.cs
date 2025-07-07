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

    public List<Vector3> AdjustPathForClearance(List<(ulong voxel, Vector3 p)> rawPath, Vector3 characterHalfExtents)
    {
        if (rawPath.Count < 2)
            return [.. rawPath.Select(node => node.p)];

        // --- 参数定义 (保持不变) ---
        const float MaxAngleDegrees = 65.0f;
        const float MaxAngleRadians = MaxAngleDegrees * (float)(Math.PI / 180.0);
        const float MaxClimbAngDeg = 45.0f;
        const float MaxClimbAngRad = MaxClimbAngDeg * (float)(Math.PI / 180.0);
        const float InterpolationStepSize = 8f;

        // 新增的智能调整参数
        const float SmoothnessFactor = 0.6f; // 平滑力的强度 (0 to 1)
        const float CenteringFactor = 0.8f; // 居中力的强度 (0 to 1)

        // ==================================================================
        // ---- PASS 1: 路径简化 (String Pulling) ----
        // 从原始路径中提取出最关键的拐点，不进行任何位置调整。
        // ==================================================================
        var keyWaypoints = new List<Vector3>();
        keyWaypoints.Add(rawPath[0].p); // 添加起点

        int currentIndex = 0;
        while (currentIndex < rawPath.Count - 1)
        {
            int furthestVisibleIndex = currentIndex + 1;
            for (int j = currentIndex + 2; j < rawPath.Count; j++)
            {
                // 使用您现有的安全检查，从当前拐点看向上一个成功连接的点
                if (IsPathSegmentSafe(keyWaypoints[^1], rawPath[j].p, characterHalfExtents, MaxAngleRadians))
                {
                    furthestVisibleIndex = j;
                }
                else
                {
                    break;
                }
            }
            keyWaypoints.Add(rawPath[furthestVisibleIndex].p);
            currentIndex = furthestVisibleIndex;
        }

        // 如果最后一个点没有被包含，确保添加它
        if (keyWaypoints[^1] != rawPath[^1].p)
        {
            keyWaypoints.Add(rawPath[^1].p);
        }

        // ==================================================================
        // ---- PASS 2: 智能调整关键拐点 (Smart Adjustment) ----
        // 对简化的拐点列表进行处理，融合平滑和居中。
        // ==================================================================
        if (keyWaypoints.Count < 3)
        {
            // 路径太短，无法进行平滑调整，直接进入最终生成阶段
            return GenerateFinalPath(keyWaypoints, InterpolationStepSize);
        }

        var adjustedWaypoints = new List<Vector3>(new Vector3[keyWaypoints.Count]);
        adjustedWaypoints[0] = keyWaypoints[0]; // 起点不变
        adjustedWaypoints[^1] = keyWaypoints[^1]; // 终点暂时不变，最后特殊处理

        // --- 特殊处理第一个拐点 (P1) ---
        // 这里可以插入您原来对 P1 的前瞻性修正逻辑，以确保平稳起步
        // (为简化，此处使用标准调整，但您可以将原逻辑放在这里)
        adjustedWaypoints[1] = ProcessPoint(keyWaypoints[1], adjustedWaypoints[0], MaxClimbAngRad, characterHalfExtents);

        // --- 循环调整中间的拐点 ---
        for (int i = 2; i < keyWaypoints.Count - 1; i++)
        {
            var prevAdjusted = adjustedWaypoints[i - 1];
            var currentOriginal = keyWaypoints[i];
            var nextOriginal = keyWaypoints[i + 1];

            // 1. 计算“平滑”目标点 (几何中心)
            var smoothTarget = prevAdjusted + (nextOriginal - prevAdjusted) * 0.5f;

            // 2. 计算“居中”目标点 (使用您强大的ProcessPoint)
            var centerTarget = ProcessPoint(currentOriginal, prevAdjusted, MaxAngleRadians, characterHalfExtents);

            // 3. 融合两个目标
            // 从原始点开始，向“平滑”和“居中”的混合目标移动
            var blendedTarget = Vector3.Lerp(smoothTarget, centerTarget, CenteringFactor);
            var candidatePoint = Vector3.Lerp(currentOriginal, blendedTarget, SmoothnessFactor);

            // 4. 安全校验
            if (IsPathSegmentSafe(prevAdjusted, candidatePoint, characterHalfExtents, MaxAngleRadians))
            {
                adjustedWaypoints[i] = candidatePoint;
            }
            else // 如果融合点不安全，退而求其次使用更可靠的居中点
            {
                if (IsPathSegmentSafe(prevAdjusted, centerTarget, characterHalfExtents, MaxAngleRadians))
                {
                    adjustedWaypoints[i] = centerTarget;
                }
                else // 如果两者都失败，保留原始拐点
                {
                    adjustedWaypoints[i] = currentOriginal;
                }
            }
        }

        // ==================================================================
        // ---- PASS 3: 生成高分辨率最终路径 (Final Generation) ----
        // 在调整好的、平滑的拐点之间进行插值。
        // ==================================================================
        return GenerateFinalPath(adjustedWaypoints, InterpolationStepSize);
    }

    private List<Vector3> GenerateFinalPath(List<Vector3> adjustedWaypoints, float stepSize)
    {
        var finalPath = new List<Vector3>();
        if (adjustedWaypoints.Count < 2) return adjustedWaypoints;

        finalPath.Add(adjustedWaypoints[0]);

        for (int i = 0; i < adjustedWaypoints.Count - 1; i++)
        {
            Vector3 startPoint = adjustedWaypoints[i];
            Vector3 endPoint = adjustedWaypoints[i + 1];

            var segmentVector = endPoint - startPoint;
            float segmentLength = segmentVector.Length();
            if (segmentLength < 0.1f) continue;

            int numSteps = (int)(segmentLength / stepSize);
            if (numSteps == 0) numSteps = 1;

            for (int j = 1; j <= numSteps; j++)
            {
                float t = (float)j / numSteps;
                finalPath.Add(Vector3.Lerp(startPoint, endPoint, t));
            }
        }
        return finalPath;
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

    private static readonly Vector3[] HorizontalProbeDirs = new Vector3[]
    {
        new Vector3(1, 0, 0), new Vector3(-1, 0, 0),
        new Vector3(0, 0, 1), new Vector3(0, 0, -1),
        Vector3.Normalize(new Vector3(1, 0, 1)), Vector3.Normalize(new Vector3(1, 0, -1)),
        Vector3.Normalize(new Vector3(-1, 0, 1)), Vector3.Normalize(new Vector3(-1, 0, -1))
    };

    private Vector3 Find3DCenter(VoxelMap volume, Vector3 position, Vector3 characterHalfExtents, float maxProbeDistance = 5f, float verticalBias = 0.5f)
    {
        // --- 步骤 1: 计算水平中心 (XZ平面质心) ---
        var boundaryPoints = new List<Vector3>();
        Vector3 horizontalCentroid = new Vector3(position.X, 0, position.Z); // 初始值

        foreach (var dir in HorizontalProbeDirs)
        {
            float dist = ProbeDistance(volume, position, dir, maxProbeDistance, characterHalfExtents);
            boundaryPoints.Add(position + dir * dist);
        }

        if (boundaryPoints.Count > 0)
        {
            float totalX = 0, totalZ = 0;
            foreach (var p in boundaryPoints)
            {
                totalX += p.X;
                totalZ += p.Z;
            }
            horizontalCentroid = new Vector3(totalX / boundaryPoints.Count, 0, totalZ / boundaryPoints.Count);
        }

        // --- 步骤 2: 可靠地计算垂直空间 (Y轴) ---

        // 使用新的、可靠的方法找到脚下的地面
        float groundY = FindGroundY(volume, position.X, position.Z, position.Y);

        // 向上探测天花板 (这个逻辑通常是可靠的，可以保留)
        float distY_pos = ProbeDistance(volume, position, Vector3.UnitY, maxProbeDistance, characterHalfExtents);
        float ceilingY = position.Y + distY_pos;

        // 如果没有找到地面，这是一个异常情况，我们必须保守处理
        if (groundY == float.MinValue)
        {
            // 无法确定地面，最安全的做法是保持原始Y坐标不变
            return new Vector3(horizontalCentroid.X, position.Y, horizontalCentroid.Z);
        }

        // 计算目标Y值
        float targetY;
        float verticalSpace = ceilingY - groundY;
        float characterHeight = characterHalfExtents.Y * 2;

        if (verticalSpace > characterHeight * 1.1f) // 确保有足够空间
        {
            // 将角色的脚底板放在地面上，然后根据bias在剩余空间里上浮
            float availableSpace = verticalSpace - characterHeight;
            targetY = groundY + characterHalfExtents.Y + (availableSpace * Math.Clamp(verticalBias, 0.0f, 1.0f));
        }
        else
        {
            // 空间狭小，取正中
            targetY = groundY + verticalSpace / 2.0f;
        }

        // --- 步骤 3: 组合成最佳猜测点 ---
        Vector3 optimalPosition = new Vector3(horizontalCentroid.X, targetY, horizontalCentroid.Z);

        // --- 步骤 4: 最终安全校验和“地平线”安全钳 ---
        // 即使所有计算都正确，我们也要做最后一道保险

        // 重新在最终的水平位置下方查找地面
        float finalGroundY = FindGroundY(volume, optimalPosition.X, optimalPosition.Z, optimalPosition.Y + characterHalfExtents.Y);

        if (finalGroundY != float.MinValue)
        {
            // 【安全钳】确保角色的脚底板不会低于最终找到的地面
            float minimumAllowedY = finalGroundY + characterHalfExtents.Y;
            if (optimalPosition.Y < minimumAllowedY)
            {
                optimalPosition.Y = minimumAllowedY;
            }
        }

        // 检查最终位置是否在实体内（作为最后的防线）
        if (VoxelSearch.FindNearestEmptyVoxel(volume, optimalPosition, characterHalfExtents) == VoxelMap.InvalidVoxel)
        {
            // 如果还是失败了，返回原始位置是最安全的选择
            return position;
        }

        return optimalPosition;
    }

    /// <summary>
    /// 辅助函数：沿一个方向探测到第一个非空体素的距离。
    /// </summary>
    private float ProbeDistance(VoxelMap volume, Vector3 startPos, Vector3 direction, float maxDistance, Vector3 characterHalfExtents)
    {
        // 使用一个比角色尺寸稍小的步长，以确保不会跳过薄墙
        float step = Math.Min(characterHalfExtents.X, characterHalfExtents.Z) * 0.9f;
        if (step < 0.1f) step = 0.1f;

        Vector3 previousProbePos = startPos;

        for (float d = step; d <= maxDistance; d += step)
        {
            var currentProbePos = startPos + (direction * d);

            // 检查从上一个探测点到当前探测点的“厚”线段是否安全
            if (!IsThickSegmentSafe(previousProbePos, currentProbePos, characterHalfExtents))
            {
                // 在 (d - step) 和 d 之间发生了碰撞。
                // 为简单起见，我们返回上一个安全点的距离。
                // 一个更精确的方法是在这个小段内进行二分搜索，但通常没有必要。
                return d - step;
            }

            previousProbePos = currentProbePos;
        }

        return maxDistance; // 整个探测距离内都没有障碍物
    }

    /// <summary>
    /// 【新辅助函数】检查一个短的直线段对于角色体积是否安全。
    /// 它通过在角色横截面的关键点上投射平行的 LineOfSight 射线来实现。
    /// </summary>
    /// <returns>如果整个体积都能安全通过，则返回 true。</returns>
    private bool IsThickSegmentSafe(Vector3 segmentStart, Vector3 segmentEnd, Vector3 characterHalfExtents)
    {
        var direction = segmentEnd - segmentStart;
        if (direction.LengthSquared() < 0.01f) return true; // Segment is too short to check

        // 计算与移动方向垂直的 "right" 和 "up" 向量，以定义角色的横截面
        Vector3 right, up;
        if (Math.Abs(Vector3.Dot(Vector3.Normalize(direction), Vector3.UnitY)) > 0.99f)
        {
            // 如果移动方向几乎是垂直的，使用 Forward 向量作为参考
            right = Vector3.Normalize(Vector3.Cross(direction, Vector3.UnitZ));
            up = Vector3.Normalize(Vector3.Cross(right, direction));
        }
        else
        {
            // 标准情况
            right = Vector3.Normalize(Vector3.Cross(direction, Vector3.UnitY));
            up = Vector3.Normalize(Vector3.Cross(right, direction));
        }

        // 定义角色横截面的5个关键检查点偏移
        var offsets = new Vector3[]
        {
        Vector3.Zero, // Center
        (right * characterHalfExtents.X) + (up * characterHalfExtents.Y),   // Top-Right
        (right * characterHalfExtents.X) - (up * characterHalfExtents.Y),   // Bottom-Right
        (-right * characterHalfExtents.X) - (up * characterHalfExtents.Y),  // Bottom-Left
        (-right * characterHalfExtents.X) + (up * characterHalfExtents.Y)   // Top-Left
        };

        // 对每个关键点都进行一次 LineOfSight 检查
        foreach (var offset in offsets)
        {
            var fromPos = segmentStart + offset;
            var toPos = segmentEnd + offset;

            // 同样，需要一个精确的 GetVoxelIDAt 函数
            ulong fromVoxel = VoxelSearch.FindNearestEmptyVoxel(Volume, fromPos, characterHalfExtents);
            ulong toVoxel = VoxelSearch.FindNearestEmptyVoxel(Volume, toPos, characterHalfExtents);

            if (fromVoxel == VoxelMap.InvalidVoxel || toVoxel == VoxelMap.InvalidVoxel)
            {
                return false; // 偏移点在地图外
            }

            if (!VoxelSearch.LineOfSight(Volume, fromVoxel, toVoxel, fromPos, toPos))
            {
                return false; // 只要有一个角的路径被挡住，整个段就不安全
            }
        }

        return true; // 所有射线的路径都畅通无阻
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

    /// <summary>
    /// 对一个由关键拐点组成的路径进行智能平滑和居中。
    /// 此方法会同时考虑路径的几何平滑度和与环境的距离。
    /// </summary>
    /// <param name="path">要处理的路径，应只包含关键拐点。</param>
    /// <param name="characterHalfExtents">角色体积。</param>
    /// <param name="iterations">平滑迭代次数。推荐 3-5 次。</param>
    /// <param name="smoothness">平滑力。每次迭代中，点向几何中心移动的比例。推荐 0.5。</param>
    /// <param name="centeringBias">居中力。每次迭代中，点向安全中心移动的比例。推荐 0.1 - 0.3。</param>
    public void SmartPostProcessPath(List<Vector3> path, Vector3 characterHalfExtents, int iterations = 4, float smoothness = 0.55f, float centeringBias = 0.3f)
    {
        if (path.Count < 3) return;

        int pointCount = path.Count;
        // 创建一个工作副本，以避免在迭代中读取和写入同一个列表时产生问题
        var workingPath = new List<Vector3>(path);

        for (int iter = 0; iter < iterations; iter++)
        {
            // 每次迭代都基于上一次的结果
            // 注意：我们不对起点和终点进行处理
            for (int i = 1; i < pointCount - 1; i++)
            {
                // --- 关键改进：使用已调整过的前一个点 ---
                Vector3 prevPoint = workingPath[i - 1];
                Vector3 currentPoint = workingPath[i];
                Vector3 nextPoint = workingPath[i + 1];

                // 1. 计算“平滑”目标点
                // 这是纯粹的几何平滑，让当前点移动到前后邻居的连线中点
                Vector3 smoothTarget = prevPoint + (nextPoint - prevPoint) * 0.5f;
                Vector3 smoothedPoint = Vector3.Lerp(currentPoint, smoothTarget, smoothness);

                // 2. 计算“居中”目标点
                // 这是为了安全，将点推向局部空间的中心
                // 注意：我们在平滑后的点上计算，以获得更相关的环境信息
                Vector3 centerTarget = Find3DCenter(Volume, smoothedPoint, characterHalfExtents);

                // 3. 融合两个目标
                // 从平滑后的点，再向安全中心移动一点点
                Vector3 finalPoint = Vector3.Lerp(smoothedPoint, centerTarget, centeringBias);

                // 4. 安全校验 (可选但推荐)
                // 这一步可以简化，因为迭代和较小的centeringBias通常能保证安全
                if (VoxelSearch.FindNearestEmptyVoxel(Volume, finalPoint, characterHalfExtents) != VoxelMap.InvalidVoxel)
                {
                    workingPath[i] = finalPoint;
                }
                // 如果不安全，则在本次迭代中不移动该点，等待下一次迭代
            }
        }

        // 将最终平滑的结果写回原始列表
        for (int i = 0; i < pointCount; i++)
        {
            path[i] = workingPath[i];
        }
    }

    /// <summary>
    /// 【新增】从一个给定的水平位置(X,Z)开始，从上往下查找地面Y坐标。
    /// 这是一个比 ProbeDistance 更可靠的地面查找方法。
    /// </summary>
    /// <param name="volume">体素地图。</param>
    /// <param name="x">世界坐标X。</param>
    /// <param name="z">世界坐标Z。</param>
    /// <param name="startY">从哪个高度开始向下搜索。</param>
    /// <param name="searchDepth">最大向下搜索深度。</param>
    /// <returns>地面的Y坐标。如果没找到，返回一个非常低的值。</returns>
    private float FindGroundY(VoxelMap volume, float x, float z, float startY, float searchDepth = 10.0f)
    {
        // 假设体素大小为1.0f，你可以根据实际情况调整步长
        const float step = 0.5f;
        for (float y = startY; y > startY - searchDepth; y -= step)
        {
            // 假设你有一个函数可以检查世界坐标点是否在实体体素内
            // 这个检查比体积检查更简单、更快速、更可靠
            if (IsPositionInSolidVoxel(volume, new Vector3(x, y, z)))
            {
                // 找到了实体体素，地面就在这个体素的上方
                // 假设体素中心在y，体素高度为1，那么上表面就是 y + 0.5
                // 你需要根据你的体素坐标系进行调整
                return y + step; // 返回碰撞点之上的位置作为地面
            }
        }
        // 如果在搜索深度内没有找到地面，返回一个安全但极低的值
        return float.MinValue;
    }

    // 你需要实现这个辅助函数，它可能依赖于你的VoxelMap结构
    private bool IsPositionInSolidVoxel(VoxelMap volume, Vector3 pos)
    {
        // 这是一个示例实现，你需要替换成你自己的逻辑
        ulong voxelId = VoxelSearch.FindNearestEmptyVoxel(volume, pos, CharaHalfExtents); // 假设有这个函数
        if (voxelId == VoxelMap.InvalidVoxel) return true; // 地图外视为固体
        return !volume.IsEmpty(voxelId); // 假设有这个函数
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
