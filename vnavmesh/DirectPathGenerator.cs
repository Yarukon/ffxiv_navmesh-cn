using System;
using System.Collections.Generic;
using System.Linq;
using DotRecast.Core;
using DotRecast.Core.Numerics;
using DotRecast.Detour;
using DotRecast.Recast;

namespace Navmesh
{
    /// <summary>
    /// 自定义路径生成系统，提供更自然、平滑且带有随机性的路径
    /// </summary>
    public class DirectPathGenerator
    {
        private readonly DtNavMeshQuery _navQuery;
        private readonly Random _random;
        private readonly float _agentRadius;
        private readonly float _agentHeight;

        public DirectPathGenerator(DtNavMeshQuery navQuery, float agentRadius = 0.5f, float agentHeight = 2.0f, int? seed = null)
        {
            _navQuery = navQuery;
            _random = seed.HasValue ? new Random(seed.Value) : new Random();
            _agentRadius = agentRadius;
            _agentHeight = agentHeight;
        }

        /// <summary>
        /// 生成简洁直接的路径，类似于拉直算法但更自然
        /// </summary>
        /// <param name="filter">导航网格过滤器</param>
        /// <param name="directness">直接程度，0.0-1.0，值越大路径越直接</param>
        /// <param name="naturalness">自然程度，0.0-1.0，值越大路径越自然</param>
        /// <returns>处理后的路径点</returns>
        public List<RcVec3f> GenerateDirectPath(RcVec3f startPos, RcVec3f endPos, List<RcVec3f> rawPathPoints, IDtQueryFilter filter, float directness = 0.7f, float naturalness = 0.3f)
        {
            if (rawPathPoints.Count < 2)
                return rawPathPoints;

            // 创建一个新的路径点列表，以当前位置作为起点
            List<RcVec3f> adjustedPath = [startPos];

            // 添加原始路径的其余点（跳过原始起点）
            adjustedPath.AddRange(rawPathPoints[1..]);

            // 1. 首先应用路径内推，将路径从导航网格边缘推向中央
            var insetPath = ApplyPathInset(adjustedPath, filter);

            // 2. 识别关键转折点，大幅简化路径
            var keyPoints = IdentifyKeyPoints(insetPath, directness);

            // 3. 尝试直接连接关键点，进一步简化路径
            var directConnections = CreateDirectConnections(keyPoints, filter);

            // 4. 添加少量自然随机性，避免完全直线
            var naturalPath = AddSubtleRandomness(directConnections, filter, naturalness);

            // 5. 确保从当前位置到路径的连接
            return EnsurePathEndpoints(startPos, endPos, naturalPath, filter);
        }

        /// <summary>
        /// 将路径从导航网格边缘内推到中央位置
        /// </summary>
        private List<RcVec3f> ApplyPathInset(List<RcVec3f> path, IDtQueryFilter filter)
        {
            if (path.Count < 2)
                return path;

            var result = new List<RcVec3f>
            {
                // 将第一个点和最后一个点保持原样
                path[0]
            };

            // 处理中间的点
            for (int i = 1; i < path.Count - 1; i++)
            {
                // 获取当前点、前一个点和后一个点
                RcVec3f prevPoint = path[i - 1];
                RcVec3f currentPoint = path[i];
                RcVec3f nextPoint = path[i + 1];

                // 判断是否是靠近导航网格边缘的拐角点
                if (IsLikelyMeshEdgeCorner(prevPoint, currentPoint, nextPoint, filter))
                {
                    // 查找更好的内部点位置
                    RcVec3f betterPosition = FindBetterInteriorPosition(prevPoint, currentPoint, nextPoint, filter);
                    if (betterPosition != RcVec3f.Zero)
                    {
                        // 确保到新位置的路径有效
                        if (IsPathValid(prevPoint, betterPosition, filter) &&
                            IsPathValid(betterPosition, nextPoint, filter))
                        {
                            result.Add(betterPosition);
                            continue;
                        }
                    }
                }

                // 如果不是边缘拐角点或找不到更好的位置，保留原始点
                result.Add(currentPoint);
            }

            // 添加最后一个点
            result.Add(path[^1]);

            return result;
        }

        /// <summary>
        /// 检查一个点是否可能是导航网格边缘的拐角点
        /// </summary>
        private bool IsLikelyMeshEdgeCorner(RcVec3f prev, RcVec3f current, RcVec3f next, IDtQueryFilter filter)
        {
            // 计算进入向量和离开向量
            RcVec3f inVector = new RcVec3f(
                current.X - prev.X,
                0,
                current.Z - prev.Z
            );

            RcVec3f outVector = new RcVec3f(
                next.X - current.X,
                0,
                next.Z - current.Z
            );

            // 归一化
            float inLength = (float)Math.Sqrt(inVector.X * inVector.X + inVector.Z * inVector.Z);
            float outLength = (float)Math.Sqrt(outVector.X * outVector.X + outVector.Z * outVector.Z);

            if (inLength < 0.001f || outLength < 0.001f)
                return false;

            inVector = new RcVec3f(inVector.X / inLength, 0, inVector.Z / inLength);
            outVector = new RcVec3f(outVector.X / outLength, 0, outVector.Z / outLength);

            // 计算向量夹角
            float dot = inVector.X * outVector.X + inVector.Z * outVector.Z;
            float angle = (float)Math.Acos(Math.Max(-1.0f, Math.Min(1.0f, dot)));
            float degrees = angle * 180.0f / (float)Math.PI;

            // 检查是否是一个明显的转角（大于一定角度）
            if (degrees > 40.0f)
            {
                // 检查点附近的网格情况
                return HasLimitedClearanceToOneDirection(current, filter);
            }

            return false;
        }

        /// <summary>
        /// 检查一个点是否只在某个方向有有限的净空，表明它可能在网格边缘
        /// </summary>
        private bool HasLimitedClearanceToOneDirection(RcVec3f point, IDtQueryFilter filter)
        {
            float rayDistance = _agentRadius * 3.0f;
            int numDirections = 8; // 检查8个方向
            int blockedDirections = 0;

            // 首先确保点在导航网格上
            RcVec3f halfExtents = new RcVec3f(_agentRadius, _agentHeight, _agentRadius);
            long polyRef;
            RcVec3f polyPos;
            bool isOverPoly;

            DtStatus status = _navQuery.FindNearestPoly(point, halfExtents, filter,
                                                     out polyRef, out polyPos, out isOverPoly);

            if (!status.Succeeded() || polyRef == 0)
                return false;

            // 从该点向各个方向射线，检查哪些方向被阻挡
            for (int i = 0; i < numDirections; i++)
            {
                float angle = i * 2.0f * (float)Math.PI / numDirections;

                RcVec3f rayEnd = new RcVec3f(
                    point.X + rayDistance * (float)Math.Cos(angle),
                    point.Y,
                    point.Z + rayDistance * (float)Math.Sin(angle)
                );

                float t;
                RcVec3f hitNormal = new RcVec3f();
                var hitPath = new List<long>();

                DtStatus rayStatus = _navQuery.Raycast(polyRef, polyPos, rayEnd, filter,
                                                    out t, out hitNormal, ref hitPath);

                // 如果射线被阻挡
                if (rayStatus.Succeeded() && t < 1.0f)
                {
                    blockedDirections++;
                }

                hitPath.Clear();
            }

            // 如果至少三分之一的方向被阻挡，则认为是在边缘
            return blockedDirections >= numDirections / 3;
        }

        /// <summary>
        /// 查找更好的内部点位置，远离导航网格边缘
        /// </summary>
        private RcVec3f FindBetterInteriorPosition(RcVec3f prev, RcVec3f current, RcVec3f next, IDtQueryFilter filter)
        {
            // 计算到当前点的方向和离开当前点的方向
            RcVec3f inDir = new RcVec3f(
                current.X - prev.X,
                0,
                current.Z - prev.Z
            );

            RcVec3f outDir = new RcVec3f(
                next.X - current.X,
                0,
                next.Z - current.Z
            );

            // 归一化方向向量
            float inLength = (float)Math.Sqrt(inDir.X * inDir.X + inDir.Z * inDir.Z);
            float outLength = (float)Math.Sqrt(outDir.X * outDir.X + outDir.Z * outDir.Z);

            if (inLength < 0.001f || outLength < 0.001f)
                return RcVec3f.Zero;

            inDir = new RcVec3f(inDir.X / inLength, 0, inDir.Z / inLength);
            outDir = new RcVec3f(outDir.X / outLength, 0, outDir.Z / outLength);

            // 计算角平分线方向（指向拐角内部）
            RcVec3f bisector = new RcVec3f(
                -inDir.X - outDir.X,
                0,
                -inDir.Z - outDir.Z
            );

            float bisectorLength = (float)Math.Sqrt(bisector.X * bisector.X + bisector.Z * bisector.Z);

            if (bisectorLength < 0.001f)
            {
                // 如果角平分线长度接近零（表示180度转弯），使用垂直于走向的方向
                bisector = new RcVec3f(-inDir.Z, 0, inDir.X);
                bisectorLength = 1.0f;
            }

            bisector = new RcVec3f(bisector.X / bisectorLength, 0, bisector.Z / bisectorLength);

            // 尝试沿着角平分线方向（指向内部）查找更好的点
            // 试探不同距离的点，找到一个最优的
            float[] offsets = { 1.5f, 2.0f, 2.5f, 3.0f, 3.5f };

            // 如果角度比较小，使用较小的偏移
            float dot = inDir.X * outDir.X + inDir.Z * outDir.Z;
            if (dot > 0) // 角度小于90度
            {
                offsets = new float[] { 1.0f, 1.5f, 2.0f, 2.5f };
            }

            foreach (float offset in offsets)
            {
                // 计算候选点位置
                RcVec3f candidatePos = new RcVec3f(
                    current.X + bisector.X * offset * _agentRadius,
                    current.Y,
                    current.Z + bisector.Z * offset * _agentRadius
                );

                // 确保点在导航网格上
                RcVec3f halfExtents = new RcVec3f(_agentRadius, _agentHeight, _agentRadius);
                long polyRef;
                RcVec3f polyPos;
                bool isOverPoly;

                DtStatus status = _navQuery.FindNearestPoly(candidatePos, halfExtents, filter,
                                                         out polyRef, out polyPos, out isOverPoly);

                if (status.Succeeded() && polyRef != 0)
                {
                    // 检查候选点是否有足够的避障空间且能连接
                    if (HasAdequateClearance(polyPos, filter) &&
                        IsPathValid(prev, polyPos, filter) &&
                        IsPathValid(polyPos, next, filter))
                    {
                        // 检查候选点到各方向的通行性
                        if (!HasLimitedClearanceToOneDirection(polyPos, filter))
                        {
                            return polyPos;
                        }
                    }
                }
            }

            // 如果没有找到更好的位置，尝试沿两个方向的中间线
            RcVec3f midDir = new RcVec3f(
                inDir.X + outDir.X,
                0,
                inDir.Z + outDir.Z
            );

            float midLength = (float)Math.Sqrt(midDir.X * midDir.X + midDir.Z * midDir.Z);
            if (midLength > 0.001f)
            {
                midDir = new RcVec3f(midDir.X / midLength, 0, midDir.Z / midLength);

                // 尝试沿中间方向偏移
                for (float offset = 1.0f; offset <= 3.0f; offset += 0.5f)
                {
                    RcVec3f candidatePos = new RcVec3f(
                        current.X + midDir.X * offset * _agentRadius,
                        current.Y,
                        current.Z + midDir.Z * offset * _agentRadius
                    );

                    // 确保点在导航网格上
                    RcVec3f halfExtents = new RcVec3f(_agentRadius, _agentHeight, _agentRadius);
                    long polyRef;
                    RcVec3f polyPos;
                    bool isOverPoly;

                    DtStatus status = _navQuery.FindNearestPoly(candidatePos, halfExtents, filter,
                                                            out polyRef, out polyPos, out isOverPoly);

                    if (status.Succeeded() && polyRef != 0 &&
                        HasAdequateClearance(polyPos, filter) &&
                        IsPathValid(prev, polyPos, filter) &&
                        IsPathValid(polyPos, next, filter))
                    {
                        return polyPos;
                    }
                }
            }

            return RcVec3f.Zero; // 如果没有找到更好的位置
        }

        /// <summary>
        /// 识别路径中的关键转折点
        /// </summary>
        private List<RcVec3f> IdentifyKeyPoints(List<RcVec3f> path, float directnessFactor)
        {
            if (path.Count <= 2)
                return path;

            var result = new List<RcVec3f>();
            result.Add(path[0]); // 添加起点

            // 设置角度阈值，directnessFactor越大，阈值越大，保留的点越少
            float angleThreshold = 25.0f + directnessFactor * 20.0f; // 25-45度范围
            float cosThreshold = (float)Math.Cos(angleThreshold * Math.PI / 180.0f);

            // 设置距离阈值，directnessFactor越大，阈值越大，保留的点越少
            float distanceThreshold = 2.0f + directnessFactor * 3.0f; // 2-5单位范围

            RcVec3f prevDirection = new RcVec3f(
                path[1].X - path[0].X,
                0,
                path[1].Z - path[0].Z
            );

            float prevDirLength = (float)Math.Sqrt(prevDirection.X * prevDirection.X + prevDirection.Z * prevDirection.Z);
            if (prevDirLength > 0.0001f)
            {
                prevDirection = new RcVec3f(
                    prevDirection.X / prevDirLength,
                    0,
                    prevDirection.Z / prevDirLength
                );
            }

            float accumulatedDistance = 0;

            for (int i = 1; i < path.Count - 1; i++)
            {
                RcVec3f currentDirection = new RcVec3f(
                    path[i + 1].X - path[i].X,
                    0,
                    path[i + 1].Z - path[i].Z
                );

                float currentDirLength = (float)Math.Sqrt(currentDirection.X * currentDirection.X + currentDirection.Z * currentDirection.Z);
                accumulatedDistance += currentDirLength;

                if (currentDirLength > 0.0001f)
                {
                    currentDirection = new RcVec3f(
                        currentDirection.X / currentDirLength,
                        0,
                        currentDirection.Z / currentDirLength
                    );

                    // 计算方向变化的点积
                    float dotProduct = prevDirection.X * currentDirection.X + prevDirection.Z * currentDirection.Z;

                    // 如果方向变化大于阈值或者累积距离大于阈值，保留该点
                    // 对于特别大的转向角度更可能保留点
                    bool isSignificantTurn = dotProduct < (float)Math.Cos(35.0f * Math.PI / 180.0f);
                    bool isNormalTurn = dotProduct < cosThreshold;
                    bool isDistancePoint = accumulatedDistance > distanceThreshold;

                    if (isSignificantTurn || (isNormalTurn && isDistancePoint))
                    {
                        result.Add(path[i]);
                        prevDirection = currentDirection;
                        accumulatedDistance = 0;
                    }
                }
            }

            result.Add(path[path.Count - 1]); // 添加终点
            return result;
        }

        /// <summary>
        /// 创建直接连接，使用增强的验证，避免拐角卡住
        /// </summary>
        private List<RcVec3f> CreateDirectConnections(List<RcVec3f> keyPoints, IDtQueryFilter filter)
        {
            if (keyPoints.Count <= 2)
                return keyPoints;

            var result = new List<RcVec3f>();
            result.Add(keyPoints[0]); // 添加起点

            int currentIndex = 0;

            while (currentIndex < keyPoints.Count - 1)
            {
                // 从当前点开始，尝试直接连接到尽可能远的点
                int farthestValidIndex = currentIndex + 1;

                // 限制最大跳跃距离，避免尝试连接太远的点
                int maxLookAhead = Math.Min(10, keyPoints.Count - currentIndex - 1);

                for (int i = currentIndex + 2; i <= currentIndex + maxLookAhead; i++)
                {
                    // 使用保守的路径验证
                    if (IsPathValidWithClearance(keyPoints[currentIndex], keyPoints[i], filter))
                    {
                        farthestValidIndex = i;
                    }
                    else
                    {
                        // 一旦找到不可直接到达的点，就停止搜索
                        break;
                    }
                }

                // 添加可直接到达的最远点
                result.Add(keyPoints[farthestValidIndex]);
                currentIndex = farthestValidIndex;
            }

            return result;
        }

        /// <summary>
        /// 检查两点之间的路径是否可行，同时确保有足够的避障空间
        /// </summary>
        private bool IsPathValidWithClearance(RcVec3f start, RcVec3f end, IDtQueryFilter filter)
        {
            // 基本的路径有效性检查
            if (!IsPathValid(start, end, filter))
                return false;

            // 额外检查：确保路径有足够的避障空间，避免刮擦墙壁和拐角

            // 找到起点和终点所在的多边形
            RcVec3f halfExtents = new RcVec3f(_agentRadius, _agentHeight, _agentRadius);

            long startRef;
            RcVec3f startPos;
            bool startIsOverPoly;

            long endRef;
            RcVec3f endPos;
            bool endIsOverPoly;

            _navQuery.FindNearestPoly(start, halfExtents, filter, out startRef, out startPos, out startIsOverPoly);
            _navQuery.FindNearestPoly(end, halfExtents, filter, out endRef, out endPos, out endIsOverPoly);

            if (startRef == 0 || endRef == 0)
                return false;

            // 使用raycast检查路径上的点与障碍物的距离
            float t;
            RcVec3f hitNormal = new RcVec3f();
            var hitPath = new List<long>();

            DtStatus rayStatus = _navQuery.Raycast(startRef, startPos, endPos, filter, out t, out hitNormal, ref hitPath);

            // 如果raycast没有被阻挡，检查路径上的多个点的避障空间
            if (rayStatus.Succeeded() && t >= 1.0f)
            {
                float pathLength = RcVec3f.Distance(start, end);

                // 对于较长的路径，检查中间点的避障空间
                if (pathLength > 2.0f)
                {
                    int numSamples = Math.Max(3, (int)(pathLength / 1.5f));

                    for (int i = 1; i < numSamples - 1; i++)
                    {
                        float sampleT = i / (float)(numSamples - 1);
                        RcVec3f samplePoint = new RcVec3f(
                            start.X + (end.X - start.X) * sampleT,
                            start.Y + (end.Y - start.Y) * sampleT,
                            start.Z + (end.Z - start.Z) * sampleT
                        );

                        // 检查该点的避障空间
                        if (!HasAdequateClearance(samplePoint, filter))
                        {
                            return false;
                        }
                    }
                }

                return true;
            }

            return false;
        }

        /// <summary>
        /// 检查指定点是否有足够的避障空间
        /// </summary>
        private bool HasAdequateClearance(RcVec3f point, IDtQueryFilter filter)
        {
            // 查找点所在的多边形
            RcVec3f halfExtents = new RcVec3f(_agentRadius, _agentHeight, _agentRadius);
            long polyRef;
            RcVec3f polyPos;
            bool isOverPoly;

            DtStatus status = _navQuery.FindNearestPoly(point, halfExtents, filter,
                                                      out polyRef, out polyPos, out isOverPoly);

            if (!status.Succeeded() || polyRef == 0)
                return false;

            // 获取当前多边形的信息
            DtMeshTile tile;
            DtPoly poly;
            _navQuery.GetAttachedNavMesh().GetTileAndPolyByRef(polyRef, out tile, out poly);

            if (tile == null || poly == null)
                return false;

            // 计算点到多边形边缘的最小距离
            // 注意：这里使用简化计算，实际导航网格的边缘距离计算更复杂

            // 为简化处理，我们使用额外的安全距离检查
            // 检查点是否足够靠近poly中心
            float distToPoly = RcVec3f.Distance(point, polyPos);

            // 如果点太靠近多边形边缘，认为空间不足
            // 为避免与复杂导航网格几何形状计算冲突，使用比例因子
            float safetyFactor = 0.7f; // 安全距离系数，越小越安全
            bool hasClearance = distToPoly < _agentRadius * safetyFactor;

            return hasClearance;
        }

        /// <summary>
        /// 添加微妙的随机性，避免完全直线
        /// </summary>
        private List<RcVec3f> AddSubtleRandomness(List<RcVec3f> directPath, IDtQueryFilter filter, float naturalnessFactor)
        {
            if (directPath.Count <= 2 || naturalnessFactor < 0.01f)
                return directPath;

            var result = new List<RcVec3f>
            {
                directPath[0] // 添加起点
            };

            // 对于较长的段，考虑在中间添加一个微小偏移的点
            for (int i = 0; i < directPath.Count - 1; i++)
            {
                RcVec3f start = directPath[i];
                RcVec3f end = directPath[i + 1];

                float segmentLength = RcVec3f.Distance(start, end);

                // 只对较长的段添加随机性
                if (segmentLength > 3.0f)
                {
                    // 在路径上找多个可能的随机点位置，选择最佳的一个
                    List<RcVec3f> candidatePoints = [];

                    // 首先尝试直接连接，如果可行就不添加随机点
                    if (IsPathValid(start, end, filter))
                    {
                        // 即使直接路径有效，也有50%几率尝试添加随机点以增加路径变化
                        if (_random.NextDouble() > 0.3)
                        {
                            result.Add(end);
                            continue;
                        }
                    }

                    // 在两点之间选择几个位置
                    List<float> positions =
                    [
                        // 始终包含中点
                        0.5f,
                    ];
                    // 添加一些其他位置
                    if (_random.NextDouble() > 0.5f) positions.Add(0.33f);
                    if (_random.NextDouble() > 0.5f) positions.Add(0.66f);

                    foreach (float t in positions)
                    {
                        // 计算段中点
                        RcVec3f midPoint = new RcVec3f(
                            start.X + (end.X - start.X) * t,
                            start.Y + (end.Y - start.Y) * t,
                            start.Z + (end.Z - start.Z) * t
                        );

                        // 计算方向向量
                        RcVec3f direction = new RcVec3f(
                            end.X - start.X,
                            0,
                            end.Z - start.Z
                        );

                        float dirLength = (float)Math.Sqrt(direction.X * direction.X + direction.Z * direction.Z);

                        if (dirLength > 0.0001f)
                        {
                            direction = new RcVec3f(
                                direction.X / dirLength,
                                0,
                                direction.Z / dirLength
                            );

                            // 计算垂直于方向的向量
                            RcVec3f perpDir = new RcVec3f(
                                -direction.Z,
                                0,
                                direction.X
                            );

                            // 计算最大随机偏移距离 - 减小最大偏移
                            float maxOffset = segmentLength * 0.12f * naturalnessFactor;

                            // 首先添加一个不偏移的中点候选
                            RcVec3f centerPoint = midPoint;
                            RcVec3f halfExtents = new RcVec3f(_agentRadius, _agentHeight, _agentRadius);
                            long centerRef;
                            RcVec3f centerPosOnMesh;
                            bool centerIsOverPoly;

                            DtStatus centerStatus = _navQuery.FindNearestPoly(centerPoint, halfExtents, filter,
                                                                    out centerRef, out centerPosOnMesh, out centerIsOverPoly);

                            if (centerStatus.Succeeded() && centerRef != 0)
                            {
                                candidatePoints.Add(centerPosOnMesh);
                            }

                            // 尝试少量偏移
                            for (int k = 0; k < 3; k++) // 减少尝试次数
                            {
                                // 使用更小的随机范围
                                float r1 = (float)_random.NextDouble() * 2 - 1;
                                float r2 = (float)_random.NextDouble() * 2 - 1;
                                float randomFactor = (r1 + r2) / 3.0f; // 更集中在中间

                                float randomOffset = randomFactor * maxOffset;

                                // 计算随机点
                                RcVec3f randomPoint = new RcVec3f(
                                    midPoint.X + perpDir.X * randomOffset,
                                    midPoint.Y, // 保持Y值不变
                                    midPoint.Z + perpDir.Z * randomOffset
                                );

                                // 确保随机点在导航网格上
                                long randomRef;
                                RcVec3f randomPosOnMesh;
                                bool randomIsOverPoly;

                                DtStatus status = _navQuery.FindNearestPoly(randomPoint, halfExtents, filter,
                                                                        out randomRef, out randomPosOnMesh, out randomIsOverPoly);

                                if (status.Succeeded() && randomRef != 0)
                                {
                                    // 只添加有足够避障空间的点
                                    bool hasClearance = HasAdequateClearance(randomPosOnMesh, filter);
                                    bool canWalkFromStart = IsPathValid(start, randomPosOnMesh, filter);
                                    bool canWalkToEnd = IsPathValid(randomPosOnMesh, end, filter);

                                    if (hasClearance && canWalkFromStart && canWalkToEnd)
                                    {
                                        candidatePoints.Add(randomPosOnMesh);
                                    }
                                }
                            }
                        }
                    }

                    // 从候选点中找出最佳的一个
                    RcVec3f bestPoint = FindBestRandomPoint(start, end, candidatePoints, filter);
                    if (bestPoint != RcVec3f.Zero)
                    {
                        result.Add(bestPoint);
                    }
                    else if (IsPathValid(start, end, filter))
                    {
                        // 如果没有找到有效的随机点，但直接连接有效，则不添加中间点
                        // 不做任何事情，直接添加终点
                    }
                    else
                    {
                        // 如果没有找到有效的随机点，且直接连接无效，尝试使用原始路径中的点
                        // 查找原始路径中这两点之间的点
                        var originalPoints = FindOriginalPointsBetween(directPath, start, end);
                        if (originalPoints.Count > 0)
                        {
                            result.AddRange(originalPoints);
                        }
                    }
                }

                result.Add(end);
            }

            return result;
        }

        /// <summary>
        /// 找出原始路径中两点之间的点
        /// </summary>
        private List<RcVec3f> FindOriginalPointsBetween(List<RcVec3f> originalPath, RcVec3f start, RcVec3f end)
        {
            var points = new List<RcVec3f>();

            int startIndex = -1;
            int endIndex = -1;

            // 找到起点和终点在原始路径中的索引
            for (int i = 0; i < originalPath.Count; i++)
            {
                if (RcVec3f.Distance(originalPath[i], start) < 0.01f)
                {
                    startIndex = i;
                }
                if (startIndex >= 0 && RcVec3f.Distance(originalPath[i], end) < 0.01f)
                {
                    endIndex = i;
                    break;
                }
            }

            // 如果找到了起点和终点，添加它们之间的点
            if (startIndex >= 0 && endIndex > startIndex + 1)
            {
                for (int i = startIndex + 1; i < endIndex; i++)
                {
                    points.Add(originalPath[i]);
                }
            }

            return points;
        }

        /// <summary>
        /// 从候选点中找出最佳的随机点
        /// </summary>
        private RcVec3f FindBestRandomPoint(RcVec3f start, RcVec3f end, List<RcVec3f> candidates, IDtQueryFilter filter)
        {
            if (candidates.Count == 0)
                return RcVec3f.Zero;

            List<KeyValuePair<RcVec3f, float>> validPoints = new List<KeyValuePair<RcVec3f, float>>();

            foreach (var point in candidates)
            {
                // 首先严格验证路径有效性
                bool canWalkFromStart = IsPathValid(start, point, filter, true);
                bool canWalkToEnd = IsPathValid(point, end, filter, true);

                if (canWalkFromStart && canWalkToEnd)
                {
                    // 计算得分 - 偏好那些不会导致路径过于曲折的点
                    float directDistance = RcVec3f.Distance(start, end);
                    float newPathDistance = RcVec3f.Distance(start, point) + RcVec3f.Distance(point, end);

                    // 偏好那些路径增加不多的点
                    float detourRatio = directDistance / newPathDistance;

                    // 计算点到直线的距离（自然度）
                    float naturalnessFactor = PointToLineDistance(point, start, end) / directDistance;

                    // 组合得分：更高的detourRatio（接近1）和适度的naturalnessFactor得到更高分数
                    float score = detourRatio * 0.8f - Math.Abs(naturalnessFactor - 0.03f) * 3.0f;

                    // 添加小的随机性到得分中
                    score += (float)_random.NextDouble() * 0.05f;

                    validPoints.Add(new KeyValuePair<RcVec3f, float>(point, score));
                }
            }

            if (validPoints.Count == 0)
                return RcVec3f.Zero;

            // 按得分排序
            validPoints.Sort((a, b) => b.Value.CompareTo(a.Value));

            // 从前3个（或更少如果没有那么多）中随机选择一个
            int topN = Math.Min(3, validPoints.Count);
            int selectedIndex = _random.Next(topN);

            return validPoints[selectedIndex].Key;
        }

        /// <summary>
        /// 计算点到线段的距离
        /// </summary>
        private float PointToLineDistance(RcVec3f point, RcVec3f lineStart, RcVec3f lineEnd)
        {
            float lineLength = RcVec3f.Distance(lineStart, lineEnd);
            if (lineLength < 0.0001f)
                return RcVec3f.Distance(point, lineStart);

            // 将高度分量（Y轴）归零，只考虑水平平面距离
            RcVec3f a = new RcVec3f(lineStart.X, 0, lineStart.Z);
            RcVec3f b = new RcVec3f(lineEnd.X, 0, lineEnd.Z);
            RcVec3f p = new RcVec3f(point.X, 0, point.Z);

            // 计算向量
            RcVec3f ab = new RcVec3f(b.X - a.X, 0, b.Z - a.Z);
            RcVec3f ap = new RcVec3f(p.X - a.X, 0, p.Z - a.Z);

            // 计算投影比例
            float t = (ap.X * ab.X + ap.Z * ab.Z) / (ab.X * ab.X + ab.Z * ab.Z);
            t = Math.Max(0, Math.Min(1, t)); // 限制在线段范围内

            // 计算投影点
            RcVec3f projection = new RcVec3f(
                a.X + t * ab.X,
                0,
                a.Z + t * ab.Z
            );

            // 返回点到投影点的距离
            return RcVec3f.Distance(p, projection);
        }

        /// <summary>
        /// 检查两点之间的路径是否可行，使用保守的验证策略
        /// </summary>
        private bool IsPathValid(RcVec3f start, RcVec3f end, IDtQueryFilter filter, bool strict = false)
        {
            // 找到起点和终点所在的多边形
            RcVec3f halfExtents = new RcVec3f(_agentRadius, _agentHeight, _agentRadius);

            long startRef;
            RcVec3f startPos;
            bool startIsOverPoly;

            long endRef;
            RcVec3f endPos;
            bool endIsOverPoly;

            DtStatus startStatus = _navQuery.FindNearestPoly(start, halfExtents, filter,
                                                          out startRef, out startPos, out startIsOverPoly);

            DtStatus endStatus = _navQuery.FindNearestPoly(end, halfExtents, filter,
                                                        out endRef, out endPos, out endIsOverPoly);

            if (!startStatus.Succeeded() || !endStatus.Succeeded() || startRef == 0 || endRef == 0)
                return false;

            // 如果起点和终点在同一个多边形内，路径有效
            if (startRef == endRef)
                return true;

            // 计算路径长度
            float pathLength = RcVec3f.Distance(start, end);

            // 对于短路径，简单的raycast就足够了
            if (pathLength < 2.0f)
            {
                return PerformRaycast(startRef, startPos, endPos, filter);
            }

            // 对于较长的路径，使用更可靠的验证方法
            if (strict)
            {
                // 严格模式：使用完整路径查找
                return ValidateWithFullPathfinding(startRef, startPos, endRef, endPos, filter);
            }
            else
            {
                // 先尝试raycast
                if (PerformRaycast(startRef, startPos, endPos, filter))
                {
                    return true;
                }

                // 如果raycast失败，使用简化的路径查找
                return ValidateWithSimplePathfinding(startRef, startPos, endRef, endPos, filter);
            }
        }

        /// <summary>
        /// 使用简化的路径查找验证路径
        /// </summary>
        private bool ValidateWithSimplePathfinding(long startRef, RcVec3f startPos, long endRef, RcVec3f endPos, IDtQueryFilter filter)
        {
            // 使用路径查找但限制节点数
            var path = new List<long>();
            var fpo = new DtFindPathOption(DtFindPathOptions.DT_FINDPATH_ANY_ANGLE, 8); // 减少搜索深度以提高性能

            DtStatus pathStatus = _navQuery.FindPath(startRef, endRef, startPos, endPos, filter, ref path, fpo);

            // 如果能找到路径，则认为是有效的
            return pathStatus.Succeeded() && path.Count > 0;
        }

        /// <summary>
        /// 执行raycast检查
        /// </summary>
        private bool PerformRaycast(long startRef, RcVec3f startPos, RcVec3f endPos, IDtQueryFilter filter)
        {
            float t;
            RcVec3f hitNormal = new RcVec3f();
            var hitPath = new List<long>();

            try
            {
                DtStatus rayStatus = _navQuery.Raycast(startRef, startPos, endPos, filter, out t, out hitNormal, ref hitPath);

                // 如果光线没有被阻挡，路径有效
                return rayStatus.Succeeded() && t >= 1.0f;
            }
            finally
            {
                hitPath.Clear();
            }
        }

        private List<RcVec3f> EnsurePathEndpoints(RcVec3f startPoint, RcVec3f endPoint,
                                    List<RcVec3f> path, IDtQueryFilter filter)
        {
            // 如果路径为空，直接创建一条从起点到终点的路径
            if (path == null || path.Count == 0)
            {
                var result = new List<RcVec3f>
                {
                    startPoint,
                    endPoint
                };
                return result;
            }

            // 创建新的结果路径
            var finalPath = new List<RcVec3f>();

            // 处理起点 - 使用raycast检查是否可以直接连接
            if (RcVec3f.Distance(path[0], startPoint) > 0.1f)
            {
                // 使用raycast检查是否可以直接从起点到第一个路径点
                if (CanRaycast(startPoint, path[0], filter))
                {
                    // 可以直接连接
                    finalPath.Add(startPoint);
                }
                else
                {
                    // 无法直接连接，添加起点和原始第一个点
                    finalPath.Add(startPoint);
                    finalPath.Add(path[0]);
                }
            }
            else
            {
                // 起点已经正确
                finalPath.Add(startPoint);
            }

            // 添加中间点，跳过第一个点（已经处理过）
            for (int i = 1; i < path.Count - 1; i++)
            {
                // 检查是否与上一个点重复
                if (RcVec3f.Distance(path[i], finalPath[^1]) > 0.01f)
                {
                    finalPath.Add(path[i]);
                }
            }

            // 处理终点 - 使用raycast检查是否可以直接连接
            int lastIndex = finalPath.Count - 1;
            if (path.Count > 1)
            {
                if (RcVec3f.Distance(path[^1], endPoint) > 0.1f)
                {
                    // 使用raycast检查是否可以直接从最后一个点到终点
                    if (CanRaycast(finalPath[lastIndex], endPoint, filter))
                    {
                        // 可以直接连接
                        finalPath.Add(endPoint);
                    }
                    else
                    {
                        // 无法直接连接，添加原始最后一个点和终点
                        if (path.Count > 1 && lastIndex >= 0 && finalPath[lastIndex] != path[^1])
                        {
                            finalPath.Add(path[^1]);
                        }
                        finalPath.Add(endPoint);
                    }
                }
                else
                {
                    // 终点已经正确
                    finalPath.Add(endPoint);
                }
            }
            else
            {
                // 路径只有一个点，直接添加终点
                finalPath.Add(endPoint);
            }

            // 确保最终路径没有重复点和太近的点
            var optimizedPath = SmoothPathCorners(finalPath, filter);
            return optimizedPath;
        }

        /// <summary>
        /// 使用完整的路径查找验证路径
        /// </summary>
        private bool ValidateWithFullPathfinding(long startRef, RcVec3f startPos, long endRef, RcVec3f endPos, IDtQueryFilter filter)
        {
            // 使用路径查找
            var path = new List<long>();
            var fpo = new DtFindPathOption(DtFindPathOptions.DT_FINDPATH_ANY_ANGLE, 16);

            DtStatus pathStatus = _navQuery.FindPath(startRef, endRef, startPos, endPos, filter, ref path, fpo);

            if (!pathStatus.Succeeded() || path.Count == 0)
                return false;

            // 生成直线路径
            var straightPath = new List<DtStraightPath>();
            _navQuery.FindStraightPath(startPos, endPos, path, ref straightPath, 16, 0);

            if (straightPath.Count < 2)
                return false;

            // 计算直线距离
            float directDistance = RcVec3f.Distance(startPos, endPos);

            // 计算路径长度
            float pathLength = 0;
            for (int i = 0; i < straightPath.Count - 1; i++)
            {
                pathLength += RcVec3f.Distance(straightPath[i].pos, straightPath[i + 1].pos);
            }

            // 如果路径长度接近直线距离，则认为是有效的直接连接
            // 放宽一点检查条件：允许路径比直线长最多15%
            return pathLength <= directDistance * 1.15f;
        }

        /// <summary>
        /// 使用raycast检查两点之间是否可以直接连接，保守版本
        /// </summary>
        private bool CanRaycast(RcVec3f start, RcVec3f end, IDtQueryFilter filter)
        {
            // 直接使用IsPathValid，确保一致性
            return IsPathValid(start, end, filter);
        }

        /// <summary>
        /// 平滑路径拐角，避免卡住
        /// </summary>
        private List<RcVec3f> SmoothPathCorners(List<RcVec3f> path, IDtQueryFilter filter)
        {
            if (path.Count < 3)
                return path;

            var result = new List<RcVec3f>();
            result.Add(path[0]); // 添加起点

            // 处理中间的拐角点
            for (int i = 1; i < path.Count - 1; i++)
            {
                RcVec3f prev = path[i - 1];
                RcVec3f current = path[i];
                RcVec3f next = path[i + 1];

                // 计算两个方向向量
                RcVec3f dir1 = new RcVec3f(
                    current.X - prev.X,
                    0,
                    current.Z - prev.Z
                );

                RcVec3f dir2 = new RcVec3f(
                    next.X - current.X,
                    0,
                    next.Z - current.Z
                );

                // 归一化方向向量
                float len1 = (float)Math.Sqrt(dir1.X * dir1.X + dir1.Z * dir1.Z);
                float len2 = (float)Math.Sqrt(dir2.X * dir2.X + dir2.Z * dir2.Z);

                if (len1 > 0.001f && len2 > 0.001f)
                {
                    dir1 = new RcVec3f(dir1.X / len1, 0, dir1.Z / len1);
                    dir2 = new RcVec3f(dir2.X / len2, 0, dir2.Z / len2);

                    // 计算夹角的点积
                    float dot = dir1.X * dir2.X + dir1.Z * dir2.Z;
                    float angle = (float)Math.Acos(Math.Max(-1.0f, Math.Min(1.0f, dot)));

                    // 如果角度很尖锐(小于约120度)，需要平滑
                    if (angle > Math.PI / 3.0f)
                    {
                        // 计算两条路径段的中间方向
                        RcVec3f midDir = new RcVec3f(
                            dir1.X + dir2.X,
                            0,
                            dir1.Z + dir2.Z
                        );

                        float midLen = (float)Math.Sqrt(midDir.X * midDir.X + midDir.Z * midDir.Z);
                        if (midLen > 0.001f)
                        {
                            midDir = new RcVec3f(midDir.X / midLen, 0, midDir.Z / midLen);

                            // 生成拐角前后一定距离的两个点，用于圆滑拐角
                            float cornerDistance = _agentRadius * 1.5f; // 拐角距离，根据需要调整

                            RcVec3f beforeCorner = new RcVec3f(
                                current.X - dir1.X * cornerDistance,
                                current.Y,
                                current.Z - dir1.Z * cornerDistance
                            );

                            RcVec3f afterCorner = new RcVec3f(
                                current.X + dir2.X * cornerDistance,
                                current.Y,
                                current.Z + dir2.Z * cornerDistance
                            );

                            // 验证这些点是否可行
                            if (IsPathValidWithClearance(prev, beforeCorner, filter) &&
                                IsPathValidWithClearance(beforeCorner, afterCorner, filter) &&
                                IsPathValidWithClearance(afterCorner, next, filter))
                            {
                                // 使用这两个点替代原拐角点
                                result.Add(beforeCorner);
                                result.Add(afterCorner);
                                continue; // 跳过添加原拐角点
                            }
                        }
                    }
                }

                // 如果没有平滑或平滑失败，检查原点是否有足够空间
                if (HasAdequateClearance(current, filter))
                {
                    result.Add(current);
                }
                else
                {
                    // 如果拐角点空间不足，尝试找到附近有足够空间的点
                    RcVec3f adjustedCorner = FindSafeCornerPosition(current, filter);
                    if (adjustedCorner != RcVec3f.Zero &&
                        IsPathValidWithClearance(prev, adjustedCorner, filter) &&
                        IsPathValidWithClearance(adjustedCorner, next, filter))
                    {
                        result.Add(adjustedCorner);
                    }
                    else
                    {
                        // 如果无法找到安全点，保留原点但标记为可能有问题
                        result.Add(current);
                    }
                }
            }

            result.Add(path[path.Count - 1]); // 添加终点
            return result;
        }

        /// <summary>
        /// 寻找拐角附近安全的位置
        /// </summary>
        private RcVec3f FindSafeCornerPosition(RcVec3f corner, IDtQueryFilter filter)
        {
            // 在拐角周围搜索安全点
            RcVec3f halfExtents = new RcVec3f(_agentRadius * 2, _agentHeight, _agentRadius * 2);
            long startRef;
            RcVec3f startPos;
            bool isOverPoly;

            DtStatus status = _navQuery.FindNearestPoly(corner, halfExtents, filter,
                                                      out startRef, out startPos, out isOverPoly);

            if (!status.Succeeded() || startRef == 0)
                return RcVec3f.Zero;

            // 使用寻路系统找到周围安全点
            var result = new RcVec3f();
            bool found = false;

            // 先尝试附近的几个方向
            float[] directions = { 0, 45, 90, 135, 180, 225, 270, 315 };
            float safeDistance = _agentRadius * 1.2f;

            foreach (float angle in directions)
            {
                // 计算该方向上的点
                float rad = angle * (float)Math.PI / 180.0f;
                RcVec3f targetPos = new RcVec3f(
                    corner.X + safeDistance * (float)Math.Cos(rad),
                    corner.Y,
                    corner.Z + safeDistance * (float)Math.Sin(rad)
                );

                // 检查该点是否安全
                long targetRef;
                RcVec3f targetPosOnMesh;
                bool targetIsOverPoly;

                DtStatus targetStatus = _navQuery.FindNearestPoly(targetPos, halfExtents, filter,
                                                                out targetRef, out targetPosOnMesh, out targetIsOverPoly);

                if (targetStatus.Succeeded() && targetRef != 0 && HasAdequateClearance(targetPosOnMesh, filter))
                {
                    result = targetPosOnMesh;
                    found = true;
                    break;
                }
            }

            return found ? result : RcVec3f.Zero;
        }
    }
}
