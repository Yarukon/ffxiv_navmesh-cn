using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using DotRecast.Core.Numerics;
using DotRecast.Detour;

namespace Navmesh
{
    /// <summary>
    /// 路径随机化工具，用于使dotRecast生成的路径更加自然
    /// </summary>
    public class PathRandomizer
    {
        private readonly DtNavMeshQuery _navQuery;
        private readonly float _agentRadius;
        private readonly float _agentHeight;
        private readonly Random _random;

        // 配置参数
        private float _minDistanceForRandomization = 3.0f; // 最小需要随机化的距离
        private float _maxDeviationRatio = 0.4f; // 最大偏移距离比例（相对于两点间距离）
        private int _minPointsPerSegment = 2; // 每段最少插入点数
        private int _maxPointsPerSegment = 5; // 每段最多插入点数

        // 随机性参数
        private float _randomnessFactor = 0.7f; // 控制路径的随机程度 (0-1)

        // 障碍物检测参数
        private int _obstacleCheckSamples = 4; // 检查障碍物的采样点数量
        private float _pathValidationTolerance = 0.05f; // 路径验证容差

        // 路径置中参数
        private bool _enablePathCentering = true; // 是否启用路径置中
        private float _centeringStrength = 0.7f; // 置中强度 (0-1)
        private float _maxCenteringDistance = 1.0f; // 最大置中距离
        private float _randomOffsetRatio = 0.15f; // 随机偏移比例（相对于通道宽度）

        /// <summary>
        /// 创建路径随机化工具
        /// </summary>
        /// <param name="navQuery">DotRecast导航网格查询对象</param>
        /// <param name="agentRadius">代理半径</param>
        /// <param name="agentHeight">代理高度</param>
        public PathRandomizer(DtNavMeshQuery navQuery, float agentRadius = 0.5f, float agentHeight = 2.0f)
        {
            _navQuery = navQuery;
            _agentRadius = agentRadius;
            _agentHeight = agentHeight;
            _random = new Random();
        }

        /// <summary>
        /// 设置随机化参数
        /// </summary>
        /// <param name="minDistance">最小随机化距离</param>
        /// <param name="maxDeviationRatio">最大偏移比例</param>
        /// <param name="randomnessFactor">随机程度因子(0-1)</param>
        public void SetRandomizationParameters(float minDistance, float maxDeviationRatio, float randomnessFactor)
        {
            _minDistanceForRandomization = Math.Max(0.1f, minDistance);
            _maxDeviationRatio = Clamp(maxDeviationRatio, 0f, 1f);
            _randomnessFactor = Clamp(randomnessFactor, 0f, 1f);
        }

        /// <summary>
        /// 设置点生成参数
        /// </summary>
        /// <param name="minPoints">每段最少点数</param>
        /// <param name="maxPoints">每段最多点数</param>
        public void SetPointGenerationParameters(int minPoints, int maxPoints)
        {
            _minPointsPerSegment = Math.Max(1, minPoints);
            _maxPointsPerSegment = Math.Max(_minPointsPerSegment, maxPoints);
        }

        /// <summary>
        /// 设置障碍物检测参数
        /// </summary>
        /// <param name="checkSamples">障碍物检测采样点数</param>
        /// <param name="validationTolerance">路径验证容差</param>
        public void SetObstacleDetectionParameters(int checkSamples, float validationTolerance)
        {
            _obstacleCheckSamples = Math.Max(2, checkSamples);
            _pathValidationTolerance = Math.Max(0.01f, validationTolerance);
        }

        /// <summary>
        /// 设置路径置中参数
        /// </summary>
        /// <param name="enableCentering">是否启用置中</param>
        /// <param name="centeringStrength">置中强度(0-1)</param>
        /// <param name="maxCenteringDistance">最大置中距离</param>
        /// <param name="randomOffsetRatio">随机偏移比例(0-0.5)</param>
        public void SetPathCenteringParameters(bool enableCentering, float centeringStrength, float maxCenteringDistance, float randomOffsetRatio = 0.15f)
        {
            _enablePathCentering = enableCentering;
            _centeringStrength = Clamp(centeringStrength, 0f, 1f);
            _maxCenteringDistance = Math.Max(0.1f, maxCenteringDistance);
            _randomOffsetRatio = Clamp(randomOffsetRatio, 0f, 0.5f); // 限制在0-0.5之间，避免偏移过大
        }

        /// <summary>
        /// 对寻路路径进行随机化处理
        /// </summary>
        /// <param name="originalPath">原始路径点列表</param>
        /// <returns>随机化后的路径点列表</returns>
        public List<Vector3> RandomizePath(List<Vector3> originalPath, IDtQueryFilter filter)
        {
            if (originalPath.Count < 2)
                return originalPath;

            // 首先对原始路径进行置中处理（如果启用）
            List<Vector3> centeredPath = _enablePathCentering ?
                CenterPath(originalPath, filter) :
                [.. originalPath];

            /*Plugin.CenteredPath_Locked = true;
            Plugin.CenteredPath = centeredPath;
            Plugin.CenteredPath_Locked = false;*/

            List<Vector3> randomizedPath =
            [
                centeredPath[0], // 添加起始点
            ];

            for (int i = 0; i < centeredPath.Count - 1; i++)
            {
                Vector3 current = centeredPath[i];
                Vector3 next = centeredPath[i + 1];

                float distance = Vector3.Distance(current, next);

                // 如果两点距离太短，不进行随机化
                if (distance < _minDistanceForRandomization)
                {
                    randomizedPath.Add(next);
                    continue;
                }

                // 检查路径段是否有效
                bool pathSegmentValid = IsPathSegmentValid(current, next, filter);

                // 如果路径段无效，使用NavMesh查询生成有效路径
                if (!pathSegmentValid)
                {
                    List<Vector3> validSegment = GenerateValidPathSegment(current, next, filter);
                    if (validSegment.Count > 0)
                    {
                        randomizedPath.AddRange(validSegment);
                        continue;
                    }
                }

                // 根据距离确定插入点数量
                int numPoints = Clamp(
                    (int)(distance / 2.5f),
                    _minPointsPerSegment,
                    _maxPointsPerSegment
                );

                // 生成随机路径点
                List<Vector3> randomPoints = GenerateRandomPathSegment(current, next, numPoints, filter);

                // 添加有效的随机点，确保不穿过障碍物
                Vector3 lastPoint = randomizedPath[^1];
                foreach (var point in randomPoints)
                {
                    if (!IsPointObstructed(lastPoint, point, filter))
                    {
                        randomizedPath.Add(point);
                        lastPoint = point;
                    }
                }

                // 添加下一个原始点
                randomizedPath.Add(next);
            }

            // 验证并平滑路径
            return SmoothPath(randomizedPath, filter);
        }

        /// <summary>
        /// 对路径进行置中处理
        /// </summary>
        private List<Vector3> CenterPath(List<Vector3> path, IDtQueryFilter filter)
        {
            if (path.Count <= 2)
                return [.. path];

            List<Vector3> densePath = CreateDensePath(path, filter);

            /*Plugin.DensedPath_Locked = true;
            Plugin.DensedPath = densePath;
            Plugin.DensedPath_Locked = false;*/

            List<Vector3> centeredPath =
            [
                path[0] // 保留起点
            ];

            // 对原始路径的每个点进行置中
            for (int i = 1; i < path.Count - 1; i++)
            {
                // 查找原始点在密集路径中的最近索引
                int denseIndex = FindClosestPointIndex(path[i], densePath);

                // 获取密集路径中的前后点以提供更好的上下文
                Vector3 prev = denseIndex > 0 ? densePath[denseIndex - 1] : path[i - 1];
                Vector3 current = path[i];
                Vector3 next = denseIndex < densePath.Count - 1 ? densePath[denseIndex + 1] : path[i + 1];

                // 使用密集路径的上下文信息计算置中位置
                Vector3 centeredPoint = FindCenteredPosition(current, prev, next, filter);

                // 添加置中后的点
                centeredPath.Add(centeredPoint);
            }

            centeredPath.Add(path[^1]); // 保留原始终点
            return centeredPath;
        }

        /// <summary>
        /// 创建更密集的路径用于分析
        /// </summary>
        private List<Vector3> CreateDensePath(List<Vector3> path, IDtQueryFilter filter)
        {
            if (path.Count <= 2)
                return [.. path];

            List<Vector3> densePath = [path[0]];

            for (int i = 0; i < path.Count - 1; i++)
            {
                Vector3 current = path[i];
                Vector3 next = path[i + 1];
                float distance = Vector3.Distance(current, next);

                // 只有足够长的路径段才添加额外点
                if (distance > _minDistanceForRandomization)
                {
                    // 计算需要添加的点数
                    int pointsToAdd = Math.Max(1, (int)(distance / 2.0f));

                    for (int j = 1; j <= pointsToAdd; j++)
                    {
                        float t = j / (float)(pointsToAdd + 1);
                        Vector3 interpolatedPoint = Vector3.Lerp(current, next, t);

                        // 确保点在导航网格上
                        if (FindNearestValidPoint(interpolatedPoint, out Vector3 validPoint, filter))
                        {
                            densePath.Add(validPoint);
                        }
                    }
                }

                densePath.Add(next); // 添加原始点
            }

            return densePath;
        }

        /// <summary>
        /// 在密集路径中查找最接近给定点的索引
        /// </summary>
        private int FindClosestPointIndex(Vector3 point, List<Vector3> path)
        {
            int closestIndex = 0;
            float minDistance = float.MaxValue;

            for (int i = 0; i < path.Count; i++)
            {
                float distance = Vector3.Distance(point, path[i]);
                if (distance < minDistance)
                {
                    minDistance = distance;
                    closestIndex = i;
                }
            }

            return closestIndex;
        }

        /// <summary>
        /// 查找导航通道中心位置，并添加微小随机偏移
        /// </summary>
        private Vector3 FindCenteredPosition(Vector3 point, Vector3 prev, Vector3 next, IDtQueryFilter filter)
        {
            // 转换为dotRecast坐标
            RcVec3f position = point.SystemToRecast();
            RcVec3f extents = GetExtents();

            // 查找当前点所在的多边形
            DtStatus status = _navQuery.FindNearestPoly(position, extents, filter,
                out long polyRef, out RcVec3f nearestPt, out bool _);

            if (polyRef == 0 || !status.Succeeded())
                return point; // 如果找不到多边形，返回原点

            // 计算更准确的移动方向
            Vector3 dirToPrev = Vector3.Normalize(prev - point);
            Vector3 dirToNext = Vector3.Normalize(next - point);

            // 如果方向几乎相反，使用多方向探测
            float dotProduct = Vector3.Dot(dirToPrev, dirToNext);
            if (dotProduct < -0.7f) // 方向基本相反
            {
                return FindCenteredPositionMultiProbe(point, filter);
            }

            // 计算平均方向的垂直向量（用于探测通道宽度）
            Vector3 avgDir = Vector3.Normalize(dirToPrev + dirToNext);
            Vector3 perpendicular = Vector3.Normalize(new Vector3(-avgDir.Z, 0, avgDir.X));

            // 向两侧探测，寻找障碍物，使用多个探测角度以获得更准确的通道宽度
            float maxProbeDistance = _agentRadius * 10; // 增加探测距离
            float bestLeftDistance = 0;
            float bestRightDistance = 0;
            Vector3 bestDirection = perpendicular;

            // 使用更多探测角度以获得更准确的通道宽度
            int numProbes = 16; // 增加探测角度数量
            for (int i = 0; i < numProbes; i++)
            {
                // 在垂直方向周围创建多个探测向量
                float angle = -50.0f + (100.0f * i / (numProbes - 1)); // -30度到+30度
                float radians = angle * MathF.PI / 180.0f;
                Vector3 rotatedPerpendicular = RotateVectorAroundY(perpendicular, radians);

                // 向左探测
                float _leftDistance = ProbeForObstacle(point, rotatedPerpendicular, maxProbeDistance, filter);

                // 向右探测
                float _rightDistance = ProbeForObstacle(point, -rotatedPerpendicular, maxProbeDistance, filter);

                // 更新最佳值 - 选择最安全的通道（左右距离的最小值最大）
                float minDistance = Math.Min(_leftDistance, _rightDistance);
                if (i == 0 || minDistance > Math.Min(bestLeftDistance, bestRightDistance))
                {
                    bestLeftDistance = _leftDistance;
                    bestRightDistance = _rightDistance;
                    bestDirection = rotatedPerpendicular;
                }
            }

            // 使用最佳探测结果
            float leftDistance = bestLeftDistance;
            float rightDistance = bestRightDistance;
            perpendicular = bestDirection;

            // 计算通道中心位置
            float centerOffset = (rightDistance - leftDistance) * 0.5f;

            // 计算通道宽度
            float channelWidth = leftDistance + rightDistance;

            // 如果通道宽度小于代理直径的2倍，减少置中强度以避免碰撞
            float adjustedStrength = _centeringStrength;
            if (channelWidth < _agentRadius * 4)
            {
                adjustedStrength *= channelWidth / (_agentRadius * 4);
            }

            // 限制偏移量在最大置中距离范围内
            centerOffset = Clamp(centerOffset * adjustedStrength, -_maxCenteringDistance, _maxCenteringDistance);

            // 应用偏移获得中心位置
            Vector3 centeredPosition = point + perpendicular * centerOffset;

            // 确保置中后的点有效
            if (!IsPointStrictlyValid(centeredPosition, filter))
            {
                // 如果置中后的点无效，尝试找到一个有效的点
                for (float ratio = 0.8f; ratio >= 0; ratio -= 0.2f)
                {
                    Vector3 fallbackPosition = Vector3.Lerp(point, centeredPosition, ratio);
                    if (IsPointStrictlyValid(fallbackPosition, filter))
                    {
                        centeredPosition = fallbackPosition;
                        break;
                    }
                }
            }

            // 如果所有尝试都失败，使用原始点
            if (!IsPointStrictlyValid(centeredPosition, filter))
            {
                return point;
            }

            // 添加微小的随机偏移，避免路径总是在正中间
            // 计算安全的随机偏移范围
            float safeRandomDistance = Math.Min(
                Math.Min(leftDistance, rightDistance) * 0.4f, // 通道较窄一侧的40%
                _agentRadius * 0.5f // 不超过代理半径的一半
            );

            // 确保随机偏移不会导致点无效
            float randomOffset = RandomRange(-safeRandomDistance, safeRandomDistance);
            Vector3 finalPosition = centeredPosition + perpendicular * randomOffset;

            // 验证最终位置
            if (IsPointStrictlyValid(finalPosition, filter))
            {
                return finalPosition;
            }

            // 如果随机偏移后的点无效，返回中心点
            return centeredPosition;
        }

        /// <summary>
        /// 更严格地检查点是否有效（确保不在墙内）
        /// </summary>
        private bool IsPointStrictlyValid(Vector3 point, IDtQueryFilter filter)
        {
            // 首先使用基本检查
            if (!IsPointValid(point, filter))
                return false;

            // 转换为dotRecast坐标
            RcVec3f position = point.SystemToRecast();
            RcVec3f extents = GetExtents();

            // 查找最近的多边形
            DtStatus status = _navQuery.FindNearestPoly(position, extents, filter,
                out long nearestRef, out RcVec3f nearestPt, out bool _);

            if (nearestRef == 0 || !status.Succeeded())
                return false;

            // 检查找到的点与请求点的距离，如果距离过大，说明点可能在墙内
            float distance = Vector3.Distance(
                new Vector3(position.X, position.Y, position.Z),
                new Vector3(nearestPt.X, nearestPt.Y, nearestPt.Z)
            );

            // 如果距离超过代理半径的一小部分，认为点无效
            if (distance > _agentRadius * 0.2f)
                return false;

            // 额外检查：在点周围进行多方向探测，确保不靠近边缘
            int numChecks = 8;
            float minSafeDistance = _agentRadius * 0.8f; // 最小安全距离

            for (int i = 0; i < numChecks; i++)
            {
                float angle = (float)i / numChecks * MathF.PI * 2;
                Vector3 checkDir = new Vector3(MathF.Cos(angle), 0, MathF.Sin(angle));

                // 使用较短距离探测
                float probeDistance = ProbeForObstacle(point, checkDir, _agentRadius * 3, filter);

                // 如果任何方向距离过短，认为点太靠近边缘
                if (probeDistance < minSafeDistance)
                    return false;
            }

            return true;
        }

        /// <summary>
        /// 使用多方向探测查找中心位置（用于当前后点几乎在同一直线上的情况）
        /// </summary>
        private Vector3 FindCenteredPositionMultiProbe(Vector3 point, IDtQueryFilter filter)
        {
            float maxProbeDistance = _agentRadius * 10; // 增加探测距离
            int numDirections = 16; // 增加探测方向数量
            float bestMinDistance = 0; // 记录最佳的最小距离
            Vector3 bestDirection = Vector3.UnitX; // 初始默认方向
            float bestLeftDistance = 0;
            float bestRightDistance = 0;

            for (int i = 0; i < numDirections; i++)
            {
                // 均匀分布在圆周上
                float angle = (float)i / numDirections * MathF.PI * 2;
                Vector3 direction = new Vector3(MathF.Cos(angle), 0, MathF.Sin(angle));

                // 探测当前方向
                float distance1 = ProbeForObstacle(point, direction, maxProbeDistance, filter);

                // 探测反方向
                float distance2 = ProbeForObstacle(point, -direction, maxProbeDistance, filter);

                // 记录这个方向的最小距离
                float minDistance = Math.Min(distance1, distance2);

                // 更新最佳探测结果 - 选择最小距离最大的方向
                if (i == 0 || minDistance > bestMinDistance)
                {
                    bestMinDistance = minDistance;
                    bestDirection = direction;
                    bestLeftDistance = distance1;
                    bestRightDistance = distance2;
                }
            }

            // 计算通道中心位置
            float centerOffset = (bestRightDistance - bestLeftDistance) * 0.5f;

            // 计算通道宽度
            float channelWidth = bestLeftDistance + bestRightDistance;

            // 如果通道宽度小于代理直径的2倍，减少置中强度以避免碰撞
            float adjustedStrength = _centeringStrength;
            if (channelWidth < _agentRadius * 4)
            {
                adjustedStrength *= channelWidth / (_agentRadius * 4);
            }

            // 限制偏移量
            centerOffset = Clamp(centerOffset * adjustedStrength, -_maxCenteringDistance, _maxCenteringDistance);

            // 应用偏移
            Vector3 centeredPosition = point + bestDirection * centerOffset;

            // 确保置中后的点有效
            if (!IsPointStrictlyValid(centeredPosition, filter))
            {
                // 如果置中后的点无效，尝试找到一个有效的点
                for (float ratio = 0.8f; ratio >= 0; ratio -= 0.2f)
                {
                    Vector3 fallbackPosition = Vector3.Lerp(point, centeredPosition, ratio);
                    if (IsPointStrictlyValid(fallbackPosition, filter))
                    {
                        return fallbackPosition;
                    }
                }
                return point; // 如果所有尝试都失败，返回原始点
            }

            // 添加微小的随机偏移
            float safeRandomDistance = Math.Min(
                Math.Min(bestLeftDistance, bestRightDistance) * 0.4f,
                _agentRadius * 0.5f
            );

            float randomOffset = RandomRange(-safeRandomDistance, safeRandomDistance);
            Vector3 finalPosition = centeredPosition + bestDirection * randomOffset;

            // 验证最终位置
            if (IsPointStrictlyValid(finalPosition, filter))
            {
                return finalPosition;
            }

            return centeredPosition;
        }

        /// <summary>
        /// 围绕指定轴旋转向量
        /// </summary>
        private Vector3 RotateVectorAroundY(Vector3 vector, float radians)
        {
            float cosTheta = MathF.Cos(radians);
            float sinTheta = MathF.Sin(radians);

            // XZ平面内的旋转矩阵
            return new Vector3(
                vector.X * cosTheta - vector.Z * sinTheta,
                0, // Y保持为0
                vector.X * sinTheta + vector.Z * cosTheta
            );
        }

        /// <summary>
        /// 向指定方向探测障碍物，返回可行走的距离
        /// </summary>
        private float ProbeForObstacle(Vector3 start, Vector3 direction, float maxDistance, IDtQueryFilter filter)
        {
            // 转换为dotRecast坐标
            RcVec3f startPos = start.SystemToRecast();
            RcVec3f extents = GetExtents();

            // 查找起点所在的多边形
            DtStatus status = _navQuery.FindNearestPoly(startPos, extents, filter,
                out long startRef, out RcVec3f _, out bool _);

            if (startRef == 0 || !status.Succeeded())
                return 0; // 如果找不到多边形，返回0

            // 计算终点
            Vector3 end = start + direction * maxDistance;
            RcVec3f endPos = end.SystemToRecast();

            // 使用DotRecast的raycast查询
            List<long> path = [];
            _navQuery.Raycast(startRef, startPos, endPos, filter, out float t, out RcVec3f hitNormal, ref path);

            // 如果t接近1，可能是因为射线长度不够，尝试更长的射线
            if (t > 0.95f)
            {
                // 尝试更长的射线
                Vector3 farEnd = start + direction * maxDistance * 2;
                RcVec3f farEndPos = farEnd.SystemToRecast();

                List<long> farPath = [];
                _navQuery.Raycast(startRef, startPos, farEndPos, filter, out float farT, out RcVec3f _, ref farPath);

                // 如果更长的射线确实检测到障碍物，调整t值
                if (farT < 0.95f)
                {
                    t = farT / 2; // 调整为原始最大距离的比例
                }
            }

            // 使用采样点验证射线结果
            float distance = t * maxDistance;
            int numSamples = 5;
            float validDistance = distance;

            for (int i = 1; i <= numSamples; i++)
            {
                float sampleT = (float)i / (numSamples + 1);
                Vector3 samplePoint = start + direction * (distance * sampleT);

                if (!IsPointValid(samplePoint, filter))
                {
                    // 找到第一个无效点，更新有效距离
                    validDistance = distance * (sampleT - 1.0f / (numSamples + 1));
                    break;
                }
            }

            // 返回有效距离，并应用安全裕度
            return Math.Max(0, validDistance - _agentRadius * 0.8f); // 增加安全裕度
        }

        /// <summary>
        /// 生成有效的路径段（通过导航网格查询）
        /// </summary>
        private List<Vector3> GenerateValidPathSegment(Vector3 start, Vector3 end, IDtQueryFilter filter)
        {
            List<Vector3> result = new List<Vector3>();

            // 转换为dotRecast坐标
            RcVec3f startPos = start.SystemToRecast();
            RcVec3f endPos = end.SystemToRecast();
            RcVec3f extents = GetExtents();

            // 查找起点和终点的多边形
            DtStatus startStatus = _navQuery.FindNearestPoly(startPos, extents, filter,
                out long startRef, out RcVec3f _, out bool _);

            DtStatus endStatus = _navQuery.FindNearestPoly(endPos, extents, filter,
                out long endRef, out RcVec3f _, out bool _);

            if (startRef == 0 || endRef == 0 || !startStatus.Succeeded() || !endStatus.Succeeded())
                return result;

            // 使用dotRecast的FindPath查询路径
            List<long> path = [];
            _navQuery.FindPath(startRef, endRef, startPos, endPos, filter, ref path, DtFindPathOption.AnyAngle);

            if (path.Count == 0)
                return result;

            // 将路径转换为平滑的点列表
            List<DtStraightPath> straightPath = [];
            _navQuery.FindStraightPath(startPos, endPos, path, ref straightPath, 8, 0);

            // 转换为Vector3并返回
            result.AddRange([.. straightPath.Select(p => p.pos.RecastToSystem())]);

            return result;
        }

        /// <summary>
        /// 生成两点之间的随机路径段
        /// </summary>
        private List<Vector3> GenerateRandomPathSegment(Vector3 start, Vector3 end, int numPoints, IDtQueryFilter filter)
        {
            List<Vector3> points = new List<Vector3>();

            // 计算基本方向和距离
            Vector3 direction = Vector3.Normalize(end - start);
            float totalDistance = Vector3.Distance(start, end);

            // 找到垂直于路径方向的向量，用于偏移
            Vector3 up = Vector3.UnitY;
            Vector3 right = Vector3.Normalize(Vector3.Cross(direction, up));
            if (right.Length() < 0.01f)
            {
                right = Vector3.Normalize(Vector3.Cross(direction, Vector3.UnitX));
            }

            // 使用递归分割方法生成随机点
            List<Vector3> candidatePoints = new List<Vector3>();
            GenerateSubdividedPath(start, end, totalDistance * _maxDeviationRatio, numPoints, candidatePoints);

            // 验证所有点的有效性
            Vector3 lastValidPoint = start;
            foreach (var point in candidatePoints)
            {
                // 首先检查点是否在导航网格上
                if (IsPointValid(point, filter))
                {
                    // 然后检查从上一个有效点到当前点是否有障碍物
                    if (!IsPointObstructed(lastValidPoint, point, filter))
                    {
                        points.Add(point);
                        lastValidPoint = point;
                    }
                    else
                    {
                        // 如果有障碍物，尝试找到附近的无障碍点
                        if (FindNearestUnobstructedPoint(point, lastValidPoint, end, out Vector3 validPoint, filter))
                        {
                            points.Add(validPoint);
                            lastValidPoint = validPoint;
                        }
                    }
                }
                else
                {
                    // 如果点无效，尝试找到附近的有效点
                    if (FindNearestValidPoint(point, out Vector3 validPoint, filter) &&
                        !IsPointObstructed(lastValidPoint, validPoint, filter))
                    {
                        points.Add(validPoint);
                        lastValidPoint = validPoint;
                    }
                }
            }

            return points;
        }

        /// <summary>
        /// 使用递归分割方法生成自然的随机路径
        /// </summary>
        private void GenerateSubdividedPath(Vector3 start, Vector3 end, float maxOffset, int depth, List<Vector3> points)
        {
            if (depth <= 0)
                return;

            // 计算中点
            Vector3 midPoint = (start + end) * 0.5f;

            // 计算垂直于路径方向的偏移向量
            Vector3 direction = Vector3.Normalize(end - start);
            Vector3 perpendicular;

            // 找到垂直于路径方向的向量
            if (Math.Abs(direction.Y) > 0.9f)
            {
                perpendicular = new Vector3(1, 0, 0);
            }
            else
            {
                perpendicular = Vector3.Normalize(new Vector3(-direction.Z, 0, direction.X));
            }

            // 添加随机偏移
            float offset = RandomRange(-maxOffset, maxOffset) * _randomnessFactor;

            // 确保偏移不为0（避免直线）
            if (Math.Abs(offset) < 0.1f)
            {
                offset = offset < 0 ? -0.1f : 0.1f;
            }

            midPoint += perpendicular * offset;

            // 递归生成左半部分和右半部分
            if (depth > 1)
            {
                GenerateSubdividedPath(start, midPoint, maxOffset * 0.6f, depth / 2, points);
                points.Add(midPoint);
                GenerateSubdividedPath(midPoint, end, maxOffset * 0.6f, depth / 2, points);
            }
            else
            {
                points.Add(midPoint);
            }
        }

        /// <summary>
        /// 检查点是否有效（不穿过障碍物）
        /// </summary>
        private bool IsPointValid(Vector3 point, IDtQueryFilter filter)
        {
            // 转换为dotRecast坐标
            RcVec3f position = point.SystemToRecast();
            RcVec3f extents = GetExtents();

            // 检查点是否在导航网格上
            DtStatus status = _navQuery.FindNearestPoly(position, extents, filter,
                out long nearestRef, out RcVec3f _, out bool _);

            return nearestRef != 0 && status.Succeeded();
        }

        /// <summary>
        /// 检查路径段是否有效
        /// </summary>
        private bool IsPathSegmentValid(Vector3 start, Vector3 end, IDtQueryFilter filter)
        {
            // 转换为dotRecast坐标
            RcVec3f startPos = start.SystemToRecast();
            RcVec3f endPos = end.SystemToRecast();
            RcVec3f extents = GetExtents();

            // 查找起点和终点的多边形
            DtStatus startStatus = _navQuery.FindNearestPoly(startPos, extents, filter,
                out long startRef, out RcVec3f _, out bool _);

            DtStatus endStatus = _navQuery.FindNearestPoly(endPos, extents, filter,
                out long endRef, out RcVec3f _, out bool _);

            if (startRef == 0 || endRef == 0 || !startStatus.Succeeded() || !endStatus.Succeeded())
                return false;

            // 检查是否可以直接连接
            List<long> path = [];
            _navQuery.Raycast(startRef, startPos, endPos, filter, out float t, out RcVec3f _, ref path);

            // 如果t接近1，表示射线几乎没有被阻挡
            return t >= (1.0f - _pathValidationTolerance);
        }

        /// <summary>
        /// 检查点是否被障碍物阻挡
        /// </summary>
        private bool IsPointObstructed(Vector3 from, Vector3 to, IDtQueryFilter filter)
        {
            // 转换为dotRecast坐标
            RcVec3f fromPos = from.SystemToRecast();
            RcVec3f toPos = to.SystemToRecast();
            RcVec3f extents = GetExtents();

            // 查找起点的多边形
            DtStatus status = _navQuery.FindNearestPoly(fromPos, extents, filter,
                out long fromRef, out RcVec3f _, out bool _);

            if (fromRef == 0 || !status.Succeeded())
                return true; // 如果起点无效，认为有障碍

            // 使用多个采样点进行障碍检测
            Vector3 dir = to - from;
            float distance = dir.Length();

            if (distance < 0.001f)
                return false; // 距离太近，认为没有障碍

            dir = Vector3.Normalize(dir);

            // 沿路径进行采样检测
            for (int i = 1; i <= _obstacleCheckSamples; i++)
            {
                float t = (float)i / (_obstacleCheckSamples + 1);
                Vector3 samplePoint = from + dir * (distance * t);

                // 检查采样点是否在导航网格上
                if (!IsPointValid(samplePoint, filter))
                {
                    // 尝试找到附近的有效点
                    if (!FindNearestValidPoint(samplePoint, out Vector3 validPoint, filter) ||
                        Vector3.Distance(samplePoint, validPoint) > _agentRadius * 0.5f)
                    {
                        return true; // 找不到附近有效点或偏差太大，认为有障碍
                    }
                }
            }

            // 使用raycast作为最后的检查
            List<long> path = [];
            _navQuery.Raycast(fromRef, fromPos, toPos, filter, out float t2, out RcVec3f _, ref path);

            // 如果t2小于1-容差，表示射线被阻挡
            return t2 < (1.0f - _pathValidationTolerance);
        }

        /// <summary>
        /// 找到最近的无障碍点
        /// </summary>
        private bool FindNearestUnobstructedPoint(Vector3 point, Vector3 from, Vector3 to, out Vector3 validPoint, IDtQueryFilter filter)
        {
            // 先尝试找到最近的有效点
            if (!FindNearestValidPoint(point, out validPoint, filter))
                return false;

            // 检查这个点是否无障碍
            if (!IsPointObstructed(from, validPoint, filter) && !IsPointObstructed(validPoint, to, filter))
                return true;

            // 如果有障碍，尝试在周围搜索
            float searchRadius = _agentRadius * 2;
            int searchAttempts = 8; // 尝试8个方向

            for (int i = 0; i < searchAttempts; i++)
            {
                float angle = (float)i / searchAttempts * MathF.PI * 2;
                Vector3 offset = new Vector3(
                    MathF.Cos(angle) * searchRadius,
                    0,
                    MathF.Sin(angle) * searchRadius
                );

                Vector3 testPoint = point + offset;

                if (FindNearestValidPoint(testPoint, out Vector3 nearPoint, filter) &&
                    !IsPointObstructed(from, nearPoint, filter) &&
                    !IsPointObstructed(nearPoint, to, filter))
                {
                    validPoint = nearPoint;
                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// 找到最近的有效导航点
        /// </summary>
        private bool FindNearestValidPoint(Vector3 point, out Vector3 validPoint, IDtQueryFilter filter)
        {
            // 转换为dotRecast坐标
            RcVec3f position = point.SystemToRecast();

            // 搜索范围可以适当调大
            RcVec3f extents = new RcVec3f(_agentRadius * 2, _agentHeight * 0.5f, _agentRadius * 2);

            // 查找最近的有效多边形
            DtStatus status = _navQuery.FindNearestPoly(position, extents, filter,
                out long nearestRef, out RcVec3f nearestPt, out bool _);

            if (nearestRef != 0 && status.Succeeded())
            {
                validPoint = new Vector3(nearestPt.X, nearestPt.Y, nearestPt.Z);
                return true;
            }

            validPoint = point;
            return false;
        }

        /// <summary>
        /// 平滑路径，使其更自然
        /// </summary>
        private List<Vector3> SmoothPath(List<Vector3> path, IDtQueryFilter filter)
        {
            if (path.Count <= 2)
                return path;

            // 使用移动平均法平滑路径
            List<Vector3> smoothedPath = new List<Vector3>();
            smoothedPath.Add(path[0]); // 保留起点

            for (int i = 1; i < path.Count - 1; i++)
            {
                Vector3 prev = path[i - 1];
                Vector3 current = path[i];
                Vector3 next = path[i + 1];

                // 简单的移动平均
                Vector3 smoothed = (prev + current + next) / 3.0f;

                // 确保平滑后的点仍然有效且无障碍
                if (IsPointValid(smoothed, filter) &&
                    !IsPointObstructed(smoothedPath[^1], smoothed, filter) &&
                    !IsPointObstructed(smoothed, next, filter))
                {
                    smoothedPath.Add(smoothed);
                }
                else
                {
                    // 如果平滑后的点无效或有障碍，使用原始点
                    smoothedPath.Add(current);
                }
            }

            smoothedPath.Add(path[^1]); // 保留终点
            return smoothedPath;
        }

        /// <summary>
        /// 获取代理的搜索范围
        /// </summary>
        private RcVec3f GetExtents()
        {
            return new RcVec3f(_agentRadius, _agentHeight * 0.5f, _agentRadius);
        }

        #region 辅助方法

        /// <summary>
        /// 限制值在指定范围内
        /// </summary>
        private T Clamp<T>(T value, T min, T max) where T : IComparable<T>
        {
            if (value.CompareTo(min) < 0) return min;
            if (value.CompareTo(max) > 0) return max;
            return value;
        }

        /// <summary>
        /// 生成指定范围内的随机浮点数
        /// </summary>
        private float RandomRange(float min, float max)
        {
            return (float)(_random.NextDouble() * (max - min) + min);
        }

        #endregion
    }
}
