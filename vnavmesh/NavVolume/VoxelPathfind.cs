using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Threading;

namespace Navmesh.NavVolume;

public class VoxelPathfind(VoxelMap volume)
{
    private readonly Random _random       = new();
    private const    float  _randomFactor = 0.5f;

    private readonly List<Node>             _nodes      = [];    // grow only (TODO: consider chunked vector)
    private readonly Dictionary<ulong, int> _nodeLookup = new(); // voxel -> node index
    private readonly List<int>              _openList   = [];    // heap containing node indices
    private          int                    _bestNodeIndex;
    private          ulong                  _goalVoxel;
    private          Vector3                _goalPos;
    private          bool                   _useRaycast;
    
    private const bool  _allowReopen    = false; // this is extremely expensive and doesn't seem to actually improve the result
    private const float _raycastLimitSq = float.MaxValue;

    public VoxelMap Volume { get; } = volume;

    public Span<Node> NodeSpan => CollectionsMarshal.AsSpan(_nodes);

    public List<(ulong voxel, Vector3 p)> FindPath(
        ulong fromVoxel,                ulong toVoxel, Vector3 fromPos, Vector3 toPos, bool useRaycast, bool returnIntermediatePoints, CancellationToken cancel)
    {
        _useRaycast = useRaycast;

        Start(fromVoxel, toVoxel, fromPos, toPos);
        Execute(cancel);

        var path = BuildPathToVisitedNode(_bestNodeIndex, returnIntermediatePoints);

        // 随机性的后处理
        if (path.Count > 2) 
            ApplyVoxelPathRandomization(path);

        return path;
    }

    public void Start(ulong fromVoxel, ulong toVoxel, Vector3 fromPos, Vector3 toPos)
    {
        _nodes.Clear();
        _nodeLookup.Clear();
        _openList.Clear();
        _bestNodeIndex = 0;
        if (fromVoxel == VoxelMap.InvalidVoxel || toVoxel == VoxelMap.InvalidVoxel)
        {
            Service.Log.Error($"无效的输入单元格: {fromVoxel:X} -> {toVoxel:X}");
            return;
        }

        _goalVoxel = toVoxel;
        _goalPos   = toPos;

        var randomFactor = GenerateRandomFactor();
        _nodes.Add(new()
        {
            HScore        = HeuristicDistance(fromVoxel, fromPos),
            Voxel         = fromVoxel,
            ParentIndex   = 0,
            OpenHeapIndex = -1,
            Position      = fromPos,
            RandomFactor  = randomFactor
        });

        _nodeLookup[fromVoxel] = 0;
        AddToOpen(0);
    }

    private float GenerateRandomFactor() => 
        (((float)_random.NextDouble() * 2) - 1) * _randomFactor;

    // 添加体素路径随机化后处理方法
    private void ApplyVoxelPathRandomization(List<(ulong voxel, Vector3 p)> path)
    {
        // 体素寻路已经在A*过程中添加了一些随机性，这里添加额外的路径扰动
        // 只处理中间点，保持起点和终点不变
        for (var i = 1; i < path.Count - 1; i++)
        {
            // 计算在路径中的相对位置
            var progressAlongPath = (float)i / (path.Count - 1);

            // 在路径中间区域应用最大随机性，接近两端应用较小随机性
            var positionFactor = MathF.Sin(progressAlongPath * MathF.PI);

            // 计算随机因子，体素路径的随机化强度比网格路径小一些，以保持稳定性
            var localRandomFactor = _randomFactor * 0.7f * positionFactor;

            // 获取当前点位置
            var currPos = path[i].p;

            // 计算与相邻点的方向，确定合理的随机化方向
            var prevPos = i > 0 ? path[i              - 1].p : currPos;
            var nextPos = i < path.Count - 1 ? path[i + 1].p : currPos;

            // 计算路径方向
            var     dirToPrev = Vector3.Normalize(prevPos - currPos);
            var     dirToNext = Vector3.Normalize(nextPos - currPos);
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

            // 生成随机偏移量 - 增加随机性强度
            var randomH = (((float)_random.NextDouble() * 2) - 1) * localRandomFactor * 1.5f;
            var randomV = (((float)_random.NextDouble() * 2) - 1) * localRandomFactor * 0.4f; // 垂直方向随机性更小

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
                    var fallbackPos   = currPos + smallerOffset;

                    if (IsValidPosition(fallbackPos))
                        path[i] = (path[i].voxel, fallbackPos);
                }
            }
        }

        // 平滑路径，确保路径连贯
        SmoothVoxelPath(path);
    }

    // 检查位置是否有效
    private bool IsValidPosition(Vector3 pos) => 
        VoxelSearch.FindNearestEmptyVoxel(Volume, pos, new Vector3(1.0f, 1.0f, 1.0f)) != VoxelMap.InvalidVoxel;

    // 平滑体素路径
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
            var smoothed = (originalPoints[i]     * 0.7f)  +
                           (originalPoints[i - 1] * 0.15f) +
                           (originalPoints[i + 1] * 0.15f);

            // 确保平滑后的位置有效
            if (IsValidPosition(smoothed)) path[i] = (path[i].voxel, smoothed);
        }
    }

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

    // returns whether search is to be terminated; on success, first node of the open list would contain found goal
    public bool ExecuteStep()
    {
        var nodeSpan = NodeSpan;
        if (_openList.Count == 0 || nodeSpan[_bestNodeIndex].HScore <= 0)
            return false;

        var     curNodeIndex = PopMinOpen();
        ref var curNode      = ref nodeSpan[curNodeIndex];

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

    private List<(ulong voxel, Vector3 p)> BuildPathToVisitedNode(int nodeIndex, bool returnIntermediatePoints)
    {
        var res = new List<(ulong voxel, Vector3 p)>();
        if (nodeIndex < _nodes.Count)
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

    private void VisitNeighbour(int parentIndex, ulong nodeVoxel)
    {
        var nodeIndex = _nodeLookup.GetValueOrDefault(nodeVoxel, -1);
        if (nodeIndex < 0)
        {
            // first time we're visiting this node, calculate heuristic
            nodeIndex = _nodes.Count;

            var randomFactor = GenerateRandomFactor();

            _nodes.Add(new()
            {
                GScore        = float.MaxValue,
                HScore        = float.MaxValue,
                Voxel         = nodeVoxel,
                ParentIndex   = parentIndex,
                OpenHeapIndex = -1,
                RandomFactor  = randomFactor
            });

            _nodeLookup[nodeVoxel] = nodeIndex;
        }
        else if (!_allowReopen && _nodes[nodeIndex].OpenHeapIndex < 0)
        {
            // in closed list already - TODO: is it possible to visit again with lower cost?..
            return;
        }

        var     nodeSpan   = NodeSpan;
        ref var parentNode = ref nodeSpan[parentIndex];
        var     enterPos   = nodeVoxel == _goalVoxel ? _goalPos : VoxelSearch.FindClosestVoxelPoint(Volume, nodeVoxel, parentNode.Position);
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

            if (curNode.HScore < _nodes[_bestNodeIndex].HScore)
                _bestNodeIndex = nodeIndex;
        }
    }

    private float CalculateGScore(ref Node parent, ulong destVoxel, Vector3 destPos, ref int parentIndex)
    {
        if (_useRaycast)
        {
            // check LoS from grandparent
            var     grandParentIndex = parent.ParentIndex;
            ref var grandParentNode  = ref NodeSpan[grandParentIndex];
            // TODO: invert LoS check to match path reconstruction step?
            var dist = (grandParentNode.Position - destPos).LengthSquared();
            if (dist <= _raycastLimitSq && VoxelSearch.LineOfSight(Volume, grandParentNode.Voxel, destVoxel, grandParentNode.Position, destPos))
            {
                parentIndex = grandParentIndex;
                return grandParentNode.GScore + MathF.Sqrt(dist);
            }
        }

        var distance = (parent.Position - destPos).Length();

        ref var node            = ref NodeSpan[_nodeLookup[destVoxel]];
        var     randomInfluence = 1.0f + node.RandomFactor;
        distance *= randomInfluence;

        return parent.GScore + distance;
    }

    private float HeuristicDistance(ulong nodeVoxel, Vector3 v)
    {
        if (nodeVoxel == _goalVoxel)
            return 0;

        var baseDistance = (v - _goalPos).Length() * 0.999f;

        if (nodeVoxel != _goalVoxel)
        {
            var nodeIndex = _nodeLookup.GetValueOrDefault(nodeVoxel, -1);
            if (nodeIndex >= 0)
            {
                // 使用已存储的随机因子，但动态调整影响程度
                // 距离目标越近，随机因子影响越小，确保能收敛到目标
                var targetDist      = (v - _goalPos).Length();
                var targetInfluence = Math.Min(1.0f, targetDist / 10.0f); // 距离目标 10 单位以内，随机因子影响逐渐减小

                // 增加随机因子影响，提高随机性
                var randomInfluence = 1.0f + (NodeSpan[nodeIndex].RandomFactor * 0.7f * targetInfluence);
                return baseDistance * randomInfluence;
            }
        }

        return baseDistance;
    }

    private void AddToOpen(int nodeIndex)
    {
        ref var node = ref NodeSpan[nodeIndex];
        if (node.OpenHeapIndex < 0)
        {
            node.OpenHeapIndex = _openList.Count;
            _openList.Add(nodeIndex);
        }

        // update location
        PercolateUp(node.OpenHeapIndex);
    }

    // remove first (minimal) node from open heap and mark as closed
    private int PopMinOpen()
    {
        var nodeSpan  = NodeSpan;
        var nodeIndex = _openList[0];
        _openList[0] = _openList[_openList.Count - 1];
        _openList.RemoveAt(_openList.Count - 1);
        nodeSpan[nodeIndex].OpenHeapIndex = -1;
        if (_openList.Count > 0)
        {
            nodeSpan[_openList[0]].OpenHeapIndex = 0;
            PercolateDown(0);
        }

        return nodeIndex;
    }

    private void PercolateUp(int heapIndex)
    {
        var nodeSpan  = NodeSpan;
        var nodeIndex = _openList[heapIndex];
        var parent    = (heapIndex - 1) >> 1;
        while (heapIndex > 0 && HeapLess(ref nodeSpan[nodeIndex], ref nodeSpan[_openList[parent]]))
        {
            _openList[heapIndex]                         = _openList[parent];
            nodeSpan[_openList[heapIndex]].OpenHeapIndex = heapIndex;
            heapIndex                                    = parent;
            parent                                       = (heapIndex - 1) >> 1;
        }

        _openList[heapIndex]              = nodeIndex;
        nodeSpan[nodeIndex].OpenHeapIndex = heapIndex;
    }

    private void PercolateDown(int heapIndex)
    {
        var nodeSpan  = NodeSpan;
        var nodeIndex = _openList[heapIndex];
        var maxSize   = _openList.Count;
        while (true)
        {
            var child1 = (heapIndex << 1) + 1;
            if (child1 >= maxSize)
                break;
            var child2 = child1 + 1;
            if (child2 == maxSize || HeapLess(ref nodeSpan[_openList[child1]], ref nodeSpan[_openList[child2]]))
            {
                if (HeapLess(ref nodeSpan[_openList[child1]], ref nodeSpan[nodeIndex]))
                {
                    _openList[heapIndex]                         = _openList[child1];
                    nodeSpan[_openList[heapIndex]].OpenHeapIndex = heapIndex;
                    heapIndex                                    = child1;
                }
                else
                    break;
            }
            else if (HeapLess(ref nodeSpan[_openList[child2]], ref nodeSpan[nodeIndex]))
            {
                _openList[heapIndex]                         = _openList[child2];
                nodeSpan[_openList[heapIndex]].OpenHeapIndex = heapIndex;
                heapIndex                                    = child2;
            }
            else
                break;
        }

        _openList[heapIndex]              = nodeIndex;
        nodeSpan[nodeIndex].OpenHeapIndex = heapIndex;
    }

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
        return nodeL.GScore > nodeR.GScore; // tie-break towards larger g-values
    }
    
    public struct Node
    {
        public float   GScore;
        public float   HScore;
        public ulong   Voxel;         // voxel map index corresponding to this node
        public int     ParentIndex;   // index in the node list of the node we entered from
        public int     OpenHeapIndex; // -1 if in closed list, otherwise index in open list
        public Vector3 Position;
        public float   RandomFactor;
    }
}
