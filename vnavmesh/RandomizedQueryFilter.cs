using DotRecast.Core.Numerics;
using DotRecast.Detour;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Navmesh
{
    public class RandomizedQueryFilter : IDtQueryFilter
    {
        private readonly float[] m_areaCost = new float[DtNavMesh.DT_MAX_AREAS];
        private int m_includeFlags;
        private int m_excludeFlags;

        // 使用固定大小的预计算随机值表
        private readonly float[] _randomTable;
        private const int TABLE_SIZE = 1024;

        // 使用固定种子，确保结果可重现
        private readonly int _seed;

        // 随机因子
        private float _randomFactor;

        public RandomizedQueryFilter(float randomFactor = 0.2f)
        {
            _randomFactor = Math.Clamp(randomFactor, 0f, 1f);
            _seed = Environment.TickCount;

            // 初始化区域成本和标志
            m_includeFlags = 0xffff;
            m_excludeFlags = 0;
            for (int i = 0; i < DtNavMesh.DT_MAX_AREAS; ++i)
            {
                m_areaCost[i] = 1.0f;
            }

            // 预计算随机值表
            _randomTable = new float[TABLE_SIZE];
            var random = new Random(_seed);
            for (int i = 0; i < TABLE_SIZE; i++)
            {
                // 生成范围在[1, 1+randomFactor]的随机值
                _randomTable[i] = 1.0f + (float)random.NextDouble() * _randomFactor;
            }
        }

        public RandomizedQueryFilter(int includeFlags, int excludeFlags, float[] areaCost, float randomFactor = 0.2f)
            : this(randomFactor)
        {
            m_includeFlags = includeFlags;
            m_excludeFlags = excludeFlags;

            for (int i = 0; i < Math.Min(DtNavMesh.DT_MAX_AREAS, areaCost.Length); ++i)
            {
                m_areaCost[i] = areaCost[i];
            }
        }

        public void RebuildRandomTable()
        {
            // 重新生成随机值表
            var random = new Random(_seed);
            for (int i = 0; i < TABLE_SIZE; i++)
            {
                // 生成范围在[1, 1+randomFactor]的随机值
                _randomTable[i] = 1.0f + (float)random.NextDouble() * _randomFactor;
            }
        }

        public bool PassFilter(long refs, DtMeshTile tile, DtPoly poly)
        {
            return (poly.flags & m_includeFlags) != 0 && (poly.flags & m_excludeFlags) == 0;
        }

        public float GetCost(RcVec3f pa, RcVec3f pb,
                            long prevRef, DtMeshTile prevTile, DtPoly prevPoly,
                            long curRef, DtMeshTile curTile, DtPoly curPoly,
                            long nextRef, DtMeshTile nextTile, DtPoly nextPoly)
        {
            // 计算基础成本
            float baseCost = RcVec3f.Distance(pa, pb) * m_areaCost[curPoly.GetArea()];

            // 使用简单的哈希获取随机表索引
            int hash = GetStableHash(curRef, nextRef);
            int index = hash & (TABLE_SIZE - 1);

            // 应用随机因子
            return baseCost * _randomTable[index];
        }

        /// <summary>
        /// 获取稳定的哈希值，确保相同的输入总是产生相同的输出
        /// </summary>
        private int GetStableHash(long a, long b)
        {
            // 简单但有效的哈希组合方法
            unchecked
            {
                int hash = 17;
                hash = hash * 31 + (int)(a & 0xFFFFFFFF);
                hash = hash * 31 + (int)(a >> 32);
                hash = hash * 31 + (int)(b & 0xFFFFFFFF);
                hash = hash * 31 + (int)(b >> 32);
                return hash;
            }
        }

        public float GetAreaCost(int area)
        {
            if (area >= 0 && area < DtNavMesh.DT_MAX_AREAS)
                return m_areaCost[area];
            return 0;
        }

        public void SetAreaCost(int area, float cost)
        {
            if (area >= 0 && area < DtNavMesh.DT_MAX_AREAS)
                m_areaCost[area] = cost;
        }

        public int GetIncludeFlags()
        {
            return m_includeFlags;
        }

        public void SetIncludeFlags(int flags)
        {
            m_includeFlags = flags;
        }

        public int GetExcludeFlags()
        {
            return m_excludeFlags;
        }

        public void SetExcludeFlags(int flags)
        {
            m_excludeFlags = flags;
        }
    }
}
