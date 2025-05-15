using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading;
using System.Threading.Tasks;
using Navmesh.Movement;

namespace Navmesh;

/// <summary>
///     管理异步寻路请求的类，处理路径计算和跟随逻辑
/// </summary>
public class AsyncMoveRequest : IDisposable
{
    #region 私有字段

    private readonly NavmeshManager NavmeshManager;
    private readonly FollowPath     FollowPath;

    private Task<List<Vector3>>? PendingTask;
    private bool                 PendingFly;
    private int                  RecalculationAttempts;
    
    private CancellationTokenSource? currentCTS;

    private const int MaxRecalculationAttempts = 5;

    #endregion

    #region 公共属性

    /// <summary>
    ///     是否有任务（进行中或已完成）
    /// </summary>
    public bool TaskInProgress => PendingTask != null;

    /// <summary>
    ///     是否有正在进行但尚未完成的任务
    /// </summary>
    public bool TaskInBusy => PendingTask is { IsCompleted: false };

    #endregion

    #region 构造和释放

    /// <summary>
    ///     初始化异步移动请求处理器
    /// </summary>
    /// <param name="manager">导航网格管理器</param>
    /// <param name="follow">路径跟随器</param>
    public AsyncMoveRequest(NavmeshManager manager, FollowPath follow)
    {
        NavmeshManager = manager;
        FollowPath     = follow;

        FollowPath.RequestPathRecalculation += OnRequestPathRecalculation;
    }

    /// <summary>
    ///     释放资源，取消订阅事件，并等待任何进行中的任务完成
    /// </summary>
    public void Dispose()
    {
        FollowPath.RequestPathRecalculation -= OnRequestPathRecalculation;

        if (PendingTask != null)
        {
            if (!PendingTask.IsCompleted)
                PendingTask.Wait();

            PendingTask.Dispose();
            PendingTask = null;
        }
    }

    #endregion

    #region 公共方法

    /// <summary>
    ///     更新异步寻路状态，检查并处理已完成的路径计算
    /// </summary>
    public void Update()
    {
        if (PendingTask is { IsCompleted: true })
        {
            Service.Log.Information("寻路完成");

            try
            {
                if (PendingTask.Result.Count <= 1)
                {
                    Service.Log.Warning("计算的路径无效或过短");
                    if (RecalculationAttempts >= MaxRecalculationAttempts)
                    {
                        Service.Log.Error($"多次路径重新计算失败，停止尝试 (尝试次数: {RecalculationAttempts})");
                        RecalculationAttempts = 0;
                    }
                }
                else
                {
                    RecalculationAttempts = 0;
                    FollowPath.Move(PendingTask.Result, !PendingFly);
                }
            }
            catch (Exception ex)
            {
                Plugin.DuoLog(ex, "寻路失败");
            }

            PendingTask.Dispose();
            PendingTask = null;
        }
    }

    /// <summary>
    ///     发起移动到指定目标位置的请求
    /// </summary>
    /// <param name="dest">目标位置</param>
    /// <param name="fly">是否允许飞行</param>
    /// <returns>请求是否成功发起</returns>
    public bool MoveTo(Vector3 dest, bool fly)
    {
        if (PendingTask != null) return false;

        Service.Log.Info($"准备 {(fly ? "飞行" : "步行")} 至 {dest:f3}");

        currentCTS  = new();
        PendingTask = NavmeshManager.QueryPath(Service.ClientState.LocalPlayer?.Position ?? default, dest, fly, currentCTS.Token);
        PendingFly  = fly;

        return true;
    }

    /// <summary>
    ///     取消当前正在进行的寻路计算
    /// </summary>
    /// <returns>是否成功取消了一个正在进行的寻路任务</returns>
    public bool CancelPathfinding()
    {
        if (!TaskInProgress)
            return false;

        Service.Log.Information("取消当前寻路任务");

        FollowPath.Stop();
        if (PendingTask != null)
        {
            var task = PendingTask;
            PendingTask = null;

            if (task.IsCompleted)
            {
                try
                {
                    task.Dispose();
                }
                catch (Exception ex)
                {
                    Service.Log.Error($"清理寻路任务时发生错误: {ex.Message}");
                }
            }
            else
            {
                Service.Log.Warning("丢弃了未完成的寻路任务");
                
                currentCTS?.Cancel();
                currentCTS?.Dispose();
                currentCTS = null;
            }
        }

        RecalculationAttempts = 0;
        PendingFly            = false;

        return true;
    }

    #endregion

    #region 私有方法

    /// <summary>
    ///     处理路径重新计算请求的回调方法
    /// </summary>
    /// <param name="currentPos">当前位置</param>
    /// <param name="targetPos">目标位置</param>
    /// <param name="ignoreDeltaY">是否忽略Y轴差异（地面行走）</param>
    private void OnRequestPathRecalculation(Vector3 currentPos, Vector3 targetPos, bool ignoreDeltaY)
    {
        if (TaskInBusy)
        {
            Service.Log.Warning("已有正在计算的路径任务，忽略重新计算请求");
            return;
        }

        RecalculationAttempts++;

        if (RecalculationAttempts > MaxRecalculationAttempts)
            Service.Log.Warning($"多次路径重新计算尝试 (尝试次数: {RecalculationAttempts})");

        Service.Log.Info($"开始重新计算路径 (第{RecalculationAttempts}次尝试): 从 {currentPos:f2} 到 {targetPos:f2}, {(ignoreDeltaY ? "地面行走" : "立体行走/飞行")}");

        var fly = !ignoreDeltaY;

        currentCTS?.Cancel();
        currentCTS = new();
        
        PendingTask = NavmeshManager.QueryPath(currentPos, targetPos, fly, currentCTS.Token);
        PendingFly  = fly;
    }

    #endregion
}
