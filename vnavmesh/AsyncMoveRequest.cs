using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading.Tasks;
using Navmesh.Movement;

namespace Navmesh;

public class AsyncMoveRequest : IDisposable
{
    private readonly NavmeshManager NavmeshManager;
    private readonly FollowPath     FollowPath;
    
    private Task<List<Vector3>>? PendingTask;
    private bool                 PendingFly;
    private int                  RecalculationAttempts;
    
    private const int MaxRecalculationAttempts = 5;

    public bool TaskInProgress => PendingTask != null;
    public bool TaskInBusy     => PendingTask is { IsCompleted: false };

    public AsyncMoveRequest(NavmeshManager manager, FollowPath follow)
    {
        NavmeshManager = manager;
        FollowPath     = follow;

        FollowPath.RequestPathRecalculation += OnRequestPathRecalculation;
    }

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

    public bool MoveTo(Vector3 dest, bool fly)
    {
        if (PendingTask != null)
        {
            Service.Log.Error("正在寻路中, 无法发起新的寻路请求...");
            return false;
        }

        Service.Log.Info($"准备 {(fly ? "飞行" : "步行")} 至 {dest:f3}");
        
        PendingTask = NavmeshManager.QueryPath(Service.ClientState.LocalPlayer?.Position ?? default, dest, fly);
        PendingFly  = fly;
        
        return true;
    }

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
        PendingTask = NavmeshManager.QueryPath(currentPos, targetPos, fly);
        PendingFly  = fly;
    }
}
