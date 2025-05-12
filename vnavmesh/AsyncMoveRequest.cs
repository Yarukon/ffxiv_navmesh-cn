using Navmesh.Movement;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading.Tasks;

namespace Navmesh;

public class AsyncMoveRequest(NavmeshManager manager, FollowPath follow) : IDisposable
{
    private Task<List<Vector3>>? _pendingTask;
    private bool                 _pendingFly;

    public bool TaskInProgress => _pendingTask != null;

    public void Dispose()
    {
        if (_pendingTask != null)
        {
            if (!_pendingTask.IsCompleted)
                _pendingTask.Wait();
            _pendingTask.Dispose();
            _pendingTask = null;
        }
    }

    public void Update()
    {
        if (_pendingTask is { IsCompleted: true })
        {
            Service.Log.Information("寻路完成");
            try
            {
                follow.Move(_pendingTask.Result, !_pendingFly);
            }
            catch (Exception ex)
            {
                Plugin.DuoLog(ex, "寻路失败");
            }
            _pendingTask.Dispose();
            _pendingTask = null;
        }
    }

    public bool MoveTo(Vector3 dest, bool fly)
    {
        if (_pendingTask != null)
        {
            Service.Log.Error("正在寻路中...");
            return false;
        }

        Service.Log.Info($"准备 {(fly ? "飞行" : "步行")} 至 {dest:f3}");
        _pendingTask = manager.QueryPath(Service.ClientState.LocalPlayer?.Position ?? default, dest, fly);
        _pendingFly = fly;
        return true;
    }
}
