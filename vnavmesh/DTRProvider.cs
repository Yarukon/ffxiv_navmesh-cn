using Dalamud.Game.Gui.Dtr;
using System;

namespace Navmesh;

public class DTRProvider(NavmeshManager Manager, AsyncMoveRequest AsyncMove) : IDisposable
{
    private IDtrBarEntry DtrBarEntry { get; } = Service.DtrBar.Get("vnavmesh");

    public void Dispose() => 
        DtrBarEntry.Remove();

    public void Update()
    {
        DtrBarEntry.Shown = Service.Config.EnableDTR;
        if (!DtrBarEntry.Shown) return;
        
        var loadProgress = Manager.LoadTaskProgress;
        var status       = loadProgress >= 0 ? $"构建进度 {loadProgress * 100:f0}%" : Manager.Navmesh != null ? "就绪" : "未就绪";
        if (AsyncMove.TaskInProgress)
            status = "计算路径中";
        
        DtrBarEntry.Text = "导航: " + status;
    }
}
