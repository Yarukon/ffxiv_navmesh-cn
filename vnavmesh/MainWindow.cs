using System;
using Dalamud.Interface.Utility.Raii;
using Dalamud.Interface.Windowing;
using Navmesh.Debug;
using Navmesh.Movement;

namespace Navmesh;

public class MainWindow : Window, IDisposable
{
    private readonly FollowPath FollowPath;
    
    private readonly DebugDrawer         DebugDrawer = new();
    private readonly DebugGameCollision  DebugGameColl;
    private readonly DebugNavmeshManager DebugNavmeshManager;
    private readonly DebugNavmeshCustom  DebugNavmeshCustom;
    private readonly DebugLayout         DebugLayout;
    
    private readonly string              ConfigDirectory;

    public MainWindow(NavmeshManager manager, FollowPath path, AsyncMoveRequest move, string configDir) : base("vnavmesh 寻路导航")
    {
        FollowPath = path;

        ConfigDirectory = configDir;
        
        DebugGameColl       = new(DebugDrawer);
        DebugNavmeshManager = new(DebugDrawer, manager, path, move);
        DebugNavmeshCustom  = new(DebugDrawer, DebugGameColl, manager, ConfigDirectory);
        DebugLayout         = new(DebugGameColl);
    }

    public void Dispose()
    {
        DebugLayout.Dispose();
        DebugNavmeshCustom.Dispose();
        DebugNavmeshManager.Dispose();
        DebugGameColl.Dispose();
        DebugDrawer.Dispose();
    }

    public void StartFrame() => DebugDrawer.StartFrame();

    public void EndFrame()
    {
        DebugGameColl.DrawVisualizers();
        if (Service.Config.ShowWaypoints)
        {
            if (Service.ClientState.LocalPlayer is { } localPlayer)
            {
                var from  = localPlayer.Position;
                var color = 0xff00ff00;
                foreach (var to in FollowPath.Waypoints)
                {
                    DebugDrawer.DrawWorldLine(from, to, color);
                    DebugDrawer.DrawWorldPointFilled(to, 3, 0xff0000ff);
                    from  = to;
                    color = 0xff00ffff;
                }
            }
        }

        DebugDrawer.EndFrame();
    }

    public override void Draw()
    {
        using var tabs = ImRaii.TabBar("###Tabs");
        if (!tabs) return;
        
        using (var tab = ImRaii.TabItem("配置"))
        {
            if (tab)
                Service.Config.Draw();
        }

        using (var tab = ImRaii.TabItem("层级"))
        {
            if (tab)
                DebugLayout.Draw();
        }

        using (var tab = ImRaii.TabItem("碰撞"))
        {
            if (tab)
                DebugGameColl.Draw();
        }

        using (var tab = ImRaii.TabItem("导航"))
        {
            if (tab)
                DebugNavmeshManager.Draw();
        }

        using (var tab = ImRaii.TabItem("自定义导航"))
        {
            if (tab)
                DebugNavmeshCustom.Draw();
        }
    }
}
