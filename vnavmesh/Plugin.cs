using Dalamud.Game.Command;
using Dalamud.Interface.Utility;
using Dalamud.Interface.Windowing;
using Dalamud.Plugin;
using Dalamud.Plugin.Services;
using FFXIVClientStructs.FFXIV.Client.Graphics.Render;
using ImGuiNET;
using Navmesh.Movement;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Globalization;
using System.Numerics;
using System.Threading.Tasks;
using Navmesh.Utilities;

namespace Navmesh;

public sealed class Plugin : IDalamudPlugin
{
    private readonly WindowSystem WindowSystem = new("vnavmesh");
    private readonly DTRProvider  DtrProvider;
    private readonly IPCProvider  IPCProvider;
    
    internal readonly NavmeshManager   NavmeshManager;
    internal readonly FollowPath       FollowPath;
    internal readonly AsyncMoveRequest AsyncMove;
    internal readonly MainWindow       MainWindow;

    public static Plugin Instance() => instance;
    
    private static Plugin instance = null!;

    public Plugin(IDalamudPluginInterface dalamud)
    {
        instance = this;
        
        dalamud.Create<Service>();
        
        Service.Config.Load(dalamud.ConfigFile);
        Service.Config.Modified += () => Service.Config.Save(dalamud.ConfigFile);

        NavmeshManager = new(new($"{dalamud.ConfigDirectory.FullName}/meshcache"));
        FollowPath     = new(dalamud, NavmeshManager);
        AsyncMove      = new(NavmeshManager, FollowPath);
        DtrProvider    = new(NavmeshManager, AsyncMove);
        MainWindow     = new(NavmeshManager, FollowPath, AsyncMove, dalamud.ConfigDirectory.FullName);
        IPCProvider    = new(NavmeshManager, FollowPath, AsyncMove, MainWindow, DtrProvider);

        WindowSystem.AddWindow(MainWindow);

        dalamud.UiBuilder.Draw += Draw;
        dalamud.UiBuilder.OpenConfigUi += () => MainWindow.IsOpen = true;

        var cmd = new CommandInfo(OnCommand)
        {
            HelpMessage = """
            Opens the debug menu.
            /vnav moveto <X> <Y> <Z> → 移动至该坐标
            /vnav movedir <X> <Y> <Z> → 根据当前面向移动指定单位距离
            /vnav movetarget → 移动至当前目标位置
            /vnav moveflag → 移动至当前标点位置
            /vnav flyto <X> <Y> <Z> → 飞行至该坐标
            /vnav flydir <X> <Y> <Z> → 根据当前面向移动指定单位距离
            /vnav flytarget → 飞行至当前目标位置
            /vnav flyflag → 飞行至当前标点位置
            /vnav stop → 停止所有导航移动任务
            /vnav reload → 从缓存中重新加载本区域导航数据
            /vnav rebuild → 从游戏中重新构建本区域导航数据
            /vnav recalculate → 重新计算当前路径（卡住时使用）
            /vnav aligncamera → 将当前面向对齐移动方向
            /vnav aligncamera true|yes|enable → 启用移动时将当前面向对齐移动方向
            /vnav aligncamera false|no|disable → 禁用移动时将当前面向对齐移动方向
            /vnav dtr → 开关服务器状态栏插件状态显示
            /vnav collider → 开关调试性碰撞显示
            """,
            
            ShowInHelp = true,
        };
        
        Service.CommandManager.AddHandler("/vnav", cmd);

        Service.Framework.Update += OnUpdate;

        // Rebuild random table based on user settings
        (NavmeshQuery._filter2 as RandomizedQueryFilter)!.RebuildRandomTable();

        // Service.PluginInterface.UiBuilder.DisableUserUiHide = true; // Disable user UI hide to ensure our windows are always visible
    }

    public void Dispose()
    {
        Service.Framework.Update -= OnUpdate;

        Service.CommandManager.RemoveHandler("/vnav");
        
        Service.PluginInterface.UiBuilder.Draw -= Draw;
        WindowSystem.RemoveAllWindows();

        IPCProvider.Dispose();
        MainWindow.Dispose();
        DtrProvider.Dispose();
        AsyncMove.Dispose();
        FollowPath.Dispose();
        NavmeshManager.Dispose();
    }

    public static void DuoLog(Exception ex)
    {
        DuoLog(ex, ex.Message);
        throw ex;
    }

    public static void DuoLog(Exception ex, string message)
    {
        Service.ChatGui.Print($"[{Service.PluginInterface.Manifest.Name}] {message}");
        Service.Log.Error(ex, message);
    }

    private void OnUpdate(IFramework fwk)
    {
        NavmeshManager.Update();
        FollowPath.Update();
        AsyncMove.Update();
        DtrProvider.Update();
    }

    /*public static Vector3 StartPos = new();
    public static Vector3 EndPos = new();
    public static List<Vector3> OriginalPath = [];
    public static List<Vector3> DensedPath = [];
    public static List<Vector3> CenteredPath = [];
    public List<List<Vector3>> Paths = [];
    public static bool OriginalPath_Locked = false;
    public static bool DensedPath_Locked = false;
    public static bool CenteredPath_Locked = false;
    public static bool Paths_Locked = false;*/

    private void Draw()
    {
        MainWindow.StartFrame();
        WindowSystem.Draw();
        MainWindow.EndFrame();

        // RenderDebug();
    }

    /*private void RenderDebug()
    {
        ImGui.PushStyleVar(ImGuiStyleVar.WindowPadding, new Vector2(0, 0));
        ImGui.SetNextWindowViewport(ImGui.GetMainViewport().ID);
        ImGui.SetNextWindowPos(new Vector2(0, 0) + ImGui.GetMainViewport().Pos);

        ImGui.Begin("NavMesh Canvas_2D", ImGuiWindowFlags.NoInputs | ImGuiWindowFlags.NoNav | ImGuiWindowFlags.NoTitleBar | ImGuiWindowFlags.NoScrollbar | ImGuiWindowFlags.NoBackground);

        ImGui.SetWindowSize(ImGui.GetIO().DisplaySize);

        ImGui.GetWindowDrawList().AddText(new(15, 15), 0xffffffff, $"NavNode Visualizer");

        // Canvas Area
        var drawList = ImGui.GetWindowDrawList();

        if (!Paths_Locked)
        {
            foreach (var path in Paths)
            {
                if (Paths_Locked)
                    break;

                for (int i = 0; i < path.Count - 1; i++)
                {
                    if (Paths_Locked)
                        break;

                    var start = path[i];
                    var end = path[i + 1];

                    if (Service.GameGui.WorldToScreen(start, out var start2D) && Service.GameGui.WorldToScreen(end, out var end2D))
                    {
                        drawList.AddCircleFilled(start2D, 5, 0xAAFBC117);
                        drawList.AddLine(start2D, end2D, 0xAAFFFFFF, 2);
                    }
                }
            }
        }

        if (!DensedPath_Locked)
        {
            for (int i = 0; i < DensedPath.Count - 1; i++)
            {
                if (DensedPath_Locked)
                    break;

                var start = DensedPath[i];
                var end = DensedPath[i + 1];

                if (Service.GameGui.WorldToScreen(start, out var start2D) && Service.GameGui.WorldToScreen(end, out var end2D))
                {
                    drawList.AddCircleFilled(start2D, 5, 0xAA00FF00);
                    drawList.AddLine(start2D, end2D, 0xAA00FF00, 2);
                }
            }
        }

        if (!OriginalPath_Locked)
        {
            for (int i = 0; i < OriginalPath.Count - 1; i++)
            {
                if (OriginalPath_Locked)
                    break;

                var start = OriginalPath[i];
                var end = OriginalPath[i + 1];

                if (Service.GameGui.WorldToScreen(start, out var start2D) && Service.GameGui.WorldToScreen(end, out var end2D))
                {
                    drawList.AddCircleFilled(start2D, 5, 0xAA0000FF);
                    drawList.AddLine(start2D, end2D, 0xAA0000FF, 2);
                }
            }
        }

        if (!CenteredPath_Locked)
        {
            for (int i = 0; i < CenteredPath.Count - 1; i++)
            {
                if (CenteredPath_Locked)
                    break;

                var start = CenteredPath[i];
                var end = CenteredPath[i + 1];

                if (Service.GameGui.WorldToScreen(start, out var start2D) && Service.GameGui.WorldToScreen(end, out var end2D))
                {
                    drawList.AddCircleFilled(start2D, 5, 0xAA00FFFF);
                    drawList.AddLine(start2D, end2D, 0xAA00FFFF, 2);
                }
            }
        }

        ImGui.End();
        ImGui.PopStyleVar();

        if (ImGui.Begin("NavMesh debugger"))
        {
            var player = Service.ClientState.LocalPlayer;
            ImGui.Text("From");
            ImGui.SetNextItemWidth(100);
            ImGui.InputFloat("X##FromX", ref StartPos.X);
            ImGui.SameLine();
            ImGui.SetNextItemWidth(100);
            ImGui.InputFloat("Y##FromY", ref StartPos.Y);
            ImGui.SameLine();
            ImGui.SetNextItemWidth(100);
            ImGui.InputFloat("Z##FromZ", ref StartPos.Z);

            ImGui.Text("To");
            ImGui.SetNextItemWidth(100);
            ImGui.InputFloat("X##ToX", ref EndPos.X);
            ImGui.SameLine();
            ImGui.SetNextItemWidth(100);
            ImGui.InputFloat("Y##ToY", ref EndPos.Y);
            ImGui.SameLine();
            ImGui.SetNextItemWidth(100);
            ImGui.InputFloat("Z##ToZ", ref EndPos.Z);

            if (ImGui.Button("set start to playerpos"))
                StartPos = player?.Position ?? default;

            if (ImGui.Button("目的地: 当前位置"))
                EndPos = player?.Position ?? default;
            ImGui.SameLine();
            if (ImGui.Button("目的地: 选中目标位置"))
                EndPos = player?.TargetObject?.Position ?? default;

            if (ImGui.Button("目的地: 标点位置"))
                EndPos = MapUtils.FlagToPoint(NavmeshManager.Query!) ?? default;

            if (ImGui.Button("call pathfind"))
            {
                Paths_Locked = true;
                Task.Run(async () => Paths.Add(await NavmeshManager.QueryPath(StartPos, EndPos, false)));
                Paths_Locked = false;
            }

            ImGui.SameLine();

            if (ImGui.Button("call voxel pathfind"))
            {
                Paths_Locked = true;
                Task.Run(async () => Paths.Add(await NavmeshManager.QueryPath(StartPos, EndPos, true)));
                Paths_Locked = false;
            }

            ImGui.Text($"path count: {Paths.Count}");

            if (ImGui.Button("clear all"))
            {
                Paths_Locked = true;
                Paths.Clear();
                Paths_Locked = false;
            }

            ImGui.End();
        }
    }*/

    private void OnCommand(string command, string arguments)
    {
        if (arguments.Length == 0)
        {
            MainWindow.IsOpen ^= true;
            return;
        }

        var args = arguments.Split(' ');
        switch (args[0])
        {
            case "reload":
                NavmeshManager.Reload(true);
                break;
            case "rebuild":
                NavmeshManager.Reload(false);
                break;
            case "moveto":
                MoveToCommand(args, false, false);
                break;
            case "movedir":
                if (args.Length > 3)
                    MoveToCommand(args, true, false);
                break;
            case "movetarget":
                var moveTarget = Service.TargetManager.Target;
                if (moveTarget != null)
                    AsyncMove.MoveTo(moveTarget.Position, false);
                break;
            case "moveflag":
                MoveFlagCommand(false);
                break;
            case "flyto":
                MoveToCommand(args, false, true);
                break;
            case "flydir":
                if (args.Length > 3)
                    MoveToCommand(args, true, true);
                break;
            case "flytarget":
                var flyTarget = Service.TargetManager.Target;
                if (flyTarget != null)
                    AsyncMove.MoveTo(flyTarget.Position, true);
                break;
            case "flyflag":
                MoveFlagCommand(true);
                break;
            case "stop":
                AsyncMove.CancelPathfinding();
                FollowPath.Stop();
                break;
            case "aligncamera":
                if (args.Length == 1)
                    Service.Config.AlignCameraToMovement ^= true;
                else
                    AlignCameraCommand(args[1]);
                Service.Config.NotifyModified();
                break;
            case "dtr":
                Service.Config.EnableDTR ^= true;
                Service.Config.NotifyModified();
                break;
            case "collider":
                Service.Config.ForceShowGameCollision ^= true;
                Service.Config.NotifyModified();
                break;
        }
    }

    private void MoveToCommand(string[] args, bool relativeToPlayer, bool fly)
    {
        var originActor = relativeToPlayer ? Service.ClientState.LocalPlayer : null;
        var origin = originActor?.Position ?? new();
        var offset = new Vector3(
            float.Parse(args[1], CultureInfo.InvariantCulture),
            float.Parse(args[2], CultureInfo.InvariantCulture),
            float.Parse(args[3], CultureInfo.InvariantCulture));
        AsyncMove.MoveTo(origin + offset, fly);
    }

    private void MoveFlagCommand(bool fly)
    {
        if (NavmeshManager.Query == null) return;
        
        var pt = MapUtils.FlagToPoint(NavmeshManager.Query);
        if (pt == null) return;
        
        AsyncMove.MoveTo(pt.Value, fly);
    }

    private static void AlignCameraCommand(string arg)
    {
        arg = arg.ToLower();
        
        Service.Config.AlignCameraToMovement = arg switch
        {
            "true" or "yes" or "enable"  => true,
            "false" or "no" or "disable" => false,
            _                            => Service.Config.AlignCameraToMovement
        };
    }
}
