using ImGuiNET;
using Newtonsoft.Json.Linq;
using System;
using System.IO;
using Dalamud.Interface.Utility;
using FFXIVClientStructs.FFXIV.Client.UI;

namespace Navmesh;

public class Config
{
    private const int _version = 1;

    public bool AutoLoadNavmesh = true;
    public bool EnableDTR       = true;
    public bool AlignCameraToMovement;
    public bool ShowWaypoints;
    public bool ForceShowGameCollision;
    public bool CancelMoveOnUserInput;
    
    public float VoxelPathfindRandomFactor = 0.5f;
    public bool EnablePathCachePrewarming = true;

    public event Action? Modified;

    public void NotifyModified() => Modified?.Invoke();

    public void Draw()
    {
        ImGui.Spacing();
        
        ImGui.Text("一般");
        
        ImGui.Separator();
        ImGui.Spacing();
        
        if (ImGui.Checkbox("切换区域时, 自动加载/构建区域导航数据", ref AutoLoadNavmesh))
            NotifyModified();
        
        if (ImGui.Checkbox("在服务器状态栏显示导航信息", ref EnableDTR))
            NotifyModified();
        
        ImGui.NewLine();
        
        ImGui.Text("操控");
        
        ImGui.Separator();
        ImGui.Spacing();
        
        if (ImGui.Checkbox("将镜头面向对齐前进方向", ref AlignCameraToMovement))
            NotifyModified();
        
        if (ImGui.Checkbox("当尝试操控游戏角色时, 自动取消寻路任务", ref CancelMoveOnUserInput))
            NotifyModified();
        
        ImGui.NewLine();
        
        ImGui.Text("显示");
        
        ImGui.Separator();
        ImGui.Spacing();
        
        if (ImGui.Checkbox("显示导航路径点", ref ShowWaypoints))
            NotifyModified();
        
        if (ImGui.Checkbox("强制显示游戏内碰撞体", ref ForceShowGameCollision))
            NotifyModified();
        
        ImGui.NewLine();
        
        ImGui.Text("体素导航 (飞行)");
        
        ImGui.Separator();
        ImGui.Spacing();

        ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
        if (ImGui.SliderFloat("路线随机性", ref VoxelPathfindRandomFactor, 0.1f, 1f, "%.1f"))
            NotifyModified();
            
        if (ImGui.Checkbox("启用路径缓存预热", ref EnablePathCachePrewarming))
            NotifyModified();
    }

    public void Save(FileInfo file)
    {
        try
        {
            JObject jContents = new()
            {
                { "Version", _version },
                { "Payload", JObject.FromObject(this) }
            };
            File.WriteAllText(file.FullName, jContents.ToString());
        }
        catch (Exception e)
        {
            Service.Log.Error($"保存配置文件至 {file.FullName} 时失败: {e}");
        }
    }

    public void Load(FileInfo file)
    {
        try
        {
            var contents = File.ReadAllText(file.FullName);
            var json = JObject.Parse(contents);
            var version = (int?)json["Version"] ?? 0;
            if (json["Payload"] is JObject payload)
            {
                payload = ConvertConfig(payload, version);
                var thisType = GetType();
                foreach (var (f, data) in payload)
                {
                    var thisField = thisType.GetField(f);
                    if (thisField != null)
                    {
                        var value = data?.ToObject(thisField.FieldType);
                        if (value != null)
                        {
                            thisField.SetValue(this, value);
                        }
                    }
                }
            }
        }
        catch (Exception e)
        {
            Service.Log.Error($"无法从 {file.FullName} 加载配置内容: {e}");
        }
    }

    private static JObject ConvertConfig(JObject payload, int version) => payload;
}
