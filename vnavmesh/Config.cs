using ImGuiNET;
using Newtonsoft.Json.Linq;
using System;
using System.IO;
using Dalamud.Interface.Utility;

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

    // Random path
    public bool UseRandomPathGen = false;
    public RandomPathMode RandomPath_Mode = RandomPathMode.RECAST_LINEPULL;

    // Recast Line Pull
    public float RandomPath_MaxRandomRange = 0.5f;
    public float RandomPath_Randomness = 0.7f;
    public int RandomPath_MinPointsPerSeg = 2;
    public int RandomPath_MaxPointsPerSeg = 5;
    public float RandomPath_MaxDeviationRatio = 0.4f;

    public bool RandomPath_UsePathCentering = true;
    public float RandomPath_MaxCenteringDist = 4f;
    public float RandomPath_CenteringStrength = 0.7f;
    public float RandomPath_RandomOffsetRatio = 0.15f;
    
    public float VoxelPathfindRandomFactor = 0.5f;

    // Custom Line Pull
    public float RandomPath_CPull_Directness = 0.7f;
    public float RandomPath_CPull_Natrualness = 0.85f;
    
    // 体素路径查找性能优化参数
    public float VoxelPathfindMaxStepsBaseFactor = 2.0f;        // 基础步数倍数因子
    public float VoxelPathfindEarlyTerminationDistance = 2.0f;  // 早期终止距离阈值
    public int   VoxelPathfindMinSteps = 5000;                  // 最小步数保证
    public float VoxelPathfindMaxStepsMultiplier = 1000.0f;     // 距离步数乘数

    public bool VoxelPathfindPostProcess = true; // 是否启用体素路径后处理

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
        if (ImGui.SliderFloat("路线随机性", ref VoxelPathfindRandomFactor, 0f, 1f, "%.1f"))
            NotifyModified();
        
        ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
        if (ImGui.SliderFloat("最大步数基础因子", ref VoxelPathfindMaxStepsBaseFactor, 1.0f, 5.0f, "%.1f"))
            NotifyModified();
            
        ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
        if (ImGui.SliderFloat("早期终止距离", ref VoxelPathfindEarlyTerminationDistance, 0.5f, 10.0f, "%.1f"))
            NotifyModified();
            
        ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
        if (ImGui.SliderInt("最小保证步数", ref VoxelPathfindMinSteps, 1000, 20000, "%d"))
            NotifyModified();
            
        ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
        if (ImGui.SliderFloat("距离步数乘数", ref VoxelPathfindMaxStepsMultiplier, 100.0f, 5000.0f, "%.0f"))
            NotifyModified();

        if (ImGui.Checkbox("体素路径后处理", ref VoxelPathfindPostProcess))
            NotifyModified();

        ImGui.Text("地面路径随机");

        ImGui.Separator();
        ImGui.Spacing();
        // Random Path Gen
        if (ImGui.Checkbox("对路径使用随机算法", ref UseRandomPathGen))
            NotifyModified();

        if (UseRandomPathGen)
        {

            ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
            if (ImGui.SliderFloat("路径请求随机程度", ref RandomPath_MaxRandomRange, 0.0f, 1.0f))
            {
                NotifyModified();
                (NavmeshQuery._filter2 as RandomizedQueryFilter)!.RebuildRandomTable();
            }

            ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
            if (ImGui.BeginCombo("随机路径模式", GetRandomPathModeName(RandomPath_Mode)))
            {
                foreach (RandomPathMode mode in Enum.GetValues(typeof(RandomPathMode)))
                {
                    if (ImGui.Selectable(GetRandomPathModeName(mode)))
                    {
                        RandomPath_Mode = mode;
                        NotifyModified();
                    }
                }

                ImGui.EndCombo();
            }

            bool valueChanged = false;
            
            switch (RandomPath_Mode)
            {
                case RandomPathMode.RECAST_LINEPULL:
                    ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
                    valueChanged |= ImGui.SliderFloat("路径后处理随机程度", ref RandomPath_Randomness, 0.0f, 1.0f);
                    ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
                    valueChanged |= ImGui.SliderInt("每段最少插入点数", ref RandomPath_MinPointsPerSeg, 1, 10);
                    ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
                    valueChanged |= ImGui.SliderInt("每段最多插入点数", ref RandomPath_MaxPointsPerSeg, 1, 10);
                    ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
                    valueChanged |= ImGui.SliderFloat("最大偏移距离比例 (相对于两点间距离)", ref RandomPath_MaxDeviationRatio, 0f, 1f);

                    ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
                    if (ImGui.Checkbox("使用路径置中", ref RandomPath_UsePathCentering))
                        NotifyModified();

                    if (RandomPath_UsePathCentering)
                    {
                        ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
                        valueChanged |= ImGui.SliderFloat("置中强度", ref RandomPath_CenteringStrength, 0f, 1f);
                        ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
                        valueChanged |= ImGui.SliderFloat("最大置中距离", ref RandomPath_MaxCenteringDist, 0f, 10f);
                        ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
                        valueChanged |= ImGui.SliderFloat("置中后随机偏移比例", ref RandomPath_RandomOffsetRatio, 0f, 0.5f);
                    }
                    break;

                case RandomPathMode.CUSTOM_LINEPULL:
                    ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
                    valueChanged |= ImGui.SliderFloat("路径直接程度", ref RandomPath_CPull_Directness, 0f, 1f);
                    ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
                    valueChanged |= ImGui.SliderFloat("路径自然程度", ref RandomPath_CPull_Natrualness, 0f, 1f);
                    break;

                default:
                    break;
            }

            if (valueChanged)
                NotifyModified();
        }
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

    public enum RandomPathMode
    {
        RECAST_LINEPULL,
        CUSTOM_LINEPULL
    }

    private string GetRandomPathModeName(RandomPathMode _enum) => _enum switch
    {
        RandomPathMode.RECAST_LINEPULL => "Recast 扯线算法",
        RandomPathMode.CUSTOM_LINEPULL => "自定义 扯线算法",
        _ => "无效值"
    };
}
