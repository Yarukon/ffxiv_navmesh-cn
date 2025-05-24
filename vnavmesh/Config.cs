using ImGuiNET;
using Newtonsoft.Json.Linq;
using System;
using System.IO;
using FFXIVClientStructs.FFXIV.Client.UI;

namespace Navmesh;

public class Config
{
    private const int _version = 1;

    public bool AutoLoadNavmesh = true;
    public bool EnableDTR = true;
    public bool AlignCameraToMovement;
    public bool ShowWaypoints;
    public bool ForceShowGameCollision;
    public bool CancelMoveOnUserInput;

    // Random path
    public bool UseRandomPathGen = false;
    public float RandomPath_MaxRandomRange = 0.5f;
    public float RandomPath_Randomness = 0.7f;
    public int RandomPath_MinPointsPerSeg = 2;
    public int RandomPath_MaxPointsPerSeg = 5;
    public float RandomPath_MaxDeviationRatio = 0.4f;

    public bool RandomPath_UsePathCentering = true;
    public float RandomPath_MaxCenteringDist = 4f;
    public float RandomPath_CenteringStrength = 0.7f;
    public float RandomPath_RandomOffsetRatio = 0.15f;

    public event Action? Modified;

    public void NotifyModified() => Modified?.Invoke();

    public void Draw()
    {
        if (ImGui.Checkbox("切换区域时, 自动加载/构建区域导航数据", ref AutoLoadNavmesh))
            NotifyModified();
        if (ImGui.Checkbox("启用服务器状态栏信息", ref EnableDTR))
            NotifyModified();
        if (ImGui.Checkbox("将镜头面向对齐前进方向", ref AlignCameraToMovement))
            NotifyModified();
        if (ImGui.Checkbox("显示即将去往的各目的地点", ref ShowWaypoints))
            NotifyModified();
        if (ImGui.Checkbox("始终开启游戏内碰撞显示", ref ForceShowGameCollision))
            NotifyModified();
        if (ImGui.Checkbox("当尝试操控游戏角色时, 自动取消寻路任务", ref CancelMoveOnUserInput))
            NotifyModified();
        if (ImGui.Checkbox("对路径使用随机算法", ref UseRandomPathGen))
            NotifyModified();

        if (UseRandomPathGen)
        {
            if (ImGui.SliderFloat("路径请求随机程度", ref RandomPath_MaxRandomRange, 0.0f, 1.0f))
            {
                NotifyModified();
                (NavmeshQuery._filter2 as RandomizedQueryFilter)!.RebuildRandomTable();
            }

            bool valueChanged = false;
            valueChanged |= ImGui.SliderFloat("路径后处理随机程度", ref RandomPath_Randomness, 0.0f, 1.0f);
            valueChanged |= ImGui.SliderInt("每段最少插入点数", ref RandomPath_MinPointsPerSeg, 1, 10);
            valueChanged |= ImGui.SliderInt("每段最多插入点数", ref RandomPath_MaxPointsPerSeg, 1, 10);
            valueChanged |= ImGui.SliderFloat("最大偏移距离比例 (相对于两点间距离)", ref RandomPath_MaxDeviationRatio, 0f, 1f);

            if (ImGui.Checkbox("使用路径置中", ref RandomPath_UsePathCentering))
                NotifyModified();

            if (RandomPath_UsePathCentering)
            {
                valueChanged |= ImGui.SliderFloat("置中强度", ref RandomPath_CenteringStrength, 0f, 1f);
                valueChanged |= ImGui.SliderFloat("最大置中距离", ref RandomPath_MaxCenteringDist, 0f, 10f);
                valueChanged |= ImGui.SliderFloat("置中后随机偏移比例", ref RandomPath_RandomOffsetRatio, 0f, 0.5f);
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

    private static JObject ConvertConfig(JObject payload, int version)
    {
        return payload;
    }
}
