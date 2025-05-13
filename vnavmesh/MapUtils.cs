using System.Numerics;
using FFXIVClientStructs.FFXIV.Client.UI.Agent;

namespace Navmesh;

public static class MapUtils
{
    public static Vector3? FlagToPoint(NavmeshQuery q)
    {
        var flag = GetFlagPosition();
        return flag == null ? null : q.FindPointOnFloor(new(flag.Value.X, 1024, flag.Value.Y));
    }

    private static unsafe Vector2? GetFlagPosition()
    {
        var map = AgentMap.Instance();
        if (map == null || map->IsFlagMarkerSet != 1) return null;
        
        var marker = map->FlagMapMarker;
        return new(marker.XFloat, marker.YFloat);
    }
}
