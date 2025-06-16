using Dalamud.Game;
using Dalamud.Game.ClientState.Objects;
using Dalamud.IoC;
using Dalamud.Plugin;
using Dalamud.Plugin.Services;

namespace Navmesh;

public class Service
{
    [PluginService] public static IPluginLog              Log             { get; private set; } = null!;
    [PluginService] public static ICommandManager         CommandManager  { get; private set; } = null!;
    [PluginService] public static IDataManager            DataManager     { get; private set; } = null!;
    [PluginService] public static ITargetManager          TargetManager   { get; private set; } = null!;
    [PluginService] public static IClientState            ClientState     { get; private set; } = null!;
    [PluginService] public static ISigScanner             SigScanner      { get; private set; } = null!;
    [PluginService] public static IGameInteropProvider    Hook            { get; private set; } = null!;
    [PluginService] public static ICondition              Condition       { get; private set; } = null!;
    [PluginService] public static IChatGui                ChatGui         { get; private set; } = null!;
    [PluginService] public static IFramework              Framework       { get; private set; } = null!;
    [PluginService] public static IDtrBar                 DtrBar          { get; private set; } = null!;
    [PluginService] public static IDalamudPluginInterface PluginInterface { get; private set; } = null!;
    [PluginService] public static IGameConfig             GameConfig      { get; private set; } = null!;
    [PluginService] public static IGameGui                GameGui         { get; private set; } = null!;

    public static Config Config { get; } = new();
    
    public static Lumina.Excel.ExcelSheet<T> LuminaSheet<T>() where T : struct, Lumina.Excel.IExcelRow<T> => 
        DataManager.GetExcelSheet<T>();
    
    public static T? LuminaRow<T>(uint row) where T : struct, Lumina.Excel.IExcelRow<T> => 
        LuminaSheet<T>().GetRowOrDefault(row);
}
