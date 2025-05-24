namespace Navmesh.Customizations;

[CustomizationTerritory(1142)]
class Z1142TheSirensongSea : NavmeshCustomization
{
    public override int Version => 2;

    public Z1142TheSirensongSea()
    {
        Settings.Filtering -= NavmeshSettings.Filter.LedgeSpans; // this allows mesh to go down the bowsprit to the land from the boat
    }

    public override void CustomizeScene(SceneExtractor scene)
    {
        if (scene.Meshes.TryGetValue("bg/ex2/03_ocn_o3/dun/o3d1/collision/tr1515.pcb", out var mesh))
        {
            foreach (var inst in mesh.Instances)
            {
                inst.ForceSetPrimFlags |= SceneExtractor.PrimitiveFlags.ForceUnwalkable;
            }
        }
    }
}
