#include "gp_fur.h"
#include "gp_mesh.h"

#include <pxr/imaging/hdGp/generativeProceduralPlugin.h>
#include <pxr/imaging/hdGp/generativeProceduralPluginRegistry.h>

PXR_NAMESPACE_USING_DIRECTIVE

class MyProceduralMeshPlugin : public HdGpGenerativeProceduralPlugin
{
public:
    MyProceduralMeshPlugin() = default;
    HdGpGenerativeProcedural* Construct(const SdfPath& proceduralPrimPath) override
    {
        return MyProceduralMesh::New(proceduralPrimPath);
    }
};

class MyProceduralFurPlugin : public HdGpGenerativeProceduralPlugin
{
public:
    MyProceduralFurPlugin() = default;
    HdGpGenerativeProcedural* Construct(const SdfPath& proceduralPrimPath) override
    {
        return MyProceduralFur::New(proceduralPrimPath);
    }
};

TF_REGISTRY_FUNCTION(TfType)
{
    HdGpGenerativeProceduralPluginRegistry::Define<MyProceduralMeshPlugin, HdGpGenerativeProceduralPlugin>();
    HdGpGenerativeProceduralPluginRegistry::Define<MyProceduralFurPlugin, HdGpGenerativeProceduralPlugin>();
}
