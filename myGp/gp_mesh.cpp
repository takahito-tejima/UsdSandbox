#include "gp_mesh.h"

#include "pxr/imaging/hd/meshSchema.h"
#include "pxr/imaging/hd/meshTopologySchema.h"
#include "pxr/imaging/hd/primvarsSchema.h"
#include "pxr/imaging/hd/retainedDataSource.h"
#include "pxr/imaging/hd/tokens.h"
#include "pxr/imaging/hd/xformSchema.h"

#include <iostream>

PXR_NAMESPACE_USING_DIRECTIVE

TF_DEFINE_PRIVATE_TOKENS(_tokens,
                         (size)  //
                         (param) //
                         (pc)    //
);

MyProceduralMesh::MyProceduralMesh(const SdfPath& proceduralPrimPath)
    : HdGpGenerativeProcedural(proceduralPrimPath)
{
}

HdGpGenerativeProcedural*
MyProceduralMesh::New(const SdfPath& proceduralPrimPath)
{
    return new MyProceduralMesh(proceduralPrimPath);
}

HdGpGenerativeProcedural::DependencyMap
MyProceduralMesh::UpdateDependencies(const HdSceneIndexBaseRefPtr& inputScene)
{
    DependencyMap result;
    // Do nothing
    return result;
}

HdGpGenerativeProcedural::ChildPrimTypeMap
MyProceduralMesh::Update(const HdSceneIndexBaseRefPtr& inputScene,
                         const ChildPrimTypeMap& previousResult,
                         const DependencyMap& dirtiedDependencies,
                         HdSceneIndexObserver::DirtiedPrimEntries* outputDirtiedPrims)
{
    ChildPrimTypeMap result;
    HdSceneIndexPrim myPrim   = inputScene->GetPrim(_GetProceduralPrimPath());
    HdPrimvarsSchema primvars = HdPrimvarsSchema::GetFromParent(myPrim.dataSource);

    // parameter
    if (HdSampledDataSourceHandle sizeDs = primvars.GetPrimvar(_tokens->size).GetPrimvarValue())
    {
        _size = std::max(1.0f, sizeDs->GetValue(0.0f).GetWithDefault(_size));
    }
    if (HdSampledDataSourceHandle paramDs = primvars.GetPrimvar(_tokens->param).GetPrimvarValue())
    {
        _param = paramDs->GetValue(0.0f).GetWithDefault(_param);
    }

    // このサンプルではキャッシュは未実装で毎回全部作っているが、本来は個別に更新管理するべき
    SdfPath path      = _GetProceduralPrimPath();
    SdfPath childPath = path.AppendChild(_tokens->pc); // Hydra Rprim を一つだけ作る。名前は何でもいい
    result[childPath] = HdPrimTypeTokens->mesh;

    if (outputDirtiedPrims)
    {
        outputDirtiedPrims->emplace_back(
            childPath,
            HdDataSourceLocatorSet{ HdPrimvarsSchema::GetPointsLocator(), HdMeshTopologySchema::GetDefaultLocator() });
    }

    return result;
}

HdSceneIndexPrim
MyProceduralMesh::GetChildPrim(const HdSceneIndexBaseRefPtr& inputScene, const SdfPath& childPrimPath)
{
    HdSceneIndexPrim result;

    // size のグリッド状メッシュを愚直に毎回作成するサンプル
    int size   = int(_size);
    int stride = size + 1;
    VtIntArray faceVertexCounts;
    VtIntArray faceVertexIndices;
    faceVertexCounts.reserve(size * size);
    faceVertexIndices.reserve(size * size * 4);
    VtVec3fArray points(stride * stride);
    int index = 0;
    for (int z = 0; z <= size; ++z)
    {
        for (int x = 0; x <= size; ++x)
        {
            if (x > 0 && z > 0)
            {
                faceVertexCounts.push_back(4);
                faceVertexIndices.push_back(index - stride);
                faceVertexIndices.push_back(index - stride - 1);
                faceVertexIndices.push_back(index - 1);
                faceVertexIndices.push_back(index);
            }
            float y         = sin(x + _param) * cos(z + _param);
            points[index++] = GfVec3f(x - size / 2 - 0.5f, y, z - size / 2 - 0.5f);
        }
    }

    result.primType   = HdPrimTypeTokens->mesh;
    result.dataSource = HdRetainedContainerDataSource::New(
        HdXformSchemaTokens->xform,
        HdXformSchema::Builder()
            .SetMatrix(HdRetainedTypedSampledDataSource<GfMatrix4d>::New(GfMatrix4d().SetIdentity()))
            .Build(),
        HdMeshSchemaTokens->mesh,
        HdMeshSchema::Builder()
            .SetTopology(HdMeshTopologySchema::Builder()
                             .SetFaceVertexCounts(HdRetainedTypedSampledDataSource<VtIntArray>::New(faceVertexCounts))
                             .SetFaceVertexIndices(HdRetainedTypedSampledDataSource<VtIntArray>::New(faceVertexIndices))
                             .Build())
            .Build(),
        HdPrimvarsSchemaTokens->primvars,
        HdRetainedContainerDataSource::New(
            HdPrimvarsSchemaTokens->points,
            HdPrimvarSchema::Builder()
                .SetPrimvarValue(HdRetainedTypedSampledDataSource<VtArray<GfVec3f>>::New(points))
                .SetInterpolation(HdPrimvarSchema::BuildInterpolationDataSource(HdPrimvarSchemaTokens->vertex))
                .SetRole(HdPrimvarSchema::BuildRoleDataSource(HdPrimvarSchemaTokens->point))
                .Build()));

    return result;
}
