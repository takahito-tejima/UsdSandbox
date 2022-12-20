#pragma once

#include "pxr/imaging/hdGp/generativeProcedural.h"

#include "pxr/imaging/hd/primvarsSchema.h"
#include "pxr/imaging/hd/retainedDataSource.h"

PXR_NAMESPACE_OPEN_SCOPE

class MyProceduralMesh : public HdGpGenerativeProcedural
{
public:
    MyProceduralMesh(const SdfPath& proceduralPrimPath);
    static HdGpGenerativeProcedural* New(const SdfPath& proceduralPrimPath);

    DependencyMap UpdateDependencies(const HdSceneIndexBaseRefPtr& inputScene) override;

    ChildPrimTypeMap Update(const HdSceneIndexBaseRefPtr& inputScene,
                            const ChildPrimTypeMap& previousResult,
                            const DependencyMap& dirtiedDependencies,
                            HdSceneIndexObserver::DirtiedPrimEntries* outputDirtiedPrims) override;

    HdSceneIndexPrim GetChildPrim(const HdSceneIndexBaseRefPtr& inputScene, const SdfPath& childPrimPath) override;

private:
    float _size  = 1.0f;
    float _param = 0.0f;
};

PXR_NAMESPACE_CLOSE_SCOPE