#pragma once

#include <pxr/imaging/hdGp/generativeProcedural.h>

PXR_NAMESPACE_OPEN_SCOPE

class MyProceduralFur : public HdGpGenerativeProcedural
{
public:
    MyProceduralFur(const SdfPath& proceduralPrimPath);
    static HdGpGenerativeProcedural* New(const SdfPath& proceduralPrimPath);

    DependencyMap UpdateDependencies(const HdSceneIndexBaseRefPtr& inputScene) override;

    ChildPrimTypeMap Update(const HdSceneIndexBaseRefPtr& inputScene,
                            const ChildPrimTypeMap& previousResult,
                            const DependencyMap& dirtiedDependencies,
                            HdSceneIndexObserver::DirtiedPrimEntries* outputDirtiedPrims) override;

    HdSceneIndexPrim GetChildPrim(const HdSceneIndexBaseRefPtr& inputScene, const SdfPath& childPrimPath) override;

private:
    HdSampledDataSourceHandle _meshPointsDs, _meshFaceVertexCountsDs, _meshFaceIndicesDs, _numSampleDs, _lengthDs;
    HdSampledDataSourceHandle _curvePointsDs;
};

PXR_NAMESPACE_CLOSE_SCOPE