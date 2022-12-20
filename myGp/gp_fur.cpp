#include "gp_fur.h"

#include "pxr/imaging/hd/basisCurvesSchema.h"
#include "pxr/imaging/hd/basisCurvesTopologySchema.h"
#include "pxr/imaging/hd/meshSchema.h"
#include "pxr/imaging/hd/meshTopologySchema.h"
#include "pxr/imaging/hd/primvarsSchema.h"
#include "pxr/imaging/hd/retainedDataSource.h"
#include "pxr/imaging/hd/tokens.h"

#include <opensubdiv/far/patchMap.h>
#include <opensubdiv/far/patchTableFactory.h>
#include <opensubdiv/far/primvarRefiner.h>
#include <opensubdiv/far/topologyDescriptor.h>
#include <iostream>
#include <numeric>

PXR_NAMESPACE_USING_DIRECTIVE

TF_DEFINE_PRIVATE_TOKENS(_tokens,
                         (child)             //
                         (sourceMeshPath)    //
                         (numSamplesPerFace) //
                         (length)            //
);

namespace {
//------------------------------------------------------------------------------
struct Vertex
{
    Vertex()
        : point(0)
    {
    }
    void Clear(void* = 0) { point.Set(0, 0, 0); }
    void AddWithWeight(Vertex const& src, float weight) { point += weight * src.point; }
    GfVec3f point;
};
struct LimitFrame
{
    void Clear(void* = 0)
    {
        point.Set(0, 0, 0);
        deriv1.Set(0, 0, 0);
        deriv2.Set(0, 0, 0);
    }
    void AddWithWeight(Vertex const& src, float weight, float d1Weight, float d2Weight)
    {
        point += weight * src.point;
        deriv1 += d1Weight * src.point;
        deriv2 += d2Weight * src.point;
    }
    GfVec3f point, deriv1, deriv2;
};

inline int
numQuadFaces(VtIntArray const &faceVertexCounts)
{
    return std::accumulate(faceVertexCounts.begin(), faceVertexCounts.end(), 0, [](int total, int nverts) {
        return total + (nverts == 4 ? 1 : nverts);
    });
}

class _CurveVertexCountsDataSource : public HdIntArrayDataSource
{
public:
    HD_DECLARE_DATASOURCE(_CurveVertexCountsDataSource);

    bool GetContributingSampleTimesForInterval(Time startTime, Time endTime, std::vector<Time>* outSampleTimes) override
    {
        return _faceVertexCountsDs->GetContributingSampleTimesForInterval(startTime, endTime, outSampleTimes) ||
               _numSamplesDs->GetContributingSampleTimesForInterval(startTime, endTime, outSampleTimes);
    }
    VtValue GetValue(Time shutterOffset) { return VtValue(GetTypedValue(shutterOffset)); }
    VtIntArray GetTypedValue(Time shutterOffset)
    {
        VtIntArray faceVertexCounts = _faceVertexCountsDs->GetValue(shutterOffset).UncheckedGet<VtIntArray>();
        int numSamplesPerFace       = int(_numSamplesDs->GetValue(shutterOffset).GetWithDefault<float>(1.0));
        int numSamples              = numSamplesPerFace * numQuadFaces(faceVertexCounts);
        VtIntArray r(numSamples);
        std::fill(r.begin(), r.end(), 2);
        return r;
    }

private:
    _CurveVertexCountsDataSource(HdSampledDataSourceHandle faceVertexCountsDs, HdSampledDataSourceHandle numSamplesDs)
        : _faceVertexCountsDs(faceVertexCountsDs)
        , _numSamplesDs(numSamplesDs)
    {
    }
    HdSampledDataSourceHandle _faceVertexCountsDs, _numSamplesDs;
};

class _CurveIndicesFromDataSource : public HdIntArrayDataSource
{
public:
    HD_DECLARE_DATASOURCE(_CurveIndicesFromDataSource);

    bool GetContributingSampleTimesForInterval(Time startTime, Time endTime, std::vector<Time>* outSampleTimes) override
    {
        return _faceVertexCountsDs->GetContributingSampleTimesForInterval(startTime, endTime, outSampleTimes) ||
               _numSamplesDs->GetContributingSampleTimesForInterval(startTime, endTime, outSampleTimes);
    }
    VtValue GetValue(Time shutterOffset) { return VtValue(GetTypedValue(shutterOffset)); }
    VtIntArray GetTypedValue(Time shutterOffset)
    {
        VtIntArray faceVertexCounts = _faceVertexCountsDs->GetValue(shutterOffset).UncheckedGet<VtIntArray>();
        int numSamplesPerFace       = int(_numSamplesDs->GetValue(shutterOffset).GetWithDefault<float>(1.0f));
        int numSamples              = numSamplesPerFace * numQuadFaces(faceVertexCounts);
        VtIntArray r(2 * numSamples);
        std::iota(r.begin(), r.end(), 0);
        return r;
    }

private:
    _CurveIndicesFromDataSource(HdSampledDataSourceHandle faceVertexCountsDs, HdSampledDataSourceHandle numSamplesDs)
        : _faceVertexCountsDs(faceVertexCountsDs)
        , _numSamplesDs(numSamplesDs)
    {
    }
    HdSampledDataSourceHandle _faceVertexCountsDs, _numSamplesDs;
};

class _CurvePointsFromMeshPointDataSource : public HdVec3fArrayDataSource
{
public:
    HD_DECLARE_DATASOURCE(_CurvePointsFromMeshPointDataSource);

    bool GetContributingSampleTimesForInterval(Time startTime, Time endTime, std::vector<Time>* outSampleTimes) override
    {
        return _pointsDs->GetContributingSampleTimesForInterval(startTime, endTime, outSampleTimes);
    }
    VtValue GetValue(Time shutterOffset) { return VtValue(GetTypedValue(shutterOffset)); }
    VtVec3fArray GetTypedValue(Time shutterOffset)
    {
        if (!_faceVertexCountsDs || !_faceIndicesDs || !_pointsDs)
        {
            return VtVec3fArray();
        }

        VtIntArray faceVertexCounts = _faceVertexCountsDs->GetValue(shutterOffset).UncheckedGet<VtIntArray>();
        VtIntArray faceIndices      = _faceIndicesDs->GetValue(shutterOffset).UncheckedGet<VtIntArray>();
        VtVec3fArray points         = _pointsDs->GetValue(shutterOffset).UncheckedGet<VtVec3fArray>();
        int numSamples              = int(_numSamplesDs->GetValue(shutterOffset).GetWithDefault<float>(1.0f));
        float length                = _lengthDs->GetValue(shutterOffset).GetWithDefault<float>(0.1f);

        if (_faceIndices != faceIndices)
        {
            _CreateRefiner(faceVertexCounts, faceIndices, points);
        }

        int nRefinerVertices = _refiner->GetNumVerticesTotal();
        int nLocalPoints     = _patchTable->GetNumLocalPoints();
        std::vector<Vertex> verts(nRefinerVertices + nLocalPoints);
        std::memcpy(&verts[0], points.cdata(), points.size() * 3 * sizeof(float));
        int nRefinedLevels = _refiner->GetNumLevels();
        OpenSubdiv::Far::PrimvarRefinerReal<float> primvarRefiner(*_refiner);

        Vertex* src = &verts[0];
        for (int level = 1; level < nRefinedLevels; ++level)
        {
            Vertex* dst = src + _refiner->GetLevel(level - 1).GetNumVertices();
            primvarRefiner.Interpolate(level, src, dst);
            src = dst;
        }
        if (nLocalPoints)
        {
            _patchTable->GetLocalPointStencilTable<float>()->UpdateValues(&verts[0], &verts[nRefinerVertices]);
        }
        OpenSubdiv::Far::PatchMap patchmap(*_patchTable);
        int nfaces = numQuadFaces(faceVertexCounts);
        std::vector<LimitFrame> samples(numSamples * nfaces);
        srand(static_cast<int>(1));
        float pWeights[20], dsWeights[20], dtWeights[20];

        for (int face = 0, count = 0; face < nfaces; ++face)
        {
            for (int sample = 0; sample < numSamples; ++sample, ++count)
            {
                float s = (float)rand() / (float)RAND_MAX;
                float t = (float)rand() / (float)RAND_MAX;

                OpenSubdiv::Far::PatchTable::PatchHandle const* handle = patchmap.FindPatch(face, s, t);
                // ここまでキャッシュ可
                _patchTable->EvaluateBasis(*handle, s, t, pWeights, dsWeights, dtWeights);
                OpenSubdiv::Far::ConstIndexArray cvs = _patchTable->GetPatchVertices(*handle);
                LimitFrame& dst                      = samples[count];
                dst.Clear();
                for (int cv = 0; cv < cvs.size(); ++cv)
                {
                    dst.AddWithWeight(verts[cvs[cv]], pWeights[cv], dsWeights[cv], dtWeights[cv]);
                }
            }
        }

        VtVec3fArray r;
        r.reserve(samples.size() * 2);
        for (size_t i = 0; i < samples.size(); ++i)
        {
            GfVec3f n = samples[i].deriv1 ^ samples[i].deriv2;
            n.Normalize();
            r.emplace_back(samples[i].point);
            r.emplace_back(samples[i].point + length * n);
        }
        return r;
    }
    void Update(HdSampledDataSourceHandle pointsDs,
                HdSampledDataSourceHandle faceVertexCountsDs,
                HdSampledDataSourceHandle faceIndicesDs,
                HdSampledDataSourceHandle numSampleDs,
                HdSampledDataSourceHandle lengthDs)
    {
        _pointsDs           = pointsDs;
        _faceVertexCountsDs = faceVertexCountsDs;
        _faceIndicesDs      = faceIndicesDs;
        _numSamplesDs       = numSampleDs;
        _lengthDs           = lengthDs;
    }

private:
    _CurvePointsFromMeshPointDataSource() {}
    void _CreateRefiner(VtIntArray const& faceVertexCounts, VtIntArray const& faceIndices, VtVec3fArray const& points)
    {
        using namespace OpenSubdiv;
        _faceIndices = faceIndices;

        using Descriptor     = Far::TopologyDescriptor;
        Sdc::SchemeType type = Sdc::SCHEME_CATMARK;
        Sdc::Options options;
        options.SetVtxBoundaryInterpolation(Sdc::Options::VTX_BOUNDARY_EDGE_ONLY);
        Descriptor desc;
        desc.numVertices        = (int)points.size();
        desc.numFaces           = (int)faceVertexCounts.size();
        desc.numVertsPerFace    = faceVertexCounts.cdata();
        desc.vertIndicesPerFace = faceIndices.cdata();
        _refiner.reset(Far::TopologyRefinerFactory<Descriptor>::Create(
            desc, Far::TopologyRefinerFactory<Descriptor>::Options(type, options)));

        Far::PatchTableFactory::Options patchOptions;
        patchOptions.SetPatchPrecision<float>();
        patchOptions.useInfSharpPatch      = true;
        patchOptions.generateVaryingTables = false;
        patchOptions.endCapType            = Far::PatchTableFactory::Options::ENDCAP_GREGORY_BASIS;

        int isolateLevel = 3;
        Far::TopologyRefiner::AdaptiveOptions adaptiveOptions(isolateLevel);
        adaptiveOptions = patchOptions.GetRefineAdaptiveOptions();
        _refiner->RefineAdaptive(adaptiveOptions);
        _patchTable.reset(Far::PatchTableFactory::Create(*_refiner, patchOptions));
    }

    HdSampledDataSourceHandle _pointsDs, _faceVertexCountsDs, _faceIndicesDs, _numSamplesDs, _lengthDs;
    std::unique_ptr<OpenSubdiv::Far::TopologyRefiner> _refiner     = nullptr;
    std::unique_ptr<OpenSubdiv::Far::PatchTable const> _patchTable = nullptr;
    VtIntArray _faceIndices;
};
} // namespace

MyProceduralFur::MyProceduralFur(const SdfPath& proceduralPrimPath)
    : HdGpGenerativeProcedural(proceduralPrimPath)
{
}

HdGpGenerativeProcedural*
MyProceduralFur::New(const SdfPath& proceduralPrimPath)
{
    return new MyProceduralFur(proceduralPrimPath);
}

HdGpGenerativeProcedural::DependencyMap
MyProceduralFur::UpdateDependencies(const HdSceneIndexBaseRefPtr& inputScene)
{
    DependencyMap result;
    HdSceneIndexPrim myPrim   = inputScene->GetPrim(_GetProceduralPrimPath());
    HdPrimvarsSchema primvars = HdPrimvarsSchema::GetFromParent(myPrim.dataSource);
    if (HdSampledDataSourceHandle sourceMeshDs = primvars.GetPrimvar(_tokens->sourceMeshPath).GetPrimvarValue())
    {
        VtValue v = sourceMeshDs->GetValue(0.0f);
        if (v.IsHolding<VtArray<SdfPath>>())
        {
            VtArray<SdfPath> a = v.UncheckedGet<VtArray<SdfPath>>();
            if (a.size() == 1)
            {
                result[a[0]] = HdDataSourceLocatorSet{ HdMeshTopologySchema::GetDefaultLocator(),
                                                       HdPrimvarsSchema::GetPointsLocator() };
            }
        }
    }
    return result;
}

HdGpGenerativeProcedural::ChildPrimTypeMap
MyProceduralFur::Update(const HdSceneIndexBaseRefPtr& inputScene,
                        const ChildPrimTypeMap& previousResult,
                        const DependencyMap& dirtiedDependencies,
                        HdSceneIndexObserver::DirtiedPrimEntries* outputDirtiedPrims)
{
    ChildPrimTypeMap result;
    HdSceneIndexPrim myPrim   = inputScene->GetPrim(_GetProceduralPrimPath());
    HdPrimvarsSchema primvars = HdPrimvarsSchema::GetFromParent(myPrim.dataSource);

    SdfPath sourceMeshPath;
    if (HdSampledDataSourceHandle sourceMeshDs = primvars.GetPrimvar(_tokens->sourceMeshPath).GetPrimvarValue())
    {
        VtValue v = sourceMeshDs->GetValue(0.0f);
        if (v.IsHolding<VtArray<SdfPath>>())
        {
            VtArray<SdfPath> a = v.UncheckedGet<VtArray<SdfPath>>();
            if (a.size() == 1)
            {
                sourceMeshPath = a[0];
            }
        }
    }
    if (sourceMeshPath.IsEmpty())
        return result;

    HdSceneIndexPrim sourceMeshPrim     = inputScene->GetPrim(sourceMeshPath);
    HdPrimvarsSchema sourcePrimvars     = HdPrimvarsSchema::GetFromParent(sourceMeshPrim.dataSource);
    HdMeshTopologySchema sourceTopology = HdMeshSchema::GetFromParent(sourceMeshPrim.dataSource).GetTopology();

    _meshPointsDs           = sourcePrimvars.GetPrimvar(HdPrimvarsSchemaTokens->points).GetPrimvarValue();
    _meshFaceVertexCountsDs = sourceTopology.GetFaceVertexCounts();
    _meshFaceIndicesDs      = sourceTopology.GetFaceVertexIndices();
    _numSampleDs            = primvars.GetPrimvar(_tokens->numSamplesPerFace).GetPrimvarValue();
    _lengthDs               = primvars.GetPrimvar(_tokens->length).GetPrimvarValue();

    SdfPath path      = _GetProceduralPrimPath();
    SdfPath childPath = path.AppendChild(_tokens->child); // Hydra Rprim を一つだけ作る

    // このサンプルではキャッシュは未実装で毎回全部作っているが、本来は個別に更新管理するべき
    result[childPath] = HdPrimTypeTokens->basisCurves;
    if (outputDirtiedPrims)
    {
        HdDataSourceLocatorSet locators;
        locators.append(HdPrimvarsSchema::GetPointsLocator());
        locators.append(HdBasisCurvesTopologySchema::GetDefaultLocator());
        outputDirtiedPrims->emplace_back(childPath, locators);
    }

    return result;
}

HdSceneIndexPrim
MyProceduralFur::GetChildPrim(const HdSceneIndexBaseRefPtr& inputScene, const SdfPath& childPrimPath)
{
    HdSceneIndexPrim result;

    if (!_curvePointsDs)
    {
        _curvePointsDs = _CurvePointsFromMeshPointDataSource::New();
    }
    _CurvePointsFromMeshPointDataSource::Cast(_curvePointsDs)
        ->Update(_meshPointsDs, _meshFaceVertexCountsDs, _meshFaceIndicesDs, _numSampleDs, _lengthDs);

    // Refiner 以外のトポロジ依存情報を毎回作っているのは無駄に思えるので、どうにかしたい

    if (_meshPointsDs)
    {
        // meshPointDs 頂点上にラインを生成する
        result.primType   = HdPrimTypeTokens->basisCurves;
        result.dataSource = HdRetainedContainerDataSource::New(
            HdBasisCurvesSchemaTokens->basisCurves,
            HdBasisCurvesSchema::Builder()
                .SetTopology(
                    HdBasisCurvesTopologySchema::Builder()
                        .SetCurveVertexCounts(_CurveVertexCountsDataSource::New(_meshFaceVertexCountsDs, _numSampleDs))
                        .SetCurveIndices(_CurveIndicesFromDataSource::New(_meshFaceVertexCountsDs, _numSampleDs))
                        .SetBasis(HdRetainedTypedSampledDataSource<TfToken>::New(HdTokens->bezier))
                        .SetType(HdRetainedTypedSampledDataSource<TfToken>::New(HdTokens->linear))
                        .SetWrap(HdRetainedTypedSampledDataSource<TfToken>::New(HdTokens->segmented))
                        .Build())
                .Build(),
            HdPrimvarsSchemaTokens->primvars,
            HdRetainedContainerDataSource::New(
                HdPrimvarsSchemaTokens->points,
                HdPrimvarSchema::Builder()
                    .SetPrimvarValue(_curvePointsDs)
                    .SetInterpolation(HdPrimvarSchema::BuildInterpolationDataSource(HdPrimvarSchemaTokens->vertex))
                    .SetRole(HdPrimvarSchema::BuildRoleDataSource(HdPrimvarSchemaTokens->point))
                    .Build(),
                HdTokens->displayColor,
                HdPrimvarSchema::Builder()
                    .SetPrimvarValue(HdRetainedTypedSampledDataSource<GfVec3f>::New(GfVec3f(0.2f, 0.2f, 0.2f)))
                    .SetInterpolation(HdPrimvarSchema::BuildInterpolationDataSource(HdPrimvarSchemaTokens->constant))
                    .SetRole(HdPrimvarSchema::BuildRoleDataSource(HdPrimvarSchemaTokens->color))
                    .Build()
                ));
    }

    return result;
}
