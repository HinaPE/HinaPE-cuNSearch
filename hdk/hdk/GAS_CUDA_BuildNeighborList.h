#ifndef CUNSEARCH_GAS_CUDA_BUILDNEIGHBORLIST_H
#define CUNSEARCH_GAS_CUDA_BUILDNEIGHBORLIST_H

#include <GAS/GAS_SubSolver.h>
#include <PRM/PRM_Default.h>

class GAS_CUDA_BuildNeighborList : public GAS_SubSolver
{
public:
	static const char *DATANAME;

	GETSET_DATA_FUNCS_F("KernelRadius", KernelRadius)
	GETSET_DATA_FUNCS_S("MainFluidDataIndexGeometrySheetName", MainFluidDataIndexGeometrySheetName)
	GETSET_DATA_FUNCS_S("NeighborSumGeometrySheetName", NeighborSumGeometrySheetName)
	GETSET_DATA_FUNCS_S("NeighborListGeometrySheetName", NeighborListGeometrySheetName)

protected:
	GAS_CUDA_BuildNeighborList(const SIM_DataFactory *factory) : BaseClass(factory) {}
	~GAS_CUDA_BuildNeighborList() override = default;
	bool solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) override;
	void initializeSubclass() override;
	void makeEqualSubclass(const SIM_Data *source) override;
	static const SIM_DopDescription *getDopDescription();

DECLARE_STANDARD_GETCASTTOTYPE();
DECLARE_DATAFACTORY(GAS_CUDA_BuildNeighborList,
					GAS_SubSolver,
					"CUDA_BuildNeighborList",
					getDopDescription());

private:
	bool Solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep, UT_WorkBuffer &error_msg) const;
};

#endif //CUNSEARCH_GAS_CUDA_BUILDNEIGHBORLIST_H
