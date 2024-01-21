#include <UT/UT_DSOVersion.h> // Very Important!!! Include this!!!

#include <hdk/GAS_CUDA_BuildNeighborList.h>

void initializeSIM(void *)
{
	IMPLEMENT_DATAFACTORY(GAS_CUDA_BuildNeighborList)
}
