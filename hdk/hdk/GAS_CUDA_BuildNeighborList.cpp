#include "GAS_CUDA_BuildNeighborList.h"

#include <SIM/SIM_Engine.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_GuideShared.h>
#include <SIM/SIM_ColliderLabel.h>
#include <SIM/SIM_ForceGravity.h>
#include <SIM/SIM_Time.h>
#include <SIM/SIM_Utils.h>

#include <PRM/PRM_Name.h>
#include <PRM/PRM_Template.h>
#include <PRM/PRM_Shared.h>
#include <PRM/PRM_Default.h>
#include <PRM/PRM_Utils.h>
#include <PRM/PRM_SpareData.h>

#include <UT/UT_WorkBuffer.h>
#include <UT/UT_NetMessage.h>

#include "cuNSearch.h"

bool GAS_CUDA_BuildNeighborList::solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	UT_WorkBuffer error_msg;
	if (!Solve(engine, obj, time, timestep, error_msg) || UTisstring(error_msg.buffer()))
	{
		SIM_Data::addError(obj, SIM_MESSAGE, error_msg.buffer(), UT_ERROR_ABORT);
		return false;
	}

	return true;
}

void GAS_CUDA_BuildNeighborList::initializeSubclass()
{
	SIM_Data::initializeSubclass();
}

void GAS_CUDA_BuildNeighborList::makeEqualSubclass(const SIM_Data *source)
{
	SIM_Data::makeEqualSubclass(source);
}

const char *GAS_CUDA_BuildNeighborList::DATANAME = "CUDA_BuildNeighborList";
const SIM_DopDescription *GAS_CUDA_BuildNeighborList::getDopDescription()
{
	static PRM_Name KernelRadius("KernelRadius", "KernelRadius");
	static PRM_Default KernelRadiusDefault{.036};

	static PRM_Name MainFluidDataIndexGeometrySheetName("MainFluidDataIndexGeometrySheetName", "MainFluidDataIndexGeometrySheetName");
	static PRM_Default MainFluidDataIndexGeometrySheetNameDefault{0, "CL_PT_IDX"};

	static PRM_Name NeighborSumGeometrySheetName("NeighborSumGeometrySheetName", "NeighborSumGeometrySheetName");
	static PRM_Default NeighborSumGeometrySheetNameDefault{0, "neighbor_sum"};

	static PRM_Name NeighborListGeometrySheetName("NeighborListGeometrySheetName", "NeighborListGeometrySheetName");
	static PRM_Default NeighborListGeometrySheetNameDefault{0, "neighbors"};

	static std::array<PRM_Template, 5> PRMS{
			PRM_Template(PRM_FLT, 1, &KernelRadius, &KernelRadiusDefault),
			PRM_Template(PRM_STRING, 1, &MainFluidDataIndexGeometrySheetName, &MainFluidDataIndexGeometrySheetNameDefault),
			PRM_Template(PRM_STRING, 1, &NeighborSumGeometrySheetName, &NeighborSumGeometrySheetNameDefault),
			PRM_Template(PRM_STRING, 1, &NeighborListGeometrySheetName, &NeighborListGeometrySheetNameDefault),
			PRM_Template()
	};

	static SIM_DopDescription DESC(true,
								   "cuda_build_neighbor_lists",
								   "CUDA Build Neighbor Lists",
								   DATANAME,
								   classname(),
								   PRMS.data());
//	DESC.setDefaultUniqueDataName(true);
	setGasDescription(DESC);
	return &DESC;
}

using namespace cuNSearch;
using Real3 = std::array<Real, 3>;

inline Real3 operator-(const Real3 &left, const Real3 &right)
{
	return Real3{left[0] - right[0], left[1] - right[1], left[2] - right[2]};
}

bool GAS_CUDA_BuildNeighborList::Solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep, UT_WorkBuffer &error_msg) const
{
	if (!obj)
	{
		error_msg.appendSprintf("Object Is Null, From %s\n", DATANAME);
		return false;
	}

	SIM_GeometryCopy *geo = SIM_DATA_GET(*obj, SIM_GEOMETRY_DATANAME, SIM_GeometryCopy);
	if (!geo)
	{
		error_msg.appendSprintf("Geometry Is Null, From %s\n", DATANAME);
		return false;
	}

	double radius = getKernelRadius();
	NeighborhoodSearch nsearch(radius);

	{
		SIM_GeometryAutoWriteLock lock(geo);
		GU_Detail &gdp = lock.getGdp();
		GA_RWHandleI gdp_handle_neighbor_sum = gdp.findPointAttribute(getNeighborSumGeometrySheetName());
		GA_RWHandleIA gdp_handle_neighbor_list = gdp.findPointAttribute(getNeighborListGeometrySheetName());
		GA_RWHandleI gdp_handle_CL_PT_IDX = gdp.findPointAttribute(getMainFluidDataIndexGeometrySheetName());
		GA_RWHandleV3 gdp_handle_pos = gdp.getP();

		if (gdp_handle_neighbor_sum.isInvalid())
		{
			error_msg.appendSprintf("NeighborSumGeometrySheetName INDALID: %s, From %s\n", getNeighborSumGeometrySheetName().c_str(), DATANAME);
			return false;
		}
		if (gdp_handle_neighbor_list.isInvalid())
		{
			error_msg.appendSprintf("NeighborListGeometrySheetName INDALID: %s, From %s\n", getNeighborListGeometrySheetName().c_str(), DATANAME);
			return false;
		}
		if (gdp_handle_CL_PT_IDX.isInvalid())
		{
			error_msg.appendSprintf("MainFluidDataIndexGeometrySheetName INDALID: %s, From %s\n", getMainFluidDataIndexGeometrySheetName().c_str(), DATANAME);
			return false;
		}

		std::vector<Real3> positions;
		int p_size = gdp.getNumPoints();
		positions.resize(p_size);

		GA_Offset pt_off;
		{
			GA_FOR_ALL_PTOFF(&gdp, pt_off)
				{
					int cl_index = gdp_handle_CL_PT_IDX.get(pt_off);
					UT_Vector3 pos = gdp_handle_pos.get(pt_off);
					positions[cl_index][0] = pos.x();
					positions[cl_index][1] = pos.y();
					positions[cl_index][2] = pos.z();
				}
		}

		auto pointSetIndex = nsearch.add_point_set(positions.front().data(), positions.size(), true, true);
		nsearch.find_neighbors();

		auto &pointSet = nsearch.point_set(pointSetIndex);

//		// Validate results
//		auto points = pointSet.GetPoints();
//
//		for (unsigned int i = 0; i < pointSet.n_points(); i++)
//		{
//			Real3 point = ((Real3 *) points)[i];
//			auto count = pointSet.n_neighbors(0, i);
//			for (unsigned int j = 0; j < count; j++)
//			{
//				auto neighbor = pointSet.neighbor(0, i, j);
//				auto diff = point - ((Real3 *) points)[neighbor];
//				float squaredLength = diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2];
//				float distance = sqrt(squaredLength);
//
//				if (distance > radius)
//					std::cout << "Not a neighbor" << "\n";
//			}
//		}

		{
			GA_FOR_ALL_PTOFF(&gdp, pt_off)
				{
					int cl_index = gdp_handle_CL_PT_IDX.get(pt_off);
					auto neighbor_count = pointSet.n_neighbors(pointSetIndex, cl_index);
					gdp_handle_neighbor_sum.set(pt_off, neighbor_count);

					UT_Int32Array nArray;
					nArray.setSize(neighbor_count);
					for (int nidx = 0; nidx < neighbor_count; ++nidx)
						nArray[nidx] = pointSet.neighbor(pointSetIndex, cl_index, nidx);
					gdp_handle_neighbor_list.set(pt_off, nArray);
				}
		}
	}

	return true;
}
