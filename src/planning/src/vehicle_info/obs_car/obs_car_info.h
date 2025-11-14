#ifndef OBS_CAR_BASE_H_
#define OBS_CAR_BASE_H_

#include "vehicle_info_base.h"

namespace Planning
{
    class ObsCar : public VehicleBase
    {
    public:
        ObsCar(const int &id);
        void vehicle_cartesian_to_frenet(const Referline &refer_line) override;
        void vehicle_cartesian_to_frenet_2path(const LocalPath &local_path, const Referline &refer_line, const std::shared_ptr<VehicleBase> &car) override;
    };
}

#endif