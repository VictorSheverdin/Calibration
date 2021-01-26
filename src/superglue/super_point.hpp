#ifndef _SUPER_POINT_HPP_
#define _SUPER_POINT_HPP_

#include "tensor_rt_utils.hpp"


namespace marker
{
    class SuperPoint : public tensor_rt_utils::EngineWrapper
    {       
    public:
        static constexpr auto NAME = "SuperPoint";

        static constexpr int IMAGES_BINDING_INDEX = 0;
        static constexpr int IMAGES_BATCH_SIZE = 2;
        static constexpr int IMAGES_CHANNEL_COUNT = 1;

        static constexpr int SCORES_BINDING_INDEX = 1;
        static constexpr int SCORES_BATCH_SIZE = 2;
        static constexpr int SCORES_CHANNEL_COUNT = 1;

        static constexpr int DESCRS_BINDING_INDEX = 2;
        static constexpr int DESCRS_BATCH_SIZE = 2;
        static constexpr int DESCRS_CHANNEL_COUNT = 256;

    public:
        SuperPoint(tensor_rt_utils::EnginePtr&& engine, tensor_rt_utils::Logger& logger)
            : tensor_rt_utils::EngineWrapper(NAME, std::move(engine), bindings(), logger)
        {}

        tensor_rt_utils::Dims4d images_dims() const
        {
            return tensor_rt_utils::Dims4d(binding_dims(IMAGES_BINDING_INDEX));
        }

        tensor_rt_utils::Dims4d scores_dims() const
        {
            return tensor_rt_utils::Dims4d(binding_dims(SCORES_BINDING_INDEX));
        }

        tensor_rt_utils::Dims4d descrs_dims() const
        {
            return tensor_rt_utils::Dims4d(binding_dims(DESCRS_BINDING_INDEX));
        }

        static EngineWrapper::BindingRequirements bindings()
        {
            BindingRequirements ret(3);

            //  Images
            ret[IMAGES_BINDING_INDEX]
                .is_input(true)
                .dims({ IMAGES_BATCH_SIZE, IMAGES_CHANNEL_COUNT, -1, -1 })
                .dtype(tensor_rt_utils::DataType::kFLOAT);

            //  Scores
            ret[SCORES_BINDING_INDEX]
                .is_input(false)
                .dims({ SCORES_BATCH_SIZE, SCORES_CHANNEL_COUNT, -1, -1 })
                .dtype(tensor_rt_utils::DataType::kFLOAT);

            //  Descriptors
            ret[DESCRS_BINDING_INDEX]
                .is_input(false)
                .dims({ DESCRS_BATCH_SIZE, DESCRS_CHANNEL_COUNT, -1, -1 })
                .dtype(tensor_rt_utils::DataType::kFLOAT);

            return ret;
        }
    };
}

#endif // _SUPER_POINT_HPP_