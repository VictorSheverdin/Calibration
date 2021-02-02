#ifndef _SUPER_POINT_HPP_
#define _SUPER_POINT_HPP_

#include <string>

#include "tensor_rt_utils.hpp"


namespace marker
{
    class SuperPoint : public tensor_rt_utils::EngineWrapper
    {       
    public:
        static inline std::string NAME = "SuperPoint";

        static constexpr int IMAGES_BINDING_INDEX = 0;
        static constexpr int IMAGES_BATCH_SIZE = 1;
        static constexpr int IMAGES_CHANNEL_COUNT = 1;

        static constexpr int SCORES_BINDING_INDEX = 1;
        static constexpr int SCORES_BATCH_SIZE = 1;
        static constexpr int SCORES_CHANNEL_COUNT = 1;

        static constexpr int DESCRS_BINDING_INDEX = 2;
        static constexpr int DESCRS_BATCH_SIZE = 1;
        static constexpr int DESCRS_CHANNEL_COUNT = 256;

    public:
        SuperPoint(tensor_rt_utils::EnginePtr&& engine, tensor_rt_utils::Logger& logger)
            : tensor_rt_utils::EngineWrapper(NAME, std::move(engine), bindings(), logger)
        {
            //  Check dynamic shape bindings requirements
            //  { Min, max, opt } counts must be the same for keypoints, descr and scores
            
            // auto image_spec = get_batch_spec(profile, IMAGES_BINDING_INDEX);
            // auto scores_spec = get_batch_spec(profile, SCORES_BINDING_INDEX);
            // auto descrs_spec = get_batch_spec(profile, DESCRS_BINDING_INDEX);

            // tensor_rt_utils::check_value(NAME + " batch specs: image vs scores", 
            //     image_spec, scores_spec, logger, "Engine binding error");
            // tensor_rt_utils::check_value(NAME + " batch specs: scores vs descr", 
            //     scores_spec, descrs_spec, logger, "Engine binding error");

            // int profile = this->context().getOptimizationProfile();
            // m_batch_spec = get_batch_spec(profile, IMAGES_BINDING_INDEX);

            m_image_dims = binding_dims(IMAGES_BINDING_INDEX);
            m_scores_dims = binding_dims(SCORES_BINDING_INDEX);
            m_descrs_dims = binding_dims(DESCRS_BINDING_INDEX);
        }

        tensor_rt_utils::Dims4d images_dims() const
        {
            return tensor_rt_utils::Dims4d(m_image_dims);
        }

        tensor_rt_utils::Dims4d scores_dims() const
        {
            return tensor_rt_utils::Dims4d(m_scores_dims);
        }

        tensor_rt_utils::Dims4d descrs_dims() const
        {
            return tensor_rt_utils::Dims4d(m_descrs_dims);
        }

        // bool set_batch_size(int size)
        // {
        //     bool success = size >= m_batch_spec.minval && size <= m_batch_spec.maxval;
        //     if (!success)
        //         return false;

        //     m_image_dims.d[0] = size;
        //     success = context().setBindingDimensions(IMAGES_BINDING_INDEX, m_image_dims);
        //     if (!success)
        //         return false;

        //     // m_scores_dims.d[0] = size;
        //     // success = context().setBindingDimensions(SCORES_BINDING_INDEX, m_scores_dims);
        //     // if (!success)
        //     //     return false;

        //     // m_descrs_dims.d[0] = size;
        //     // success = context().setBindingDimensions(DESCRS_BINDING_INDEX, m_descrs_dims);
        //     // if (!success)
        //     //     return false;

        //     return true;
        // }

        // tensor_rt_utils::ShapeSpec image_batch_spec() const
        // {
        //     return m_batch_spec;
        // }

        tensor_rt_utils::ShapeSpec get_batch_spec(int profile, int binding_index)
        {
            using ProfileDimOpt = nvinfer1::OptProfileSelector;
            tensor_rt_utils::ShapeSpec ret;
            ret.minval = this->engine().getProfileDimensions(
                binding_index, profile, ProfileDimOpt::kMIN).d[0];    
            ret.maxval = this->engine().getProfileDimensions(
                binding_index, profile, ProfileDimOpt::kMAX).d[0];
            ret.optval = this->engine().getProfileDimensions(
                binding_index, profile, ProfileDimOpt::kOPT).d[0];
            return ret;
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

        private:
            tensor_rt_utils::Dims m_image_dims;
            tensor_rt_utils::Dims m_scores_dims;
            tensor_rt_utils::Dims m_descrs_dims;

            // tensor_rt_utils::ShapeSpec m_batch_spec;
    };
}

#endif // _SUPER_POINT_HPP_