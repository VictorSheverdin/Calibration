#ifndef _SUPER_GLUE_HPP_
#define _SUPER_GLUE_HPP_

#include "tensor_rt_utils.hpp"


namespace marker
{
    namespace utils = tensor_rt_utils;

    class SuperGlue : public utils::EngineWrapper
    {       
    public:
        static inline std::string NAME = "SuperGlue";
        static constexpr int IMAGE_COUNT = 2;

        static constexpr std::array<int, IMAGE_COUNT> DESCR_BINDING_INDEX = { 0, 1 };
        static constexpr auto DESCR_LAYOUT = utils::BlockLayout::DEPTH_FIRST;
        static constexpr std::array<int, IMAGE_COUNT> KEYPOINTS_BINDING_INDEX = { 2, 3 };
        static constexpr auto KEYPOINTS_LAYOUT = utils::BlockLayout::COUNT_FIRST;
        static constexpr std::array<int, IMAGE_COUNT> SCORES_BINDING_INDEX = { 4, 5 };
        static constexpr auto SCORES_LAYOUT = utils::BlockLayout::DEPTH_FIRST;
        static constexpr int KEYPOINT_COUNT_BINDING_INDEX = 6;
        static constexpr int MATCH_BINDING_INDEX = 7;
        

        using DescrDims = utils::DimsBlock<DESCR_LAYOUT>;
        using KeypointsDims = utils::DimsBlock<KEYPOINTS_LAYOUT>;
        using ScoresDims = utils::DimsBlock<SCORES_LAYOUT>;


        SuperGlue(utils::EnginePtr&& engine, utils::Logger& logger)
            : utils::EngineWrapper(NAME, std::move(engine), bindings(), logger)
        {
            //  Check dynamic shape bindings requirements
            //  { Min, max, opt } counts must be the same for keypoints, descr and scores
            int profile = this->context().getOptimizationProfile();
            std::array<utils::ShapeSpec, IMAGE_COUNT> descr_specs;
            std::transform(DESCR_BINDING_INDEX.begin(), DESCR_BINDING_INDEX.end(), descr_specs.begin(), 
                [&](int bi) { return get_count_spec(profile, bi, DescrDims::count_axis()); });
            std::array<utils::ShapeSpec, IMAGE_COUNT> keypoints_specs;
            std::transform(KEYPOINTS_BINDING_INDEX.begin(), KEYPOINTS_BINDING_INDEX.end(), keypoints_specs.begin(), 
                [&](int bi) { return get_count_spec(profile, bi, KeypointsDims::count_axis()); });
            std::array<utils::ShapeSpec, IMAGE_COUNT> scores_specs;
            std::transform(SCORES_BINDING_INDEX.begin(), SCORES_BINDING_INDEX.end(), scores_specs.begin(), 
                [&](int bi) { return get_count_spec(profile, bi, ScoresDims::count_axis()); });
            for(int i = 0; i < IMAGE_COUNT; ++i)
            {
                auto str_i = std::to_string(i);
                utils::check_value(NAME + " count specs: descr vs keypoints " + str_i, 
                    descr_specs[i], keypoints_specs[i], logger, "Engine binding error");
                utils::check_value(NAME + " count specs: keypoints vs scores " + str_i, 
                    keypoints_specs[i], scores_specs[i], logger, "Engine binding error");
            }

            //  If count specs are equal then save one of them
            m_count_spec = keypoints_specs;

            //  Used in resize_bindings() - only count dimension is changed so can ignore [1]
            m_descr_dims = binding_dims(DESCR_BINDING_INDEX[0]);
            m_keypoints_dims = binding_dims(KEYPOINTS_BINDING_INDEX[0]);
            m_scores_dims = binding_dims(SCORES_BINDING_INDEX[0]);
        }

        DescrDims descr_dims(int index) const
        {   
            utils::Dims dims = binding_dims(DESCR_BINDING_INDEX[index]);
            return DescrDims(dims);
        }

        KeypointsDims keypoints_dims(int index) const
        {
            utils::Dims dims = binding_dims(KEYPOINTS_BINDING_INDEX[index]);
            return KeypointsDims(dims);
        }

        ScoresDims scores_dims(int index) const
        {
            utils::Dims dims = binding_dims(SCORES_BINDING_INDEX[index]);
            return ScoresDims(dims);
        }

        utils::Dims2d keypoint_count_dims() const
        {
            return utils::Dims2d(binding_dims(KEYPOINT_COUNT_BINDING_INDEX));
        }

        utils::Dims3d match_dims() const
        {
            return utils::Dims3d(binding_dims(MATCH_BINDING_INDEX));
        }

        const std::array<utils::ShapeSpec, IMAGE_COUNT>& count_spec() const
        {
            return m_count_spec;
        }

        bool resize_bindings(int index, int count)
        {
            bool success = count >= m_count_spec[index].minval && count <= m_count_spec[index].maxval;
            if (!success)
                return false;

            m_descr_dims.d[DescrDims::count_axis()] = count;
            success = context().setBindingDimensions(DESCR_BINDING_INDEX[index], m_descr_dims);
            if (!success)
                return false;

            m_keypoints_dims.d[KeypointsDims::count_axis()] = count;
            success = context().setBindingDimensions(KEYPOINTS_BINDING_INDEX[index], m_keypoints_dims);
            if (!success)
                return false;

            m_scores_dims.d[ScoresDims::count_axis()] = count;
            success = context().setBindingDimensions(SCORES_BINDING_INDEX[index], m_scores_dims);
            if (!success)
                return false;

            return true;
        }

        static EngineWrapper::BindingRequirements bindings()
        {
            BindingRequirements ret(8);

            //  Descriptors
            for(int i: DESCR_BINDING_INDEX)
            {
                ret[i]
                    .is_input(true)
                    .dims({ -1, -1 })
                    .dtype(utils::DataType::kFLOAT);
            }

            //  Keypoints
            for(int i: KEYPOINTS_BINDING_INDEX)
            {
                ret[i]
                    .is_input(true)
                    .dims({ -1, 2 })
                    .dtype(utils::DataType::kFLOAT);
            }

            //  Keypoint scores
            for(int i: SCORES_BINDING_INDEX)
            {
                ret[i]
                    .is_input(true)
                    .dims({ 1, -1 })
                    .dtype(utils::DataType::kFLOAT);
            }

            //  Keypoint counts
            ret[KEYPOINT_COUNT_BINDING_INDEX]
                .is_input(true)
                .dims({ 1, 2})
                .dtype(utils::DataType::kFLOAT);

            //  Match scores
            ret[MATCH_BINDING_INDEX]
                .is_input(false)
                .dims({ 1, -1, -1 })
                .dtype(utils::DataType::kFLOAT);

            

            return ret;
        }

        utils::ShapeSpec get_count_spec(int profile, int binding_index, int count_axis)
        {
            using ProfileDimOpt = nvinfer1::OptProfileSelector;
            utils::ShapeSpec ret;
            ret.minval = this->engine().getProfileDimensions(
                binding_index, profile, ProfileDimOpt::kMIN).d[count_axis];    
            ret.maxval = this->engine().getProfileDimensions(
                binding_index, profile, ProfileDimOpt::kMAX).d[count_axis];
            ret.optval = this->engine().getProfileDimensions(
                binding_index, profile, ProfileDimOpt::kOPT).d[count_axis];
            return ret;
        }

    private:
        utils::Dims m_keypoints_dims;
        utils::Dims m_descr_dims;
        utils::Dims m_scores_dims;

        std::array<utils::ShapeSpec, IMAGE_COUNT> m_count_spec;
    };
}

#endif // _SUPER_GLUE_HPP_