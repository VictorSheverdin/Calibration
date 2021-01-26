/*!
    \file
    \brief Заголовочный файл. содержащий утилиты TensorRT
*/
#pragma once

#include <string>
#include <fstream>
#include <ostream>
#include <vector>
#include <stdexcept>
#include <memory>

#include "NvInfer.h"
#include "NvOnnxParser.h"

#include "cuda_runtime.h"


namespace tensor_rt_utils
{
    /*! 
        Deleter для std::unique_ptr на объекты классов 
        TensorRT, имеющих метод destroy()
    */
    template<typename NvObjT>
    struct NvObjDeleter
    {
        void operator()(NvObjT *ptr)
        {
            ptr->destroy();
        }
    };

    template<typename NvObjT>
    using NvObjPtr = std::unique_ptr<NvObjT, NvObjDeleter<NvObjT>>;

    using Engine = nvinfer1::ICudaEngine;
    using EnginePtr = NvObjPtr<Engine>;

    using Context = nvinfer1::IExecutionContext;
    using ContextPtr = NvObjPtr<Context>;

    using Builder = nvinfer1::IBuilder;
    using BuilderPtr = NvObjPtr<Builder>;
    
    using BuilderCfg = nvinfer1::IBuilderConfig;
    using BuilderCfgPtr = NvObjPtr<BuilderCfg>;

    using NetworkDef = nvinfer1::INetworkDefinition;
    using NetworkDefPtr =  NvObjPtr<NetworkDef>;

    using OnnxParser = nvonnxparser::IParser;
    using OnnxParserPtr = NvObjPtr<OnnxParser>;

    using Logger = nvinfer1::ILogger;

    using PluginFactory = nvinfer1::IPluginFactory;

    using Dims = nvinfer1::Dims;
    using DataType = nvinfer1::DataType;

    struct ShapeSpec
    {
        int minval;
        int maxval;
        int optval;
    };

    inline std::ostream& operator<<(std::ostream& out, const ShapeSpec& spec)
    {
        out << "ShapeSpec[ "
            << "min=" << spec.minval 
            << "; max=" << spec.maxval 
            << "; opt=" << spec.optval << "]";
        return out;
    }

    inline bool operator==(const ShapeSpec& first, const ShapeSpec& second)
    {
        return first.minval == second.minval 
            && first.maxval == second.maxval 
            && first.optval == second.optval;
    }

    inline bool operator!=(const ShapeSpec& first, const ShapeSpec& second)
    {
        return !(first == second);
    }

    struct Dims2d
    {
        int rows;
        int cols;

        Dims2d() = default;

        Dims2d(const Dims& dims)
            : rows(dims.d[0])
            , cols(dims.d[1])
        {}

        Dims2d(int rows, int cols)
            : rows(rows)
            , cols(cols)
        {}

        void get_dims(Dims& dst) const
        {
            dst.nbDims = 2;
            dst.d[0] = rows;
            dst.d[1] = cols;
        }
    };

    struct Dims3d
    {
        int chan;
        int rows;
        int cols;

        Dims3d() = default;

        Dims3d(const Dims& dims)
            : chan(dims.d[0])
            , rows(dims.d[1])
            , cols(dims.d[2])
        {}

        Dims3d(int chan, int rows, int cols)
            : chan(chan)
            , rows(rows)
            , cols(cols)
        {}

        void get_dims(Dims& dst) const
        {
            dst.nbDims = 3;
            dst.d[0] = chan;
            dst.d[1] = rows;
            dst.d[2] = cols;
        }
    };

    struct Dims4d
    {
        int batch;
        int chan;
        int rows;
        int cols;

        Dims4d() = default;

        Dims4d(const Dims& dims)
            : batch(dims.d[0])
            , chan(dims.d[1])
            , rows(dims.d[2])
            , cols(dims.d[3])
        {}

        Dims4d(int batch, int chan, int rows, int cols)
            : batch(batch)
            , chan(chan)
            , rows(rows)
            , cols(cols)
        {}

        void get_dims(Dims& dst) const
        {
            dst.nbDims = 4;
            dst.d[0] = batch;
            dst.d[1] = chan;
            dst.d[2] = rows;
            dst.d[3] = cols;
        }
    };
    
    enum class BlockLayout
    {
        COUNT_FIRST,
        DEPTH_FIRST
    };

    template<BlockLayout layout>
    struct DimsBlock
    {
        static constexpr BlockLayout layout_val = layout;

        int count;
        int depth;

        DimsBlock() = default;

        DimsBlock(const Dims& dims)
            : count(dims.d[count_axis()])
            , depth(dims.d[depth_axis()])
        {}

        DimsBlock(int count, int depth)
            : count(count)
            , depth(depth)
        {}

        static int count_axis()
        {
            if constexpr (layout == BlockLayout::COUNT_FIRST)
                return 0;
            else if constexpr (layout == BlockLayout::DEPTH_FIRST)
                return 1;
            return -1;
        }

        static int depth_axis()
        {
            if constexpr (layout == BlockLayout::COUNT_FIRST)
                return 1;
            else if constexpr (layout == BlockLayout::DEPTH_FIRST)
                return 0;
            return -1;
        }
    };

    inline bool operator==(const Dims& first, const Dims& second)
    {
        if (first.nbDims != second.nbDims)    
            return false;
        return std::equal(first.d, first.d + first.nbDims, second.d);
    }

    inline bool operator!=(const Dims& first, const Dims& second)
    {
        return !(first == second);
    }

    /*! 
        \brief Обёртка для создания IBuilder
        \param logger Логгер
    */
    inline BuilderPtr create_builder(Logger& logger)
    {
        return BuilderPtr(nvinfer1::createInferBuilder(logger));
    }

    /*! 
        \brief Обёртка для создания IParser
        \param network Сеть, для которой будет создан парсер
        \param logger Логгер 
    */
    inline OnnxParserPtr create_onnx_parser(NetworkDef& network, Logger& logger)
    {
        return OnnxParserPtr(nvonnxparser::createParser(network, logger));
    }

    /*! 
        \brief Собрать engine
        \param builder IBuilder для сборки engine
        \param network Модель сети
        \param batch_size Размер пакета
        \param builder_memory Доля используемой при сборке памяти видеокарты
        \param use_fp16 Собирать engine в режиме FP16
    */
    inline EnginePtr build_engine(Builder& builder, NetworkDef& network, 
        int batch_size, float builder_memory, bool use_fp16)
    {
        cudaDeviceProp prop;
        cudaGetDeviceProperties(&prop, 0);

        builder.setMaxBatchSize(batch_size);
        NvObjPtr<nvinfer1::IBuilderConfig> config(builder.createBuilderConfig());
        config->setMaxWorkspaceSize((std::size_t)(prop.totalGlobalMem * builder_memory));
        if(use_fp16)
            config->setFlag(nvinfer1::BuilderFlag::kFP16);
        
        return EnginePtr(builder.buildEngineWithConfig(network, *config));
    }

    /*!
        \brief Загрузить engine из файла
        \param path Путь к файлу
        \param logger Логгер
        \param plugin_factory Фабрика плагинов
    */
    inline EnginePtr load_engine(
        const std::string& path, Logger& logger, 
        PluginFactory* plugin_factory = nullptr)
    {
        NvObjPtr<nvinfer1::IRuntime> runtime(
            nvinfer1::createInferRuntime(logger));
        std::ifstream in(path, std::ios::binary | std::ios::ate);
		if(!in)
			throw std::runtime_error("Cannot open engine file " + path);
		auto buffer_size = in.tellg();
		std::vector<char> buffer(buffer_size);
		in.seekg(std::ios::beg);
		in.read(buffer.data(), buffer.size());
		return EnginePtr(runtime->deserializeCudaEngine(
            buffer.data(), buffer_size, plugin_factory));   
    }

    /*!
        \brief Сохранить engine в файле
        \param path Путь к файлу
        \param engine engine для сохранения
    */
    inline void save_engine(const std::string& path, 
        const nvinfer1::ICudaEngine& engine)
    {
        NvObjPtr<nvinfer1::IHostMemory> serialized(engine.serialize());
		std::ofstream out(path, std::ios::binary);

		if (!out)
			throw std::runtime_error("Cannot access output path");

		out.write(static_cast<char*>(serialized->data()), serialized->size());
    }
    
    template<typename T, typename U>
    void check_value(const std::string& name, T value, U required, 
        Logger& logger, const std::string& exception_text)
    {
        if (value != required)
        {
            std::stringstream err;
            err << name << ": " << value << "; required " << required;
            logger.log(Logger::Severity::kERROR, err.str().data());
            throw std::runtime_error(exception_text);
        }
    }

    class EngineWrapper
    {
    public:
        class BindingRequirement
        {
        public:
            bool is_input() const
            {
                return m_is_input;
            }

            const std::vector<int>& dims() const
            {
                return m_dims;
            }

            DataType dtype() const
            {
                return m_dtype;
            }

            BindingRequirement& is_input(bool value)
            {
                m_is_input = value;
                return *this;
            }

            BindingRequirement& dims(const std::vector<int>& value)
            {
                m_dims = value;
                return *this;
            }

            BindingRequirement& dtype(DataType value)
            {
                m_dtype = value;
                return *this;
            }

        private:
            bool m_is_input;
            std::vector<int> m_dims;
            DataType m_dtype;
        };

        using BindingRequirements = std::vector<BindingRequirement>;

    public:
        EngineWrapper(const std::string& name, EnginePtr&& engine, const BindingRequirements& bindings, Logger& logger)
            : m_name(name)
            , m_engine(std::move(engine))
            , m_context(m_engine->createExecutionContext())
        {
            int binding_count = m_engine->getNbBindings();
            check_value(m_name + " engine binding count", binding_count, 
                bindings.size(), logger, "Engine binding error");

            for(int i = 0; i < binding_count; ++i)
                check_binding(i, bindings[i], logger);
        }

        void check_binding(int index, const BindingRequirement& required, Logger& logger)
        {
            auto index_str = std::to_string(index);

            bool is_input = m_engine->bindingIsInput(index);
            check_value(m_name + " binding " + index_str + " is input", 
                is_input, required.is_input(), logger, "Engine binding error");

            Dims dims = m_engine->getBindingDimensions(index);
            check_value(m_name + " binding " + index_str + " dim count", 
                dims.nbDims, required.dims().size(), logger, "Engine binding error");

            for(int i = 0; i < dims.nbDims; ++i)
            {
                if(required.dims()[i] != -1)
                {
                    check_value(m_name + " binding " + index_str + " dim " + std::to_string(i),
                        dims.d[i], required.dims()[i], logger, "Engine binding error");
                }
            }
        }

        Dims binding_dims(int binding_index) const
        {
            return m_engine->getBindingDimensions(binding_index);
        }

        void enqueue(void** bindings, cudaStream_t cuda_stream, cudaEvent_t* input_consumed)
        {
            m_context->enqueueV2(bindings, cuda_stream, input_consumed);
        }

        void execute(void** bindings)
        {
            m_context->executeV2(bindings);
        }

        Engine& engine()
        {
            return *m_engine;
        }

        const Engine& engine() const
        {
            return *m_engine;
        }

        Context& context()
        {
            return *m_context;
        }

        const Context& context() const
        {
            return *m_context;
        }

        const std::string& name() const
        {
            return m_name;
        }

    private:
        std::string m_name;
        EnginePtr m_engine;
        ContextPtr m_context;
    };

    //! Вывод уровня логгирования в std::ostream
    inline std::ostream& operator<<(std::ostream &out, Logger::Severity severity)
    {
        switch(severity)
        {
            case Logger::Severity::kINTERNAL_ERROR:
                out << "[INTERNAL ERROR]";
                break;
            case Logger::Severity::kERROR:
                out << "[ERROR]";
                break;
            case Logger::Severity::kWARNING:
                out << "[WARNING]";
                break;
            case Logger::Severity::kINFO:
                out << "[INFO]";
                break;
            case Logger::Severity::kVERBOSE:
                out << "[VERBOSE]";
                break;
        }
        return out;
    }

    inline std::ostream& operator<<(std::ostream& out, DataType dtype)
    {
        switch (dtype)
        {
        case DataType::kBOOL:
            out << "BOOL";
            break;
        case DataType::kFLOAT:
            out << "FLOAT";
            break;
        case DataType::kHALF:
            out << "HALF";
            break;
        case DataType::kINT32:
            out << "INT32";
            break;
        case DataType::kINT8:
            out << "INT8";
            break;
        }
        return out;
    }

    inline std::ostream& operator<<(std::ostream& out, Dims dims)
    {
        out << "[";
        for(int i = 0; i < dims.nbDims - 1; ++i)
            out << dims.d[i] << ", ";
        out << dims.d[dims.nbDims - 1] << "]";
        return out;
    }

    /*!
        \class StreamLogger
        \brief Логгер, осуществляющий вывод логов в std::ostream
    */
    class StreamLogger : public Logger
    {
    public:
        /*!
            \brief Конструктор StreamLogger
            \param out Логгирование в данный поток
            \param max_severity Для логгирования сообщение должно иметь статус не менее данного
        */
        StreamLogger(std::ostream &out, Severity max_severity)
            : m_out(out)
            , m_max_severity(max_severity)
        {}

        /*!
            \brief Конструктор StreamLogger
            \param severity Статус сообщения
            \param msg Сообщение
        */
        virtual void log(Severity severity, const char* msg) override
        {
            if (severity <= m_max_severity)
                m_out << severity << ' ' << msg << std::endl;
        }

    private:
        std::ostream &m_out;
        Severity m_max_severity;
    };
}