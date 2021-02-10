#!/bin/bash

help()
{
    text="Usage: $0
        --onnx <ONNX model path>
        --engine <output engine path>
        --workspace <GPU workspace in megabytes>
        [--trt <TensorRT trtexec path>]
        [--fp16]
        [--verbose]"
    text=$(echo $text | tr '\n' ' ')

    echo "SuperPoint engine builder"
    echo "$text"
    echo "By default, searches for trtexec using find in /usr and /opt"
}

# Parse arguments
while [ -n "$1" ]
do
    case "$1" in
        --trt) 
            trt_executable="$2"
            shift;;
        --onnx) 
            onnx="$2"
            shift;;
        --engine) 
            engine="$2"
            shift;;
        --workspace) 
            workspace="$2"
            shift;;
        --verbose) 
            verbose_flag="--verbose";;
        --fp16) 
            fp16_flag="--fp16";;
        --help)
            help
            exit;;
        *) 
            echo "$1 is not an option";;
    esac
    shift
done


params_ok=true

# Check if trt executable path was specified, if not then find it.
if [ -z "$trt_executable" ]
then
    trt_find_results=$(find /usr /opt -name trtexec -type f -executable 2>&1 | grep -v "Permission denied")
    trt_find_count=$(echo "$trt_find_results" | wc -w)

    if [ $trt_find_count -eq 1 ]
    then
        trt_executable=$trt_find_results
        echo "Using $trt_executable"
    else
        echo "Found $trt_find_count TensorRT trtexec executables:"
        echo "$trt_find_results"
        echo "Specify which one to use as --trt <path to trtexec>"
        params_ok=false
    fi
fi

# ONNX model path
if [ -z "$onnx" ]
then
    echo "Must specify ONNX model path for input as --onnx <path>"
    params_ok=false
fi

# Output engine path
if [ -z "$engine" ]
then
    echo "Must specify engine path for output as --engine <path>"
    params_ok=false
fi

# Workspace size
if [ -z "$workspace" ]
then
    echo "Must specify GPU workspace size in megabytes as --workspace <workspace>"
    params_ok=false
fi

if [ "$params_ok" = true ]
then
    $trt_executable --buildOnly --onnx=$onnx --saveEngine=$engine --workspace=$workspace $verbose_flag $fp16_flag 
fi
