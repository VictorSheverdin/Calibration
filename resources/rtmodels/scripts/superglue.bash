#!/bin/bash


# Default arguments
min_kpts=1
max_kpts=1024
opt_kpts=1024
kpts_name=keyp0,keyp1
score_name=scor0,scor1
descr_name=desc0,desc1


help()
{
    text="Usage: $0
        --onnx <ONNX model path>
        --engine <output engine path>
        --workspace <GPU workspace in megabytes>
        [--trt <TensorRT trtexec path>]
        [--fp16]
        [--verbose]
        [--min_kpts <Minimum keypoint count for dynamic binding size, default: $min_kpts>]
        [--max_kpts <Maximum keypoint count for dynamic binding size, default: $max_kpts>]
        [--opt_kpts <Optimal keypoint count for dynamic binding size, default: $opt_kpts>]
        [--kpts_name <Keypoint bindings names separated by a comma, default: $kpts_name>]
        [--score_name <Score bindings names separated by a comma, default: $score_name>]
        [--descr_name <Descriptor bindings names separated by a comma, default: $descr_name>]"
    text=$(echo $text | tr '\n' ' ')

    echo "SuperGlue engine builder"
    echo "$text"
    echo "By default, searches for trtexec using find in /usr and /opt"
}


# Regular expression to check kpts count
number_regexpr="^[0-9]+$"


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
        --kpts_name)
            kpts_name="$2"
            shift;;
        --score_name)
            score_name="$2"
            shift;;
        --descr_name)
            descr_name="$2"
            shift;;
        --min_kpts)
            min_kpts="$2"
            shift;;
        --max_kpts)
            max_kpts="$2"
            shift;;
        --opt_kpts)
            opt_kpts="$2"
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

# Min keypoints
if ! [[ $min_kpts =~ $number_regexpr ]]
then
    echo "Minimum keypoint count must be a positive integer"
    params_ok=false
fi

# Max keypoints
if ! [[ $max_kpts =~ $number_regexpr ]]
then
    echo "Maximum keypoint count must be a positive integer"
    params_ok=false
fi

# Opt keypoints
if ! [[ $opt_kpts =~ $number_regexpr ]]
then
    echo "Optimal keypoint count must be a positive integer"
    params_ok=false
fi

# Keypoint binding names
kpts_name_list=($(echo $kpts_name | tr "," "\n"))
if [ ${#kpts_name_list[@]} -ne 2 ]
then
    echo "Keypoint binding names format error"
    params_ok=false
fi

# Score binding names
score_name_list=($(echo $score_name | tr "," "\n"))
if [ ${#score_name_list[@]} -ne 2 ]
then
    echo "Score binding names format error"
    params_ok=false
fi

# Descriptor binding names
descr_name_list=($(echo $descr_name | tr "," "\n"))
if [ ${#descr_name_list[@]} -ne 2 ]
then
    echo "Descriptor binding names format error"
    params_ok=false
fi

# $1 is keypoint count
keypoint_shapes()
{
    echo "${kpts_name_list[0]}:${1}x2,${kpts_name_list[1]}:${1}x2"
}

# $1 is keypoint count
score_shapes()
{
    echo "${score_name_list[0]}:1x${1},${score_name_list[1]}:1x${1}"
}

# $1 is keypoint count
descriptor_shapes()
{
    echo "${descr_name_list[0]}:256x${1},${descr_name_list[1]}:256x${1}"
}


if [ "$params_ok" = true ]
then
    min_keypoint_shapes=$(keypoint_shapes ${min_kpts})
    max_keypoint_shapes=$(keypoint_shapes ${max_kpts})
    opt_keypoint_shapes=$(keypoint_shapes ${opt_kpts})

    min_score_shapes=$(score_shapes ${min_kpts})
    max_score_shapes=$(score_shapes ${max_kpts})
    opt_score_shapes=$(score_shapes ${opt_kpts})

    min_descr_shapes=$(descriptor_shapes ${min_kpts})
    max_descr_shapes=$(descriptor_shapes ${max_kpts})
    opt_descr_shapes=$(descriptor_shapes ${opt_kpts})

    min_shapes="${min_keypoint_shapes},${min_score_shapes},${min_descr_shapes}"
    max_shapes="${max_keypoint_shapes},${max_score_shapes},${max_descr_shapes}"
    opt_shapes="${opt_keypoint_shapes},${opt_score_shapes},${opt_descr_shapes}"
    shape_arguments="--minShapes=${min_shapes} --maxShapes=${max_shapes} --optShapes=${opt_shapes}"

    $trt_executable --buildOnly --onnx=$onnx --saveEngine=$engine --workspace=$workspace $shape_arguments $verbose_flag $fp16_flag 
fi
