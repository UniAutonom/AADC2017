#!/bin/bash

export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim

GPU_IDX=0
EXPERIMENT_NAME="irgendein_name"
EXPERIMENT_NO="01"
CONFIG_PATH="object_detection/VOCdevkit/VOC2017/ImageSets/Main"

CONFIG_FILE="object_detection/samples/configs/ssd_mobilenet_v1_pets.config"

TRAIN_DIR="${CONFIG_PATH}/adult_train"
EVAL_DIR="${CONFIG_PATH}/adult_val"

export CUDA_VISIBLE_DEVICES="${GPU_IDX}"


LOG_NAME="log.txt"
ARGS="object_detection/train.py --logtostderr --pipeline_config_path=${CONFIG_FILE} --train_dir=${TRAIN_DIR}"

printf "Launching training...\n${ARGS}\nLogging to ${LOG_NAME}...\n\n"
nohup python ${ARGS} >${LOG_NAME} 2>&1 &
