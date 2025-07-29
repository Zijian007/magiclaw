#!/bin/bash
# -*- coding: utf-8 -*-

set -e

echo "=========================================="
echo "Get Pretrained Models"
echo "=========================================="

# Get the current script directory
script_dir=$(dirname "$(realpath "$0")")

# Define the target folder path
models_dir="$script_dir/models"

# Check if the target folder exists, if not, create it
if [ ! -d "$models_dir" ]; then
  mkdir -p "$models_dir"
fi
mkdir -p "$models_dir"
echo "Folder ready: $models_dir"

# Download the model file
onnx_url="Model file URL"
model_name=$(basename "$onnx_url")
model_path="$models_dir/$model_name"

echo "Downloading model to: $model_path"

# Use curl to download the model and show the progress bar
curl -L "$onnx_url" -o "$model_path" -#

echo "==========================================="
if [ $? -eq 0 ]; then
  echo "Model downloaded successfully: $model_path"
else
  echo "Download failed. Please check the Internet connection."
  echo "Please try again later or check the URL."
fi
echo "==========================================="
