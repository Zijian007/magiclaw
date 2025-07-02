#!/bin/bash

# Define the target folder path
models_dir="models"

# Check if the target folder exists, if not, create it
if [ ! -d "$models_dir" ]; then
  mkdir "$models_dir"
  echo "Created folder: $models_dir"
else
  echo "Folder $models_dir already exists"
fi

# Download the model file
onnx_url="Model file URL"
model_name="FingerNet.onnx"

echo "Starting to download the model..."

# Use curl to download the model and show the progress bar
curl -L "$onnx_url" -o "$models_dir/$model_name" -#

if [ $? -eq 0 ]; then
  echo "Model downloaded successfully: $models_dir/$model_name"
else
  echo "Model download failed. Please check the Internet connection."
fi
