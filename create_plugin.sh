#!/bin/bash

# Usage: ./create_plugin <plugin_type> <base_name>
# Example: ./create_plugin planner my_test

is_lower_snake_case() {
  [[ "$1" =~ ^[a-z]+(_[a-z]+)*$ ]]
}

usage() {
  echo "Usage: $0 <plugin_type> <base_name>"
  echo "  plugin_type: planner | controller"
  echo "  base_name: lower_snake_case name (e.g., my_test)"
  exit 1
}

if [ $# -ne 2 ]; then
  usage
fi

PLUGIN_TYPE=$1
BASE_NAME=$2

# Validate plugin_type
if [[ "$PLUGIN_TYPE" != "planner" && "$PLUGIN_TYPE" != "controller" ]]; then
  echo "Error: Invalid plugin_type '$PLUGIN_TYPE'. Must be 'planner' or 'controller'."
  usage
fi

# Validate base_name
if ! is_lower_snake_case "$BASE_NAME"; then
  echo "Error: Base name '$BASE_NAME' must be in lower_snake_case (e.g., my_test_plugin)."
  exit 1
fi

PACKAGE_NAME="${BASE_NAME}_${PLUGIN_TYPE}"
PACKAGE_CAMEL_CASE=$(echo "$BASE_NAME" | sed -r 's/(^|_)([a-z])/\U\2/g')$(echo "$PLUGIN_TYPE" | sed -r 's/(^|_)([a-z])/\U\2/g')
TEMPLATE_DIR="${PLUGIN_TYPE}/template_${PLUGIN_TYPE}"
TARGET_DIR=$(echo "$PLUGIN_TYPE")/$PACKAGE_NAME

if [ ! -d "$TEMPLATE_DIR" ]; then
  echo "Error: Template directory '$TEMPLATE_DIR' not found."
  exit 1
fi

# Copy template
mkdir -p "$(dirname "$TARGET_DIR")"
cp -r "$TEMPLATE_DIR" "$TARGET_DIR"

# Replace file content
find "$TARGET_DIR" -type f -exec sed -i "s/template_${PLUGIN_TYPE}/$PACKAGE_NAME/g" {} +
find "$TARGET_DIR" -type f -exec sed -i "s/Template$(echo "$PLUGIN_TYPE" | sed -r 's/(^|_)([a-z])/\U\2/g')/$PACKAGE_CAMEL_CASE/g" {} +

# Rename files and directories
find "$TARGET_DIR" -depth -name "*template_${PLUGIN_TYPE}*" | while read file; do
  new_file=$(echo "$file" | sed "s/template_${PLUGIN_TYPE}/$PACKAGE_NAME/g")
  mv "$file" "$new_file"
done

find "$TARGET_DIR" -depth -name "*Template$(echo "$PLUGIN_TYPE" | sed -r 's/(^|_)([a-z])/\U\2/g')*" | while read file; do
  new_file=$(echo "$file" | sed "s/Template$(echo "$PLUGIN_TYPE" | sed -r 's/(^|_)([a-z])/\U\2/g')/$PACKAGE_CAMEL_CASE/g")
  mv "$file" "$new_file"
done

echo "Package '$PACKAGE_NAME' created successfully at '$TARGET_DIR'."