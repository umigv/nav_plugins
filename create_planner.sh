#!/bin/bash

# Usage: ./create_planner.sh base_name

is_lower_snake_case() {
  [[ "$1" =~ ^[a-z]+(_[a-z]+)*$ ]]
}

if [ -z "$1" ]; then
  echo "Usage: $0 <base_name>"
  exit 1
fi

BASE_NAME=$1

if ! is_lower_snake_case "$BASE_NAME"; then
  echo "Error: Base name '$BASE_NAME' must be in lower_snake_case (e.g., my_planner)."
  exit 1
fi

PACKAGE_NAME="${BASE_NAME}_planner"
PACKAGE_CAMEL_CASE=$(echo "$BASE_NAME" | sed -r 's/(^|_)([a-z])/\U\2/g')Planner
TEMPLATE_DIR="planner/template_planner"
NEW_DIR="planner/$PACKAGE_NAME"

if [ ! -d "$TEMPLATE_DIR" ]; then
  echo "Error: Template directory '$TEMPLATE_DIR' not found."
  exit 1
fi

# Copy
cp -r "$TEMPLATE_DIR" "$NEW_DIR"

# Replace content
find "$NEW_DIR" -type f -exec sed -i "s/template_planner/$PACKAGE_NAME/g" {} +
find "$NEW_DIR" -type f -exec sed -i "s/TemplatePlanner/$PACKAGE_CAMEL_CASE/g" {} +

# Replace file name
find "$NEW_DIR" -depth -name "*template_planner*" | while read file; do
  new_file=$(echo "$file" | sed "s/template_planner/$PACKAGE_NAME/g")
  mv "$file" "$new_file"
done

find "$NEW_DIR" -depth -name "*TemplatePlanner*" | while read file; do
  new_file=$(echo "$file" | sed "s/TemplatePlanner/$PACKAGE_CAMEL_CASE/g")
  mv "$file" "$new_file"
done

echo "Package '$PACKAGE_NAME' created successfully."
