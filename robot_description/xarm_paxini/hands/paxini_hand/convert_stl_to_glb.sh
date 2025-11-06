#!/bin/bash

folder="meshes"

for file in $folder/*.STL; do 
    echo "Converting $file"
    filename=$(basename "$file" .STL)
    assimp export "$file" "$folder/$filename.glb"
done
