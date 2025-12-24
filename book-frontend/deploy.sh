#!/bin/bash
# Script to deploy Docusaurus site to GitHub Pages

set -e

if [ -d "build" ]; then
  rm -rf build
fi

npm run build

cd build

git init
git config user.name "github-actions[bot]"
git config user.email "41898282+github-actions[bot]@users.noreply.github.com"
git add .
git commit -m "Deploy to GitHub Pages"

git remote add origin https://github.com/ParizahShaikhh/Physical-AI-and-Humanoid-Robotics-Course-Book1.git
git push -f origin main:gh-pages

echo "Deployment completed successfully!"