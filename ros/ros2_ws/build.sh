#!/bin/bash
REPO_URL="https://github.com/NMT-Lunabotics/NMT-Lunabotics2025-Python-based"
BRANCH="benjamintestbrance

if colcon build; then
    echo "Build successful! Pushing to GitHub..."
    git add .
    git commit -m "Auto-commit: $(date "+%Y-%m-%d %H:%M:%S")"
    git push $REPO_URL $BRANCH
    echo "Done! Pushed to GitHub"
else
    echo "Build failed! Not pushing"
    exit 1
fi' > /usr/local/bin/buildpush

chmod +x /usr/local/bin/buildpush