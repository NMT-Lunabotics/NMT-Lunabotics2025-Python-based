#!/bin/bash
set -e

# Git setup
git config --global user.email "benjamin.peterson@student.nmt.edu"
git config --global user.name "benjamin-p15"
git remote add origin git@github.com:NMT-Lunabotics/NMT-Lunabotics2025-Python-based.git 2>/dev/null || true

# Git pull and overwrite local files
git fetch origin
git reset --hard origin/main
git clean -fdx