#!/bin/bash
set -e  # exit on error

cd /app   # your project directory inside container

# --- 1. Verify token exists ---
if [ -z "$GITHUB_TOKEN" ]; then
  echo "Error: GITHUB_TOKEN is not set. Make sure you passed .env into container."
  exit 1
fi

# --- 2. Configure Git remote with token ---
git remote set-url origin https://$GITHUB_TOKEN@github.com/<username>/<repo>.git

# --- 3. Pull latest changes ---
echo "Pulling latest changes..."
git pull origin main

# --- 4. Commit & push local changes (if any) ---
echo "Pushing local changes..."
git add .
git commit -m "Auto update from container" || true   # no-op if no changes
git push origin main

# --- 5. Build project ---
echo "🔨 Building project..."
# Example: if using Make
if [ -f Makefile ]; then
  make
elif [ -f package.json ]; then
  npm install && npm run build
elif [ -f CMakeLists.txt ]; then
  mkdir -p build && cd build && cmake .. && make
else
  echo "⚠️ No known build system found. Add your build commands here."
fi

echo "Build finished successfully!"
