#!/usr/bin/env bash
# Build the "Subsystem graph" panel and install it into Lichtblick desktop. Restart Lichtblick after.
set -euo pipefail
cd "$(dirname "$0")"
npm ci --no-audit --no-fund
npm test
npm run build
version=$(node -p 'require("./package.json").version')
target="$HOME/.lichtblick-suite/extensions/greenhouse.subsystem-graph-$version"
rm -rf "$HOME"/.lichtblick-suite/extensions/greenhouse.subsystem-graph-*
mkdir -p "$target/dist"
cp package.json "$target/"
cp dist/extension.js "$target/dist/"
echo "Installed $target. Restart Lichtblick; add the panel from Add panel > Subsystem graph."
