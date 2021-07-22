#!/usr/bin/env bash

set -euo pipefail

# get release URLs to delete
deletable=$(curl "https://api.github.com/repos/icyphy/lingua-franca/releases/tags/nightly" | jq '.assets|.[]|.url')

for item in $deletable; do
    echo "Will delete: $item"
    curl \
        -X DELETE \
        -u $GITHUB_TOKEN:x-oauth-basic
        -H "Accept: application/vnd.github.v3+json" \
        $item
done
